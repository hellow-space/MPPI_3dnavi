#include "custom_path_navigation/path_executor.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "custom_path_navigation/execution_model.hpp"
#include "custom_path_navigation/path_processor.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace custom_path_navigation
{
struct PathExecutor::Impl
{
  using Action = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using Trigger = std_srvs::srv::Trigger;
  using SteadyClock = std::chrono::steady_clock;

  explicit Impl(PathExecutor & node)
  : node(node)
  {
    fixed_frame = stringParameter("fixed_frame", "map");
    robot_frame = stringParameter("robot_frame", "base_link");
    start_tolerance = positiveParameter("start_tolerance", 0.5);
    config.min_raw_spacing = positiveParameter("min_raw_spacing", 0.02);
    config.min_path_length = positiveParameter("min_path_length", 0.20);
    config.sample_spacing = positiveParameter("sample_spacing", 0.05);
    const auto passes = node.declare_parameter<int64_t>("smoothing_passes", 1);
    if (passes < 0 || passes > std::numeric_limits<int>::max()) {
      throw std::invalid_argument("smoothing_passes must be a nonnegative integer");
    }
    config.smoothing_passes = static_cast<int>(passes);
    action_name = stringParameter("follow_path_action", "/follow_path");
    controller_id = stringParameter("controller_id", "FollowPath");
    goal_checker_id = stringParameter("goal_checker_id", "precise_goal_checker");
    action_server_timeout = positiveParameter("action_server_timeout", 2.0);
    cancel_timeout = positiveParameter("cancel_timeout", 2.0);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer, &node, false);
    action = rclcpp_action::create_client<Action>(&node, action_name);
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    preview_pub = node.create_publisher<nav_msgs::msg::Path>("/custom_path/preview_path", retained);
    marker_pub = node.create_publisher<visualization_msgs::msg::Marker>(
      "/custom_path/preview_marker", retained);
    status_pub = node.create_publisher<std_msgs::msg::String>("/custom_path/status", retained);
    raw_sub = node.create_subscription<nav_msgs::msg::Path>(
      "/custom_path/raw_path", rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {receivePath(*message);});
    execute_service = node.create_service<Trigger>(
      "/custom_path/execute",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        execute(*response);
      });
    cancel_service = node.create_service<Trigger>(
      "/custom_path/cancel",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        cancel(*response);
      });
    timer = node.create_wall_timer(std::chrono::milliseconds(10), [this] {checkTimeouts();});
    publishState("waiting for a drawn path");
  }

  double positiveParameter(const std::string & name, double initial)
  {
    const auto value = node.declare_parameter<double>(name, initial);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  std::string stringParameter(const std::string & name, const std::string & initial)
  {
    const auto value = node.declare_parameter<std::string>(name, initial);
    if (value.empty()) {throw std::invalid_argument(name + " must not be empty");}
    return value;
  }

  bool checkStart(const nav_msgs::msg::Path & path, std::string & reason)
  {
    try {
      const auto transform =
        tf_buffer->lookupTransform(fixed_frame, robot_frame, tf2::TimePointZero);
      // Static transforms have a zero timestamp. Dynamic transforms older than
      // the TF cache cannot establish the robot's current position.
      const rclcpp::Time stamp(transform.header.stamp, node.get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 &&
        node.now().nanoseconds() - stamp.nanoseconds() >
        std::chrono::duration_cast<std::chrono::nanoseconds>(tf_buffer->getCacheLength()).count())
      {
        reason = "TF is stale";
        return false;
      }
      const auto & robot = transform.transform.translation;
      const auto & start = path.poses.front().pose.position;
      const auto distance = std::hypot(start.x - robot.x, start.y - robot.y);
      if (!std::isfinite(distance) || distance > start_tolerance) {
        reason = "path start exceeds start_tolerance";
        return false;
      }
      return true;
    } catch (const tf2::TransformException & exception) {
      reason = std::string("TF unavailable: ") + exception.what();
      return false;
    }
  }

  void receivePath(const nav_msgs::msg::Path & message)
  {
    // A fresh stroke supersedes any replacement that has not been dispatched.
    // Keep canceling the old active goal, but require Enter for this candidate.
    const bool replacement_invalidated = model.hasPendingReplacement();
    if (replacement_invalidated) {model.setCandidate(false);}
    raw_candidate = message;
    const auto result = processPath(message, config);
    candidate_valid = result.valid && message.header.frame_id == fixed_frame;
    candidate = candidate_valid ? result.path : nav_msgs::msg::Path();
    candidate.header.frame_id = fixed_frame;
    model.setCandidate(candidate_valid);
    preview_is_dispatched = false;
    preview_red = !candidate_valid;
    std::string reason = candidate_valid ?
      replacement_invalidated ? "new candidate ready; unsent replacement invalidated" :
      "candidate ready" :
      message.header.frame_id != fixed_frame ? "path frame must match fixed_frame" : result.error;
    if (candidate_valid && !checkStart(candidate, reason)) {preview_red = true;}
    preview_pub->publish(candidate);
    publishState(reason);
  }

  void execute(Trigger::Response & response)
  {
    if (!candidate_valid) {
      response.message = "no valid candidate path";
      publishState(response.message);
      return;
    }
    if (model.state() == ExecutionState::SENDING || cancellation_pending) {
      response.message = "goal send or cancellation already pending";
      return;
    }
    if (!checkStart(candidate, response.message)) {
      preview_red = true;
      publishState(response.message);
      return;
    }
    if (!action->action_server_is_ready()) {
      response.message = "FollowPath action server unavailable";
      publishState(response.message);
      return;
    }
    const auto decision = model.requestExecute();
    if (decision == ExecutionDecision::Rejected) {
      response.message = "execution request rejected";
      return;
    }
    response.success = true;
    preview_red = false;
    dispatch_path = candidate;
    response.message = decision == ExecutionDecision::SendGoal ?
      "path admitted for asynchronous dispatch" : "replacement waiting for confirmed cancellation";
    if (decision == ExecutionDecision::SendGoal) {sendGoal();} else {beginCancel();}
    publishState(response.message);
  }

  void sendGoal()
  {
    const auto generation = model.generation();
    Action::Goal goal;
    goal.path = dispatch_path;
    preview_is_dispatched = candidate_valid && candidate == dispatch_path;
    goal.controller_id = controller_id;
    goal.goal_checker_id = goal_checker_id;
    send_pending = true;
    send_timeout_reported = false;
    send_started = SteadyClock::now();
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, generation](GoalHandle::SharedPtr handle) {
        if (generation != model.generation()) {return;}
        send_pending = false;
        goal_handle = handle;
        if (!handle) {
          cancellation_pending = false;
          ++cancel_request_id;
          user_cancel_requested = false;
          model.goalFinished(generation, TerminalResult::Failed);
          publishState("FollowPath goal rejected");
          return;
        }
        model.goalAccepted(generation);
        if (user_cancel_requested && !cancellation_pending) {
          // A timed-out wait for acceptance does not revoke the user's stop
          // intent. The newly received handle allows a fresh cancel attempt.
          model.requestCancel();
          // Preserve a candidate drawn since Esc while stopping the older goal.
          model.setCandidate(candidate_valid);
          beginCancel();
          publishState("late goal acceptance; cancellation requested");
        } else if (cancellation_pending) {
          requestActionCancel(generation);
        } else {
          publishState("goal active");
        }
      };
    options.result_callback = [this, generation](const GoalHandle::WrappedResult & result) {
        if (generation != model.generation()) {return;}
        send_pending = false;
        goal_handle.reset();
        const bool was_canceling = cancellation_pending;
        cancellation_pending = false;
        ++cancel_request_id;
        user_cancel_requested = false;
        if (result.code == rclcpp_action::ResultCode::CANCELED && was_canceling) {
          const auto decision = model.cancelConfirmed(generation);
          if (decision == ExecutionDecision::SendGoal) {
            // Revalidate after the old goal has stopped: the robot may have moved.
            std::string reason;
            if (!checkStart(dispatch_path, reason) || !action->action_server_is_ready()) {
              model.goalFinished(model.generation(), TerminalResult::Failed);
              preview_red = true;
              publishState(reason.empty() ? "FollowPath action server unavailable" : reason);
            } else {
              sendGoal();
              publishState("replacement dispatched after canceled result");
            }
            return;
          }
        } else {
          const auto terminal = result.code == rclcpp_action::ResultCode::SUCCEEDED ?
            TerminalResult::Succeeded : result.code == rclcpp_action::ResultCode::CANCELED ?
            TerminalResult::Canceled : TerminalResult::Failed;
          model.goalFinished(generation, terminal);
        }
        publishState("FollowPath goal finished");
      };
    try {
      action->async_send_goal(goal, options);
    } catch (const std::exception & exception) {
      send_pending = false;
      user_cancel_requested = false;
      model.goalFinished(generation, TerminalResult::Failed);
      publishState(std::string("goal send failed: ") + exception.what());
    }
  }

  void beginCancel()
  {
    ++cancel_request_id;
    cancellation_pending = true;
    cancel_started = SteadyClock::now();
    if (goal_handle) {requestActionCancel(model.generation());}
  }

  void requestActionCancel(uint64_t generation)
  {
    const auto request_id = cancel_request_id;
    try {
      action->async_cancel_goal(
        goal_handle,
        [this, generation,
        request_id](rclcpp_action::Client<Action>::CancelResponse::SharedPtr response) {
          if (generation != model.generation() || request_id != cancel_request_id ||
          !cancellation_pending)
          {
            return;
          }
          if (response->return_code != 0 || response->goals_canceling.empty()) {
            cancelFailed(generation, request_id, "FollowPath cancellation rejected");
          }
          // Accepted cancellation is not a terminal result. Wait for CANCELED.
        });
    } catch (const std::exception & exception) {
      cancelFailed(
        generation, request_id,
        std::string("FollowPath cancellation failed: ") + exception.what());
    }
  }

  void cancelFailed(uint64_t generation, uint64_t request_id, const std::string & reason)
  {
    if (generation != model.generation() || request_id != cancel_request_id ||
      !cancellation_pending)
    {
      return;
    }
    cancellation_pending = false;
    ++cancel_request_id;
    model.goalFinished(generation, TerminalResult::CancelFailed);
    publishState(reason);
  }

  void cancel(Trigger::Response & response)
  {
    user_cancel_requested = user_cancel_requested || send_pending || bool(goal_handle);
    candidate_valid = false;
    raw_candidate = nav_msgs::msg::Path();
    candidate = nav_msgs::msg::Path();
    candidate.header.frame_id = fixed_frame;
    dispatch_path = nav_msgs::msg::Path();
    preview_red = false;
    preview_is_dispatched = false;
    preview_pub->publish(candidate);
    const auto decision = model.requestCancel();
    if (decision == ExecutionDecision::CancelGoal) {beginCancel();}
    response.success = true;
    response.message = cancellation_pending ? "cancellation requested; candidate cleared" :
      "candidate cleared";
    publishState(response.message);
  }

  void checkTimeouts()
  {
    const auto now = SteadyClock::now();
    if (cancellation_pending &&
      std::chrono::duration<double>(now - cancel_started).count() >= cancel_timeout)
    {
      cancelFailed(
        model.generation(), cancel_request_id,
        "FollowPath cancellation timed out; replacement dropped");
    }
    if (send_pending && !send_timeout_reported &&
      std::chrono::duration<double>(now - send_started).count() >= action_server_timeout)
    {
      // A lost goal response does not prove rejection. Retain SENDING occupancy
      // until acceptance/rejection arrives, so retry cannot create a second goal.
      send_timeout_reported = true;
      publishState("FollowPath goal response timed out; awaiting response");
    }
  }

  void publishState(const std::string & reason)
  {
    std_msgs::msg::String status;
    status.data = std::string(stateName(model.state())) + ": " + reason;
    status_pub->publish(status);
    visualization_msgs::msg::Marker marker;
    marker.header = candidate.header;
    marker.header.frame_id = fixed_frame;
    marker.header.stamp = node.now();
    marker.ns = "custom_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = candidate.poses.empty() ? visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.a = 1.0;
    if (preview_red || model.state() == ExecutionState::FAILED) {
      marker.color.r = 1.0;
    } else if (preview_is_dispatched &&
      (model.state() == ExecutionState::ACTIVE || model.state() == ExecutionState::SENDING))
    {
      marker.color.b = 1.0;
    } else if (model.state() == ExecutionState::READY ||
      (candidate_valid && !preview_is_dispatched))
    {
      marker.color.g = 1.0;
    } else {
      marker.color.r = marker.color.g = marker.color.b = 0.5;
    }
    for (const auto & pose : candidate.poses) {marker.points.push_back(pose.pose.position);}
    marker_pub->publish(marker);
  }

  PathExecutor & node;
  ProcessingConfig config;
  ExecutionModel model;
  std::string fixed_frame, robot_frame, action_name, controller_id, goal_checker_id;
  double start_tolerance{}, action_server_timeout{}, cancel_timeout{};
  bool candidate_valid{false}, preview_red{false}, cancellation_pending{false};
  bool user_cancel_requested{false};
  bool preview_is_dispatched{false};
  uint64_t cancel_request_id{0};
  bool send_pending{false}, send_timeout_reported{false};
  SteadyClock::time_point cancel_started, send_started;
  nav_msgs::msg::Path raw_candidate, candidate, dispatch_path;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp_action::Client<Action>::SharedPtr action;
  GoalHandle::SharedPtr goal_handle;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_sub;
  rclcpp::Service<Trigger>::SharedPtr execute_service, cancel_service;
  rclcpp::TimerBase::SharedPtr timer;
};

PathExecutor::PathExecutor(const rclcpp::NodeOptions & options)
: Node("custom_path_executor", options), impl_(std::make_unique<Impl>(*this)) {}
PathExecutor::~PathExecutor() = default;
}  // namespace custom_path_navigation
