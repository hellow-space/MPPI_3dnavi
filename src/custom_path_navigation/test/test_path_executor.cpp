#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "custom_path_navigation/path_executor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;
using FollowPath = nav2_msgs::action::FollowPath;
using Trigger = std_srvs::srv::Trigger;
using ServerHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

// Real Action protocol services with explicit response gates. Deferred service
// responses let tests order network events without blocking executor callbacks.
class DeferredFollowPathServer
{
public:
  using SendGoal = FollowPath::Impl::SendGoalService;
  using GetResult = FollowPath::Impl::GetResultService;
  using CancelGoal = FollowPath::Impl::CancelGoalService;
  using Header = std::shared_ptr<rmw_request_id_t>;

  explicit DeferredFollowPathServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    send_ = node->create_service<SendGoal>(
      "/follow_path/_action/send_goal",
      [this](Header header, SendGoal::Request::SharedPtr request) {
        goals.push_back({header, request, false, false});
      });
    result_ = node->create_service<GetResult>(
      "/follow_path/_action/get_result",
      [this](Header header, GetResult::Request::SharedPtr request) {
        results.push_back({header, request});
      });
    cancel_ = node->create_service<CancelGoal>(
      "/follow_path/_action/cancel_goal",
      [this](Header header, CancelGoal::Request::SharedPtr request) {
        cancellations.push_back({header, request});
      });
    feedback_ = node->create_publisher<FollowPath::Impl::FeedbackMessage>(
      "/follow_path/_action/feedback", 10);
    status_ = node->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/follow_path/_action/status", rclcpp::QoS(1).reliable().transient_local());
  }

  void accept(std::size_t index)
  {
    auto & goal = goals.at(index);
    goal.accepted = true;
    SendGoal::Response response;
    response.accepted = true;
    response.stamp = node_->now();
    send_->send_response(*goal.header, response);
    std::size_t active = 0;
    for (const auto & entry : goals) {if (entry.accepted && !entry.finished) {++active;}}
    max_active = std::max(max_active, active);
  }

  void answerCancellation(std::size_t index, bool accepted)
  {
    auto & cancellation = cancellations.at(index);
    CancelGoal::Response response;
    response.return_code = accepted ? CancelGoal::Response::ERROR_NONE :
      CancelGoal::Response::ERROR_REJECTED;
    if (accepted) {response.goals_canceling.push_back(cancellation.request->goal_info);}
    cancel_->send_response(*cancellation.header, response);
  }

  void finishCanceled(std::size_t index)
  {
    auto & goal = goals.at(index);
    goal.finished = true;
    for (const auto & result : results) {
      if (result.request->goal_id == goal.request->goal_id) {
        GetResult::Response response;
        response.status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
        result_->send_response(*result.header, response);
        return;
      }
    }
    FAIL() << "test must await the GetResult request before finishing a goal";
  }

  struct GoalEntry
  {
    Header header;
    SendGoal::Request::SharedPtr request;
    bool accepted;
    bool finished;
  };
  struct ResultEntry {Header header; GetResult::Request::SharedPtr request;};
  struct CancelEntry {Header header; CancelGoal::Request::SharedPtr request;};
  std::vector<GoalEntry> goals;
  std::vector<ResultEntry> results;
  std::vector<CancelEntry> cancellations;
  std::size_t max_active{0};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<SendGoal>::SharedPtr send_;
  rclcpp::Service<GetResult>::SharedPtr result_;
  rclcpp::Service<CancelGoal>::SharedPtr cancel_;
  rclcpp::Publisher<FollowPath::Impl::FeedbackMessage>::SharedPtr feedback_;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_;
};

class PathExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    observer_ = std::make_shared<rclcpp::Node>("path_executor_test");
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter("cancel_timeout", 0.35),
      rclcpp::Parameter("action_server_timeout", 0.35)});
    node_ = std::make_shared<custom_path_navigation::PathExecutor>(options);
    executor_->add_node(observer_);
    executor_->add_node(node_);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    raw_ = observer_->create_publisher<nav_msgs::msg::Path>("/custom_path/raw_path", 1);
    preview_ = observer_->create_subscription<nav_msgs::msg::Path>(
      "/custom_path/preview_path", qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {
        last_preview_ = *message;
        ++preview_count_;
      });
    status_ = observer_->create_subscription<std_msgs::msg::String>(
      "/custom_path/status", qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {last_status_ = message->data;});
    marker_ = observer_->create_subscription<visualization_msgs::msg::Marker>(
      "/custom_path/preview_marker", qos,
      [this](visualization_msgs::msg::Marker::ConstSharedPtr message) {last_marker_ = *message;});
    twists_.push_back(
      observer_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr) {++twist_count_;}));
    twists_.push_back(
      observer_->create_subscription<geometry_msgs::msg::Twist>(
        "/command/cmd_twist", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr) {++twist_count_;}));
    execute_ = observer_->create_client<Trigger>("/custom_path/execute");
    cancel_ = observer_->create_client<Trigger>("/custom_path/cancel");
    ASSERT_TRUE(wait([this] {return raw_->get_subscription_count() > 0;}));
  }

  void TearDown() override
  {
    EXPECT_EQ(twist_count_, 0U);
    EXPECT_LE(max_active_, 1U);
    if (deferred_) {EXPECT_LE(deferred_->max_active, 1U);}
    if (node_) {executor_->remove_node(node_);}
    if (observer_) {executor_->remove_node(observer_);}
    node_.reset();
    broadcaster_.reset();
    server_.reset();
    timer_.reset();
    handles_.clear();
    deferred_.reset();
    observer_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  bool wait(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 2500ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      executor_->spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    wait([] {return false;}, duration);
  }

  void startServer()
  {
    server_ = rclcpp_action::create_server<FollowPath>(
      observer_, "/follow_path",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal>) {
        return reject_goals_ ? rclcpp_action::GoalResponse::REJECT :
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<ServerHandle>) {
        ++cancel_requests_;
        return reject_cancel_ ? rclcpp_action::CancelResponse::REJECT :
        rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<ServerHandle> handle) {
        handles_.push_back(handle);
        std::size_t active = 0;
        for (const auto & goal : handles_) {
          if (goal->is_active() || goal->is_canceling()) {++active;}
        }
        max_active_ = std::max(max_active_, active);
      });
    timer_ = observer_->create_wall_timer(
      5ms, [this] {
        if (finish_cancellation_) {
          for (const auto & handle : handles_) {
            if (handle->is_canceling()) {
              handle->canceled(std::make_shared<FollowPath::Result>());
            }
          }
        }
      });
    spinFor(150ms);
  }

  void addTransform(double x = 0.0, double y = 0.0)
  {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(observer_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = observer_->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(transform);
    spinFor(150ms);
  }

  nav_msgs::msg::Path path(double start = 0.0, std::string frame = "map")
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = frame;
    message.header.stamp = observer_->now();
    for (const auto & xy : std::vector<std::pair<double, double>>{
      {start, 0.0}, {start + 0.25, 0.3}, {start + 0.8, 0.4}})
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = xy.first;
      pose.pose.position.y = xy.second;
      pose.pose.position.z = 4.0;
      message.poses.push_back(pose);
    }
    return message;
  }

  void publish(const nav_msgs::msg::Path & message)
  {
    const auto count = preview_count_;
    raw_->publish(message);
    ASSERT_TRUE(wait([this, count] {return preview_count_ > count;}));
    spinFor(20ms);
  }

  Trigger::Response::SharedPtr call(const rclcpp::Client<Trigger>::SharedPtr & client)
  {
    if (!client->wait_for_service(1s)) {return nullptr;}
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (!wait([&future] {return future.wait_for(0s) == std::future_status::ready;})) {
      return nullptr;
    }
    return future.get();
  }

  void startGoal()
  {
    startServer();
    addTransform();
    publish(path());
    auto response = call(execute_);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;
    ASSERT_TRUE(
      wait(
        [this] {
          return handles_.size() == 1U && last_status_.find("ACTIVE") == 0;
        }));
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr observer_;
  std::shared_ptr<custom_path_navigation::PathExecutor> node_;
  rclcpp_action::Server<FollowPath>::SharedPtr server_;
  std::unique_ptr<DeferredFollowPathServer> deferred_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr preview_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> twists_;
  rclcpp::Client<Trigger>::SharedPtr execute_;
  rclcpp::Client<Trigger>::SharedPtr cancel_;
  nav_msgs::msg::Path last_preview_;
  visualization_msgs::msg::Marker last_marker_;
  std::string last_status_;
  std::vector<std::shared_ptr<ServerHandle>> handles_;
  std::size_t preview_count_{0}, twist_count_{0}, max_active_{0}, cancel_requests_{0};
  bool reject_goals_{false}, reject_cancel_{false}, finish_cancellation_{true};
};

TEST_F(PathExecutorTest, InvalidRawPathReplacesAndInvalidatesPreviousCandidate)
{
  startServer();
  addTransform();
  publish(path());
  EXPECT_EQ(last_status_.find("READY"), 0U);
  auto invalid = path();
  invalid.poses.resize(1);
  publish(invalid);
  auto response = call(execute_);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_TRUE(last_preview_.poses.empty());
  EXPECT_TRUE(handles_.empty());
  EXPECT_EQ(last_status_.find("FAILED"), 0U);
}

TEST_F(PathExecutorTest, RejectsWrongFrameAndMissingTransform)
{
  startServer();
  publish(path(0.0, "odom"));
  EXPECT_FALSE(call(execute_)->success);
  publish(path());
  EXPECT_FALSE(call(execute_)->success);
  EXPECT_NE(last_status_.find("TF"), std::string::npos);
  EXPECT_TRUE(handles_.empty());
  EXPECT_GT(last_marker_.color.r, last_marker_.color.g);
}

TEST_F(PathExecutorTest, AcceptsExactlyHalfMeterStartAndRejectsBeyondBoundary)
{
  startServer();
  addTransform();
  publish(path(0.5));
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return handles_.size() == 1U;}));
  handles_[0]->succeed(std::make_shared<FollowPath::Result>());
  ASSERT_TRUE(wait([this] {return last_status_.find("SUCCEEDED") == 0;}));
  publish(path(0.500001));
  EXPECT_FALSE(call(execute_)->success);
  EXPECT_EQ(handles_.size(), 1U);
  EXPECT_GT(last_marker_.color.r, last_marker_.color.g);
}

TEST_F(PathExecutorTest, ExecuteFailsWhenActionServerIsUnavailable)
{
  addTransform();
  publish(path());
  EXPECT_FALSE(call(execute_)->success);
  EXPECT_EQ(last_status_.find("READY"), 0U);
}

TEST_F(PathExecutorTest, SendsProcessedPathControllerAndGoalCheckerIds)
{
  startGoal();
  const auto goal = handles_.front()->get_goal();
  EXPECT_EQ(goal->controller_id, "FollowPath");
  EXPECT_EQ(goal->goal_checker_id, "precise_goal_checker");
  EXPECT_GT(goal->path.poses.size(), 3U);
  EXPECT_EQ(goal->path, last_preview_);
  for (const auto & pose : goal->path.poses) {
    EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.0);
  }
  EXPECT_GT(last_marker_.color.b, last_marker_.color.r);
}

TEST_F(PathExecutorTest, FollowPathTerminalPoseEqualsPreviewTerminalPose)
{
  startGoal();
  EXPECT_EQ(handles_.front()->get_goal()->path.poses.back(), last_preview_.poses.back());
  EXPECT_DOUBLE_EQ(last_preview_.poses.back().pose.position.x, 0.8);
  EXPECT_DOUBLE_EQ(last_preview_.poses.back().pose.position.y, 0.4);
}

TEST_F(PathExecutorTest, CancelServiceCancelsActiveGoalAndClearsCandidate)
{
  startGoal();
  EXPECT_TRUE(call(cancel_)->success);
  ASSERT_TRUE(wait([this] {return last_status_.find("CANCELED") == 0;}));
  EXPECT_FALSE(call(execute_)->success);
  EXPECT_TRUE(last_preview_.poses.empty());
  EXPECT_EQ(cancel_requests_, 1U);
}

TEST_F(PathExecutorTest, ReplacementCancelsOldGoalBeforeSendingOneNewGoal)
{
  startGoal();
  finish_cancellation_ = false;
  publish(path(0.1));
  EXPECT_EQ(cancel_requests_, 0U);
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return handles_[0]->is_canceling();}));
  EXPECT_FALSE(call(execute_)->success);
  EXPECT_EQ(handles_.size(), 1U);
  finish_cancellation_ = true;
  ASSERT_TRUE(
    wait(
      [this] {
        return handles_.size() == 2U && last_status_.find("ACTIVE") == 0;
      }));
  EXPECT_EQ(cancel_requests_, 1U);
  EXPECT_DOUBLE_EQ(handles_[1]->get_goal()->path.poses.front().pose.position.x, 0.1);
}

TEST_F(PathExecutorTest, CancelFailureDoesNotSendReplacementGoal)
{
  startGoal();
  reject_cancel_ = true;
  publish(path(0.1));
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return last_status_.find("FAILED") == 0;}));
  EXPECT_EQ(handles_.size(), 1U);
  EXPECT_TRUE(handles_[0]->is_executing());
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return cancel_requests_ == 2U;}));
  EXPECT_EQ(handles_.size(), 1U);
}

TEST_F(PathExecutorTest, LateOldResultDoesNotChangeReplacementStatus)
{
  startGoal();
  publish(path(0.1));
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(
    wait(
      [this] {
        return handles_.size() == 2U && last_status_.find("ACTIVE") == 0;
      }));
  spinFor(150ms);
  EXPECT_EQ(last_status_.find("ACTIVE"), 0U);
  EXPECT_TRUE(handles_[1]->is_executing());
}

TEST_F(PathExecutorTest, CancelTimeoutDoesNotSendReplacementEvenAfterLateCancellation)
{
  startGoal();
  finish_cancellation_ = false;
  publish(path(0.1));
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return last_status_.find("FAILED") == 0;}));
  EXPECT_EQ(handles_.size(), 1U);
  finish_cancellation_ = true;
  spinFor(150ms);
  EXPECT_EQ(handles_.size(), 1U);
}

TEST_F(PathExecutorTest, InvalidCandidateDuringCancellationPreventsReplacement)
{
  startGoal();
  finish_cancellation_ = false;
  publish(path(0.1));
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return handles_[0]->is_canceling();}));
  auto invalid = path();
  invalid.poses.clear();
  publish(invalid);
  finish_cancellation_ = true;
  spinFor(150ms);
  EXPECT_EQ(handles_.size(), 1U);
  EXPECT_FALSE(call(execute_)->success);
}

TEST_F(PathExecutorTest, GoalRejectionAndAbortPublishFailedStatus)
{
  startServer();
  addTransform();
  reject_goals_ = true;
  publish(path());
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return last_status_.find("FAILED") == 0;}));
  EXPECT_TRUE(handles_.empty());
  reject_goals_ = false;
  EXPECT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return handles_.size() == 1U;}));
  handles_[0]->abort(std::make_shared<FollowPath::Result>());
  EXPECT_TRUE(wait([this] {return last_status_.find("FAILED") == 0;}));
}

TEST_F(PathExecutorTest, RejectsInvalidParametersAtStartup)
{
  for (const auto & parameter : std::vector<rclcpp::Parameter>{
    {"start_tolerance", 0.0}, {"sample_spacing", -1.0},
    {"cancel_timeout", 0.0}, {"action_server_timeout", -1.0}, {"smoothing_passes", -1}})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    options.arguments({"--ros-args", "-r", "__node:=invalid_parameters"});
    EXPECT_THROW(
      std::make_shared<custom_path_navigation::PathExecutor>(
        options), std::invalid_argument);
  }
}

TEST_F(PathExecutorTest, EscapeBeforeDelayedAcceptanceStillCancelsAfterTimeout)
{
  deferred_ = std::make_unique<DeferredFollowPathServer>(observer_);
  addTransform();
  publish(path());
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return deferred_->goals.size() == 1U;}));
  ASSERT_TRUE(call(cancel_)->success);
  ASSERT_TRUE(wait([this] {return last_status_.find("FAILED") == 0;}));
  deferred_->accept(0);
  ASSERT_TRUE(wait([this] {return deferred_->results.size() == 1U;}));
  ASSERT_TRUE(wait([this] {return deferred_->cancellations.size() == 1U;}));
  EXPECT_EQ(
    deferred_->cancellations[0].request->goal_info.goal_id,
    deferred_->goals[0].request->goal_id);
  deferred_->answerCancellation(0, true);
  deferred_->finishCanceled(0);
  ASSERT_TRUE(wait([this] {return last_status_.find("CANCELED") == 0;}));
  EXPECT_TRUE(deferred_->goals[0].finished);
  EXPECT_TRUE(last_preview_.poses.empty());
  EXPECT_FALSE(call(execute_)->success);
}

TEST_F(PathExecutorTest, CandidateChangeDuringCancellationInvalidatesPendingReplacement)
{
  startGoal();
  finish_cancellation_ = false;
  publish(path(0.1));
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return handles_[0]->is_canceling();}));
  publish(path(0.2));
  const auto newest_preview = last_preview_;
  EXPECT_GT(last_marker_.color.g, last_marker_.color.b);
  ASSERT_FALSE(last_marker_.points.empty());
  EXPECT_EQ(last_marker_.points.front(), newest_preview.poses.front().pose.position);
  finish_cancellation_ = true;
  ASSERT_TRUE(
    wait(
      [this] {
        return last_status_.find("CANCELED") == 0 || handles_.size() == 2U;
      }));
  EXPECT_EQ(handles_.size(), 1U) << "candidate C must invalidate unsent replacement B";
  EXPECT_EQ(last_preview_, newest_preview);
  EXPECT_GT(last_marker_.color.g, last_marker_.color.b);
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(
    wait(
      [this] {
        return handles_.size() == 2U && last_status_.find("ACTIVE") == 0;
      }));
  EXPECT_EQ(handles_[1]->get_goal()->path, newest_preview);
  EXPECT_EQ(last_preview_, newest_preview);
  EXPECT_GT(last_marker_.color.b, last_marker_.color.g);
}

TEST_F(PathExecutorTest, NewCandidateDuringActiveGoalIsGreenUntilExplicitExecution)
{
  startGoal();
  publish(path(0.1));
  EXPECT_GT(last_marker_.color.g, last_marker_.color.b);
  const auto newest_preview = last_preview_;
  EXPECT_EQ(handles_.size(), 1U);
  EXPECT_EQ(cancel_requests_, 0U);
  EXPECT_DOUBLE_EQ(handles_[0]->get_goal()->path.poses.front().pose.position.x, 0.0);
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(
    wait(
      [this] {
        return handles_.size() == 2U && last_status_.find("ACTIVE") == 0;
      }));
  EXPECT_EQ(handles_[1]->get_goal()->path, newest_preview);
  EXPECT_EQ(last_preview_, newest_preview);
  EXPECT_GT(last_marker_.color.b, last_marker_.color.g);
}

TEST_F(PathExecutorTest, LateOldCancelRejectionCannotDropNewReplacementAttempt)
{
  deferred_ = std::make_unique<DeferredFollowPathServer>(observer_);
  addTransform();
  publish(path());
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return deferred_->goals.size() == 1U;}));
  deferred_->accept(0);
  ASSERT_TRUE(
    wait(
      [this] {
        return deferred_->results.size() == 1U && last_status_.find("ACTIVE") == 0;
      }));
  publish(path(0.1));
  const auto replacement = last_preview_;
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return deferred_->cancellations.size() == 1U;}));
  ASSERT_TRUE(
    wait(
      [this] {
        return last_status_.find(
          "cancellation timed out") != std::string::npos;
      }));
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return deferred_->cancellations.size() == 2U;}));
  EXPECT_EQ(
    deferred_->cancellations[0].request->goal_info.goal_id,
    deferred_->cancellations[1].request->goal_info.goal_id);
  deferred_->answerCancellation(1, true);
  deferred_->answerCancellation(0, false);
  // Drain the ordered CancelGoal replies before delivering the terminal result.
  spinFor(80ms);
  EXPECT_EQ(last_status_.find("cancellation rejected"), std::string::npos);
  deferred_->finishCanceled(0);
  ASSERT_TRUE(wait([this] {return deferred_->goals.size() == 2U;}));
  EXPECT_EQ(deferred_->goals[1].request->goal.path, replacement);
  deferred_->accept(1);
  ASSERT_TRUE(wait([this] {return last_status_.find("ACTIVE") == 0;}));
  EXPECT_EQ(last_preview_, replacement);
  EXPECT_GT(last_marker_.color.b, last_marker_.color.g);
}

TEST_F(PathExecutorTest, CandidateDrawnAfterEscapeSurvivesLateAcceptanceCancellation)
{
  deferred_ = std::make_unique<DeferredFollowPathServer>(observer_);
  addTransform();
  publish(path());
  ASSERT_TRUE(call(execute_)->success);
  ASSERT_TRUE(wait([this] {return deferred_->goals.size() == 1U;}));
  ASSERT_TRUE(call(cancel_)->success);
  ASSERT_TRUE(wait([this] {return last_status_.find("FAILED") == 0;}));
  publish(path(0.1));
  const auto new_candidate = last_preview_;
  ASSERT_FALSE(new_candidate.poses.empty());
  EXPECT_EQ(deferred_->goals.size(), 1U);
  deferred_->accept(0);
  ASSERT_TRUE(
    wait(
      [this] {
        return deferred_->results.size() == 1U && deferred_->cancellations.size() == 1U;
      }));
  deferred_->answerCancellation(0, true);
  deferred_->finishCanceled(0);
  ASSERT_TRUE(wait([this] {return last_status_.find("CANCELED") == 0;}));
  EXPECT_EQ(last_preview_, new_candidate);
  EXPECT_GT(last_marker_.color.g, last_marker_.color.b);
  EXPECT_EQ(deferred_->goals.size(), 1U) << "new B must still wait for its own Enter";
  const auto response = call(execute_);
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(wait([this] {return deferred_->goals.size() == 2U;}));
  EXPECT_EQ(deferred_->goals[1].request->goal.path, new_candidate);
  EXPECT_DOUBLE_EQ(deferred_->goals[1].request->goal.path.poses.front().pose.position.x, 0.1);
  deferred_->accept(1);
  ASSERT_TRUE(wait([this] {return last_status_.find("ACTIVE") == 0;}));
  EXPECT_EQ(last_preview_, new_candidate);
  EXPECT_GT(last_marker_.color.b, last_marker_.color.g);
}
