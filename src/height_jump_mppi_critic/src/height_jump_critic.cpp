#include "height_jump_mppi_critic/height_jump_critic.hpp"

#include <array>
#include <utility>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>

namespace mppi::critics
{
namespace
{
// A fixed binary rejection encoding for Nav2's cost interface, never a height-dependent cost.
constexpr float kRejected = 1000000.0f;

template<class T>
T parameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, const T & default_value)
{
  if (!node->has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "HeightJump configuration; restart controller node to change";
    node->declare_parameter<T>(name, default_value, descriptor);
  }
  return node->get_parameter(name).get_value<T>();
}

struct PlanarTransform
{
  double x{0}, y{0}, c{1}, s{0};
  std::pair<double, double> apply(double px, double py) const
  {
    return {c * px - s * py + x, s * px + c * py + y};
  }
  std::pair<double, double> inverse(double px, double py) const
  {
    return {c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)};
  }
};

PlanarTransform transform(tf2_ros::Buffer & buffer, const std::string & target,
  const std::string & source, const rclcpp::Time & time)
{
  if (target == source) {return {};}
  const auto msg = buffer.lookupTransform(target, source, time, rclcpp::Duration(0, 0));
  const auto & t = msg.transform.translation;
  const auto & q = msg.transform.rotation;
  if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    throw std::runtime_error("non-finite terrain transform");
  }
  tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
  if (std::abs(rotation.length2() - 1) > 1e-3) {
    throw std::runtime_error("invalid terrain transform rotation");
  }
  double roll, pitch, yaw;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  if (std::abs(roll) > 1e-3 || std::abs(pitch) > 1e-3) {
    throw std::runtime_error("terrain and planning frames must be gravity-aligned");
  }
  return {t.x, t.y, std::cos(yaw), std::sin(yaw)};
}
}  // namespace

void HeightJumpCritic::initialize()
{
  auto node = parent_.lock();
  if (!node) {throw std::runtime_error("HeightJump parent expired");}
  const auto prefix = name_ + ".";
  const auto topic = parameter<std::string>(node, prefix + "map_topic", "");
  const auto layer = parameter<std::string>(node, prefix + "elevation_layer", "elevation");
  config_.jump_radius = parameter<double>(node, prefix + "jump_radius", 0.10);
  config_.max_height_jump = static_cast<float>(
    parameter<double>(node, prefix + "max_height_jump", 0.0));
  config_.unknown_is_traversable =
    parameter<bool>(node, prefix + "unknown_is_traversable", false);
  map_timeout_ = parameter<double>(node, prefix + "map_timeout", 1.0);
  startup_enabled_ = parameter<bool>(node, prefix + "startup_unknown_enabled", false);
  startup_radius_ = parameter<double>(node, prefix + "unknown_ignore_radius", 0.0);
  fixed_frame_ = parameter<std::string>(node, prefix + "fixed_frame", "");
  config_.validate();
  if (topic.empty() || layer.empty() || !std::isfinite(map_timeout_) || map_timeout_ <= 0 ||
    !std::isfinite(startup_radius_) || startup_radius_ < 0 ||
    (startup_enabled_ && (fixed_frame_.empty() || startup_radius_ <= 0)))
  {
    throw std::invalid_argument("HeightJump requires map_topic, valid timing and explicit startup frame/radius");
  }
  source_ = std::make_unique<height_jump::TerrainMapSource>(node, topic, layer);
  startup_ = {};
  last_time_ns_ = 0;
  clock_invalid_ = false;
  RCLCPP_INFO(
    logger_,
    "HeightJump binary feasibility: layer=%s threshold=%.3f unknown=%s startup_exemption=%s",
    layer.c_str(), config_.max_height_jump,
    config_.unknown_is_traversable ? "traversable" : "blocked",
    startup_enabled_ ? "enabled" : "disabled");
}

void HeightJumpCritic::score(CriticData & data)
{
  if (!enabled_ || data.fail_flag) {return;}
  auto node = parent_.lock();
  if (!node || !source_) {data.fail_flag = true; return;}
  const auto now = node->now();
  if (last_time_ns_ != 0 && now.nanoseconds() < last_time_ns_) {clock_invalid_ = true;}
  last_time_ns_ = now.nanoseconds();
  const auto map = source_->snapshot();
  if (clock_invalid_ || !map || !map->fresh(now.nanoseconds(), map_timeout_)) {
    data.fail_flag = true;
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000,
      "HeightJump unavailable: missing/invalid/stale map or clock reset; startup exemption not applied");
    return;
  }
  const auto & trajectories = data.trajectories;
  const size_t count = trajectories.x.shape(0), steps = trajectories.x.shape(1);
  const auto frame = costmap_ros_->getGlobalFrameID();
  const auto & pose = data.state.pose;
  if (count == 0 || steps == 0 || trajectories.y.shape() != trajectories.x.shape() ||
    data.costs.shape(0) != count || pose.header.frame_id != frame ||
    !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y))
  {
    data.fail_flag = true; return;
  }
  PlanarTransform to_map, to_fixed;
  try {
    const rclcpp::Time stamp(pose.header.stamp, node->get_clock()->get_clock_type());
    if (stamp.nanoseconds() <= 0) {throw std::runtime_error("missing robot pose timestamp");}
    to_map = transform(*costmap_ros_->getTfBuffer(), map->map.getFrameId(), frame, stamp);
    if (startup_enabled_) {
      to_fixed = transform(*costmap_ros_->getTfBuffer(), fixed_frame_, frame, stamp);
    }
  } catch (const std::exception & ex) {
    data.fail_flag = true;
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000, "HeightJump TF: %s", ex.what());
    return;
  }
  if (startup_enabled_ && !startup_.captured()) {
    const auto p = to_fixed.apply(pose.pose.position.x, pose.pose.position.y);
    if (!startup_.capture(p.first, p.second)) {data.fail_flag = true; return;}
    RCLCPP_INFO(logger_, "HeightJump fixed startup region: frame=%s x=%.3f y=%.3f radius=%.3f",
      fixed_frame_.c_str(), p.first, p.second, startup_radius_);
  }
  const auto query = [&map](double x, double y) {return map->query(x, y);};
  const auto exempt = [&](double x, double y) {
      if (!startup_enabled_) {return false;}
      const auto p = to_map.inverse(x, y);
      const auto fixed = to_fixed.apply(p.first, p.second);
      return startup_.contains(fixed.first, fixed.second, startup_radius_);
    };
  std::array<size_t, 5> decisions{};
  for (size_t k = 0; k < count; ++k) {
    auto decision = height_jump::Decision::Pass;
    for (size_t t = 0; t < steps; ++t) {
      const auto p = to_map.apply(trajectories.x(k, t), trajectories.y(k, t));
      decision = height_jump::evaluate(p.first, p.second, config_, query, exempt).decision;
      if (decision != height_jump::Decision::Pass) {break;}
    }
    ++decisions[static_cast<size_t>(decision)];
    if (decision != height_jump::Decision::Pass) {data.costs(k) += kRejected;}
  }
  if (decisions[0] == 0) {data.fail_flag = true;}
  RCLCPP_DEBUG_THROTTLE(logger_, *node->get_clock(), 1000,
    "HeightJump pass=%zu height=%zu unknown=%zu outside=%zu invalid=%zu",
    decisions[0], decisions[1], decisions[2], decisions[3], decisions[4]);
}
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::HeightJumpCritic, mppi::critics::CriticFunction)
