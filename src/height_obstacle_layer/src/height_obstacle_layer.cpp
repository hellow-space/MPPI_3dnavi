#include "height_obstacle_layer/height_obstacle_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "height_obstacle_layer/height_difference.hpp"
#include "height_obstacle_layer/snapshot_policy.hpp"

namespace height_obstacle_layer
{
namespace
{

template<typename T>
T readParameter(
  nav2_costmap_2d::Layer & layer,
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  layer.declareParameter(name, rclcpp::ParameterValue(default_value));
  T value = default_value;
  node->get_parameter(layer.getFullName(name), value);
  return value;
}

bool validGeometry(const grid_map_msgs::msg::GridMap & message, const std::string & layer)
{
  const auto & info = message.info;
  const auto layer_count = std::count(message.layers.begin(), message.layers.end(), layer);
  return !message.header.frame_id.empty() && layer_count == 1 &&
         message.layers.size() == message.data.size() &&
         std::isfinite(info.resolution) && info.resolution > 0.0 &&
         std::isfinite(info.length_x) && info.length_x > 0.0 &&
         std::isfinite(info.length_y) && info.length_y > 0.0 &&
         std::isfinite(info.pose.position.x) && std::isfinite(info.pose.position.y);
}

}  // namespace

bool HeightObstacleLayer::Bounds::valid() const
{
  return min_x <= max_x && min_y <= max_y;
}

void HeightObstacleLayer::Bounds::include(const double x, const double y)
{
  min_x = std::min(min_x, x);
  min_y = std::min(min_y, y);
  max_x = std::max(max_x, x);
  max_y = std::max(max_y, y);
}

void HeightObstacleLayer::Bounds::include(const Bounds & other)
{
  if (!other.valid()) {
    return;
  }
  include(other.min_x, other.min_y);
  include(other.max_x, other.max_y);
}

void HeightObstacleLayer::onInitialize()
{
  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("HeightObstacleLayer parent node expired");
  }

  enabled_ = readParameter(*this, node, "enabled", true);
  map_topic_ = readParameter(*this, node, "map_topic", std::string{});
  elevation_layer_ = readParameter(*this, node, "elevation_layer", std::string{"elevation"});
  jump_radius_ = readParameter(*this, node, "jump_radius", 0.10);
  max_height_jump_ = static_cast<float>(
    readParameter(*this, node, "max_height_jump", 0.30));
  map_timeout_ = readParameter(*this, node, "map_timeout", 1.0);
  unknown_as_obstacle_ = readParameter(*this, node, "unknown_as_obstacle", false);
  publish_height_difference_ =
    readParameter(*this, node, "publish_height_difference", false);
  height_difference_topic_ = readParameter(
    *this, node, "height_difference_topic",
    std::string{"height_difference_map"});

  if (map_topic_.empty() || elevation_layer_.empty() ||
    !std::isfinite(jump_radius_) || jump_radius_ <= 0.0 ||
    !std::isfinite(max_height_jump_) || max_height_jump_ <= 0.0F ||
    !std::isfinite(map_timeout_) || map_timeout_ <= 0.0 ||
    (publish_height_difference_ && height_difference_topic_.empty()))
  {
    throw std::invalid_argument(
            "HeightObstacleLayer requires a map topic and finite positive radius, threshold and timeout");
  }

  matchSize();
  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  resetMaps();

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    map_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&HeightObstacleLayer::mapCallback, this, std::placeholders::_1), options);

  if (publish_height_difference_) {
    height_difference_publisher_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
      height_difference_topic_, rclcpp::QoS(1).best_effort());
  }

  current_ = false;
  RCLCPP_INFO(
    logger_,
    "HeightObstacleLayer: topic=%s layer=%s radius=%.3f threshold=%.3f unknown=%s",
    map_topic_.c_str(), elevation_layer_.c_str(), jump_radius_, max_height_jump_,
    unknown_as_obstacle_ ? "obstacle" : "traversable");
}

void HeightObstacleLayer::activate()
{
  if (height_difference_publisher_) {
    height_difference_publisher_->on_activate();
  }
}

void HeightObstacleLayer::deactivate()
{
  if (height_difference_publisher_) {
    height_difference_publisher_->on_deactivate();
  }
}

void HeightObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMaps();
  if (latest_snapshot_) {
    dirty_bounds_.include(latest_snapshot_->obstacle_bounds);
  }
  current_ = false;
}

bool HeightObstacleLayer::isFresh(const Snapshot & snapshot, const int64_t now_ns) const
{
  if (!isSnapshotTimestampFresh(snapshot.stamp_ns, now_ns, map_timeout_)) {
    return false;
  }
  const double receipt_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - snapshot.received).count();
  return receipt_age <= map_timeout_;
}

void HeightObstacleLayer::mapCallback(
  const grid_map_msgs::msg::GridMap::ConstSharedPtr message)
{
  const std::string expected_frame = layered_costmap_->getGlobalFrameID();
  if (!validGeometry(*message, elevation_layer_) || message->header.frame_id != expected_frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_input_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "HeightObstacleLayer rejected invalid map or frame (expected '%s', got '%s')",
      expected_frame.c_str(), message->header.frame_id.c_str());
    return;
  }

  grid_map::GridMap elevation_map;
  try {
    if (!grid_map::GridMapRosConverter::fromMessage(
        *message, elevation_map, {elevation_layer_}, false, false))
    {
      throw std::runtime_error("GridMap conversion failed");
    }
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_input_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "HeightObstacleLayer map conversion: %s", error.what());
    return;
  }

  auto next = std::make_shared<Snapshot>();
  next->stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
  next->received = std::chrono::steady_clock::now();
  if (!isSnapshotTimestampFresh(next->stamp_ns, clock_->now().nanoseconds(), map_timeout_)) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_input_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "HeightObstacleLayer rejected a zero, future or stale elevation-map timestamp");
    return;
  }

  grid_map::GridMap difference_map({"height_difference"});
  if (publish_height_difference_) {
    difference_map.setGeometry(
      elevation_map.getLength(), elevation_map.getResolution(), elevation_map.getPosition());
    difference_map.setFrameId(elevation_map.getFrameId());
    difference_map.setTimestamp(elevation_map.getTimestamp());
  }

  const std::array<std::pair<int, int>, 9> offsets{{
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1}, {0, 0}, {0, 1},
    {1, -1}, {1, 0}, {1, 1}}};

  for (grid_map::GridMapIterator iterator(elevation_map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position center;
    if (!elevation_map.getPosition(*iterator, center)) {
      continue;
    }

    std::array<float, 9> samples;
    samples.fill(std::numeric_limits<float>::quiet_NaN());
    for (size_t i = 0; i < offsets.size(); ++i) {
      const grid_map::Position position(
        center.x() + offsets[i].first * jump_radius_,
        center.y() + offsets[i].second * jump_radius_);
      if (elevation_map.isInside(position)) {
        samples[i] = elevation_map.atPosition(elevation_layer_, position);
      }
    }

    auto result = classifyNeighborhood(samples, max_height_jump_);
    if (unknown_as_obstacle_ && !result.has_known_height) {
      result.is_obstacle = true;
    }
    if (publish_height_difference_) {
      difference_map.atPosition("height_difference", center) = result.height_difference;
    }
    if (result.is_obstacle) {
      next->obstacles.push_back({center.x(), center.y()});
      next->obstacle_bounds.include(center.x(), center.y());
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t latest_stamp_ns = latest_snapshot_ ? latest_snapshot_->stamp_ns : 0;
    if (!shouldAcceptSnapshot(
        next->stamp_ns, clock_->now().nanoseconds(), map_timeout_, latest_stamp_ns))
    {
      // A stale candidate makes the input unavailable. A fresh duplicate or
      // out-of-order frame is simply ignored and cannot resurrect old terrain.
      if (!isSnapshotTimestampFresh(
          next->stamp_ns, clock_->now().nanoseconds(), map_timeout_))
      {
        latest_input_valid_ = false;
      }
      return;
    }
    if (latest_snapshot_) {
      dirty_bounds_.include(latest_snapshot_->obstacle_bounds);
    }
    dirty_bounds_.include(next->obstacle_bounds);
    latest_snapshot_ = std::move(next);
    latest_input_valid_ = true;
  }

  if (height_difference_publisher_ && height_difference_publisher_->is_activated()) {
    height_difference_publisher_->publish(
      *grid_map::GridMapRosConverter::toMessage(difference_map));
  }
}

void HeightObstacleLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  Bounds update_bounds;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cycle_snapshot_ = latest_snapshot_;
    if (cycle_snapshot_) {
      update_bounds.include(cycle_snapshot_->obstacle_bounds);
    }
    update_bounds.include(dirty_bounds_);
    dirty_bounds_ = Bounds{};
    current_ = latest_input_valid_ && cycle_snapshot_ &&
      isFresh(*cycle_snapshot_, clock_->now().nanoseconds());
  }

  if (!current_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "HeightObstacleLayer is using its last valid obstacle snapshot; elevation map is unavailable or stale");
  }
  if (update_bounds.valid()) {
    const double padding = std::max(jump_radius_, getResolution());
    touch(update_bounds.min_x - padding, update_bounds.min_y - padding, min_x, min_y, max_x, max_y);
    touch(update_bounds.max_x + padding, update_bounds.max_y + padding, min_x, min_y, max_x, max_y);
  }
}

void HeightObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  const int min_i, const int min_j, const int max_i, const int max_j)
{
  if (!enabled_ || !cycle_snapshot_) {
    return;
  }

  if (layered_costmap_->isRolling()) {
    updateOrigin(master_grid.getOriginX(), master_grid.getOriginY());
  }

  const int bounded_min_i = std::max(0, min_i);
  const int bounded_min_j = std::max(0, min_j);
  const int bounded_max_i = std::min(static_cast<int>(getSizeInCellsX()), max_i);
  const int bounded_max_j = std::min(static_cast<int>(getSizeInCellsY()), max_j);
  if (bounded_min_i >= bounded_max_i || bounded_min_j >= bounded_max_j) {
    return;
  }

  resetMap(
    static_cast<unsigned int>(bounded_min_i), static_cast<unsigned int>(bounded_min_j),
    static_cast<unsigned int>(bounded_max_i), static_cast<unsigned int>(bounded_max_j));

  for (const Point & obstacle : cycle_snapshot_->obstacles) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (worldToMap(obstacle.x, obstacle.y, mx, my)) {
      setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  updateWithMax(master_grid, bounded_min_i, bounded_min_j, bounded_max_i, bounded_max_j);
}

}  // namespace height_obstacle_layer

PLUGINLIB_EXPORT_CLASS(
  height_obstacle_layer::HeightObstacleLayer,
  nav2_costmap_2d::Layer)
