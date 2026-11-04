#pragma once

#include <chrono>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "height_jump_mppi_critic/evaluator.hpp"

namespace height_jump
{
struct TerrainSnapshot
{
  grid_map::GridMap map;
  std::string layer;
  std::chrono::steady_clock::time_point received;
  static std::shared_ptr<const TerrainSnapshot> fromMessage(
    const grid_map_msgs::msg::GridMap & msg, const std::string & layer, std::string & error);
  Sample query(double x, double y) const;
  bool fresh(int64_t now_ns, double timeout_seconds) const;
};

// Prepared once per score call. Own the immutable snapshot so cached matrix
// access stays valid even when the source publishes a replacement map.
class TerrainQuery
{
public:
  explicit TerrainQuery(std::shared_ptr<const TerrainSnapshot> snapshot);

  Sample operator()(double x, double y) const
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {return {SampleState::Outside, 0};}
    const auto & map = snapshot_->map;
    const grid_map::Position point(x, y);
    // Keep this precheck: getIndex converts to integer before checking containment.
    if (!map.isInside(point)) {return {SampleState::Outside, 0};}
    grid_map::Index index;
    if (!map.getIndex(point, index) ||
      (index < grid_map::Index::Zero()).any() || (index >= map.getSize()).any())
    {
      return {SampleState::Outside, 0};
    }
    const float z = (*elevation_)(index.x(), index.y());
    return {std::isfinite(z) ? SampleState::Known : SampleState::Unknown, z};
  }

  std::array<Sample, 9> neighborhood(double x, double y, double radius) const
  {
    std::array<Sample, 9> samples;
    samples.fill({SampleState::Outside, 0});
    if (!geometry_valid_ || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(radius) || radius <= 0)
    {
      return samples;
    }
    std::array<int, 3> rows, columns;
    for (int offset = -1; offset <= 1; ++offset) {
      rows[offset + 1] = coordinateIndex(x + offset * radius, 0);
      columns[offset + 1] = coordinateIndex(y + offset * radius, 1);
    }
    size_t n = 0;
    for (int row : rows) {
      for (int column : columns) {
        if (row >= 0 && column >= 0) {
          const float z = (*elevation_)(row, column);
          samples[n] = {std::isfinite(z) ? SampleState::Known : SampleState::Unknown, z};
        }
        ++n;
      }
    }
    return samples;
  }

private:
  int coordinateIndex(double coordinate, int axis) const
  {
    if (!std::isfinite(coordinate)) {return -1;}
    // Match GridMap 2.0.1 checkIfPositionWithinMap's arithmetic order.
    const double position = -(coordinate - center_[axis] - half_length_[axis]);
    if (!(position >= 0 && position < length_[axis])) {return -1;}
    // getIndexFromPosition subtracts half-length BEFORE map center. Do not
    // reassociate these expressions or replace division with a reciprocal:
    // their rounding determines which side of an exact grid line is sampled.
    const double value = -((coordinate - half_length_[axis] - center_[axis]) / resolution_);
    if (!std::isfinite(value) || value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max())
    {
      return -1;
    }
    int index = static_cast<int>(value);
    if (wrap_indices_) {
      int64_t wrapped = static_cast<int64_t>(index) + start_[axis];
      if (wrapped < 0 || wrapped >= size_[axis]) {
        wrapped %= size_[axis];
        if (wrapped < 0) {wrapped += size_[axis];}
      }
      index = static_cast<int>(wrapped);
    }
    return index >= 0 && index < size_[axis] ? index : -1;
  }

  std::shared_ptr<const TerrainSnapshot> snapshot_;
  const grid_map::Matrix * elevation_;
  grid_map::Position center_;
  grid_map::Length length_, half_length_;
  grid_map::Size size_;
  grid_map::Index start_;
  double resolution_;
  bool wrap_indices_, geometry_valid_;
};

class TerrainMapSource
{
public:
  TerrainMapSource(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic, const std::string & layer);
  std::shared_ptr<const TerrainSnapshot> snapshot() const;

private:
  struct Cache
  {
    std::mutex mutex;
    std::shared_ptr<const TerrainSnapshot> latest;
  };
  // Callback owns cache, not this: safe even if a queued callback outlives the critic.
  std::shared_ptr<Cache> cache_{std::make_shared<Cache>()};
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
};
}  // namespace height_jump
