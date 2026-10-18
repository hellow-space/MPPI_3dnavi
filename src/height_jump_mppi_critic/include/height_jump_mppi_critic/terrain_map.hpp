#pragma once

#include <chrono>
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
