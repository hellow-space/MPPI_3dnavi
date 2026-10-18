#pragma once

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace height_obstacle_layer
{

class HeightObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  HeightObstacleLayer() = default;
  ~HeightObstacleLayer() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  struct Bounds
  {
    double min_x{std::numeric_limits<double>::infinity()};
    double min_y{std::numeric_limits<double>::infinity()};
    double max_x{-std::numeric_limits<double>::infinity()};
    double max_y{-std::numeric_limits<double>::infinity()};

    bool valid() const;
    void include(double x, double y);
    void include(const Bounds & other);
  };

  struct Point
  {
    double x{0.0};
    double y{0.0};
  };

  struct Snapshot
  {
    std::vector<Point> obstacles;
    Bounds obstacle_bounds;
    int64_t stamp_ns{0};
    std::chrono::steady_clock::time_point received;
  };

  void mapCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr message);
  bool isFresh(const Snapshot & snapshot, int64_t now_ns) const;

  std::string map_topic_;
  std::string elevation_layer_;
  std::string height_difference_topic_;
  double jump_radius_{0.10};
  float max_height_jump_{0.30F};
  double map_timeout_{1.0};
  bool unknown_as_obstacle_{false};
  bool publish_height_difference_{false};

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr
    height_difference_publisher_;

  mutable std::mutex mutex_;
  std::shared_ptr<const Snapshot> latest_snapshot_;
  std::shared_ptr<const Snapshot> cycle_snapshot_;
  Bounds dirty_bounds_;
  bool latest_input_valid_{false};
};

}  // namespace height_obstacle_layer
