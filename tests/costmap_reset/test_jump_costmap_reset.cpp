#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

// Exercise the shipped Jump YAML against the installed Nav2 implementation.
// No controller, velocity publisher, sensor input, or robot connection is used.
int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: test_jump_costmap_reset NAV2_PARAMS_YAML\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    const auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", argv[1]});
    auto node = std::make_shared<nav2_util::LifecycleNode>(
      "local_costmap", "/local_costmap", options);
    const auto plugins = node->declare_parameter<std::vector<std::string>>("plugins");
    if (plugins != std::vector<std::string>{"inflation_layer"} ||
      node->declare_parameter<std::string>("inflation_layer.plugin") !=
      "nav2_costmap_2d::InflationLayer")
    {
      throw std::runtime_error("test expects the sensor-free Jump placeholder configuration");
    }

    nav2_costmap_2d::LayeredCostmap map("map", true, false);
    map.resizeMap(140, 140, 0.05, -3.5, -3.5);
    auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto layer = std::make_shared<nav2_costmap_2d::InflationLayer>();
    map.addPlugin(layer);
    layer->initialize(&map, "inflation_layer", buffer.get(), node, group);
    std::vector<geometry_msgs::msg::Point> footprint;
    if (!nav2_costmap_2d::makeFootprintFromString(
        node->declare_parameter<std::string>("footprint"), footprint))
    {
      throw std::runtime_error("invalid configured robot footprint");
    }
    map.setFootprint(footprint);
    map.updateMap(0, 0, 0);
    if (!map.isCurrent()) {
      throw std::runtime_error("Jump costmap is not ready at startup");
    }

    // A stationary robot must recover too: movement cannot be required to make
    // the map ready, because the controller waits for readiness before moving.
    for (int reset = 0; reset < 3; ++reset) {
      map.getCostmap()->resetMap(0, 0, 140, 140);
      layer->reset();
      for (int update = 0; update < 20; ++update) {
        map.updateMap(0, 0, 0);
      }
      if (!map.isCurrent()) {
        throw std::runtime_error(
                "Jump costmap never becomes ready after clear/reset at a stationary pose");
      }
    }
    map.updateMap(1.0, -0.7, 0.4);
    if (!map.isCurrent()) {
      throw std::runtime_error("Jump costmap is not ready after rolling-window movement");
    }
    auto * costs = map.getCostmap();
    for (unsigned int y = 0; y < costs->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < costs->getSizeInCellsX(); ++x) {
        if (costs->getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {
          throw std::runtime_error("sensor-free Jump costmap introduced a non-free cell");
        }
      }
    }
    std::cout << "PASS: Jump YAML stays ready through 3 resets and window movement; "
              << "all cells remain free\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
