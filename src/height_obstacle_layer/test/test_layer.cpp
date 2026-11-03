#include "height_obstacle_layer/height_obstacle_layer.hpp"

#include <gtest/gtest.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <thread>
#include <vector>

namespace
{
constexpr float kThreshold = 0.30f;

float referenceDifference(grid_map::GridMap & map, const grid_map::Position & center, double radius)
{
  std::vector<float> known;
  for (int ix = -1; ix <= 1; ++ix) {
    for (int iy = -1; iy <= 1; ++iy) {
      const grid_map::Position p(center.x() + ix * radius, center.y() + iy * radius);
      if (!map.isInside(p)) {continue;}
      grid_map::Index index;
      if (!map.getIndex(p, index) || (index < grid_map::Index::Zero()).any() ||
        (index >= map.getSize()).any()) {continue;}
      const float z = map.atPosition("elevation", p);
      if (std::isfinite(z)) {known.push_back(z);}
    }
  }
  return known.empty() ? std::numeric_limits<float>::quiet_NaN() :
    *std::max_element(known.begin(), known.end()) - *std::min_element(known.begin(), known.end());
}

grid_map::GridMap makeMap(double resolution = 0.10)
{
  grid_map::GridMap map({"elevation"});
  map.setGeometry(grid_map::Length(2.6, 1.82), resolution, grid_map::Position(0.2, -0.1));
  map.setFrameId("map");
  map["elevation"].setZero();
  return map;
}

class LayerHarness
{
public:
  explicit LayerHarness(bool rolling = false, bool unknown_obstacle = false, double radius = 0.10)
  : radius_(radius), unknown_obstacle_(unknown_obstacle)
  {
    static int serial = 0;
    const auto id = std::to_string(++serial);
    const auto input_topic = "/offline_height_input_" + id;
    const auto difference_topic = "/offline_height_difference_" + id;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("height.map_topic", input_topic),
      rclcpp::Parameter("height.jump_radius", radius),
      rclcpp::Parameter("height.max_height_jump", static_cast<double>(kThreshold)),
      rclcpp::Parameter("height.unknown_as_obstacle", unknown_obstacle),
      rclcpp::Parameter("height.publish_height_difference", true),
      rclcpp::Parameter("height.height_difference_topic", difference_topic)});
    node_ = std::make_shared<nav2_util::LifecycleNode>("offline_height_" + id, "", options);
    tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    layered_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", rolling, false);
    layered_->resizeMap(80, 60, 0.05, -2.0, -1.5);
    layer_ = std::make_shared<height_obstacle_layer::HeightObstacleLayer>();
    layered_->addPlugin(layer_);
    layer_->initialize(layered_.get(), "height", tf_.get(), node_,
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    layer_->activate();
    publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(input_topic, rclcpp::SensorDataQoS());
    publisher_->on_activate();
    difference_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
      difference_topic, rclcpp::SensorDataQoS(),
      [this](grid_map_msgs::msg::GridMap::ConstSharedPtr msg) {last_difference_ = msg;});
    executor_.add_node(node_->get_node_base_interface());
    wait([&]() {return publisher_->get_subscription_count() > 0 && difference_->get_publisher_count() > 0;});
  }

  ~LayerHarness()
  {
    layer_->deactivate();
    executor_.remove_node(node_->get_node_base_interface());
  }

  void send(grid_map::GridMap & map)
  {
    const auto stamp = node_->now().nanoseconds();
    map.setTimestamp(stamp);
    publisher_->publish(*grid_map::GridMapRosConverter::toMessage(map));
    wait([&]() {return last_difference_ && rclcpp::Time(last_difference_->header.stamp).nanoseconds() == stamp;});
    update();
    ASSERT_TRUE(layer_->isCurrent());
  }

  void sendStale(grid_map::GridMap map)
  {
    map.setTimestamp(node_->now().nanoseconds() - 5000000000LL);
    publisher_->publish(*grid_map::GridMapRosConverter::toMessage(map));
    wait([&]() {update(); return !layer_->isCurrent();});
  }

  void update(double x = 0, double y = 0) {layered_->updateMap(x, y, 0);}
  void reset() {layer_->reset(); update();}

  void check(grid_map::GridMap & source)
  {
    const auto * actual = layered_->getCostmap();
    nav2_costmap_2d::Costmap2D expected(actual->getSizeInCellsX(), actual->getSizeInCellsY(),
      actual->getResolution(), actual->getOriginX(), actual->getOriginY(), nav2_costmap_2d::FREE_SPACE);
    grid_map::GridMap difference;
    ASSERT_NE(last_difference_, nullptr);
    ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(*last_difference_, difference));
    for (grid_map::GridMapIterator iterator(source); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position p;
      ASSERT_TRUE(source.getPosition(*iterator, p));
      const float jump = referenceDifference(source, p, radius_);
      const float published = difference.atPosition("height_difference", p);
      if (std::isnan(jump)) {ASSERT_TRUE(std::isnan(published));}
      else {ASSERT_EQ(jump, published) << "at " << p.transpose();}
      const bool obstacle = std::isfinite(jump) ? jump >= kThreshold : unknown_obstacle_;
      unsigned int mx, my;
      if (obstacle && expected.worldToMap(p.x(), p.y(), mx, my)) {
        expected.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
    for (unsigned int y = 0; y < actual->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < actual->getSizeInCellsX(); ++x) {
        ASSERT_EQ(expected.getCost(x, y), actual->getCost(x, y)) << "costmap cell " << x << ',' << y;
      }
    }
  }

private:
  void wait(const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      executor_.spin_some();
      if (done()) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("height layer callback/discovery timeout");
  }

  double radius_;
  bool unknown_obstacle_;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered_;
  std::shared_ptr<height_obstacle_layer::HeightObstacleLayer> layer_;
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr difference_;
  grid_map_msgs::msg::GridMap::ConstSharedPtr last_difference_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

class HeightLayerIntegration : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(HeightLayerIntegration, WritesAndRemovesObstaclesAndPublishesMatchingDifferences)
{
  LayerHarness harness;
  auto map = makeMap();
  harness.send(map); harness.check(map);
  map.atPosition("elevation", grid_map::Position(0.225, -0.075)) = 0.4f;
  harness.send(map); harness.check(map);
  harness.reset(); harness.check(map);
  map["elevation"].setZero();
  harness.send(map); harness.check(map);
}

TEST_F(HeightLayerIntegration, RollingGridAndNonIntegralRadiusPreserveWorldCells)
{
  LayerHarness harness(false, false, 0.17);
  auto map = makeMap(0.13);
  map.atPosition("elevation", grid_map::Position(0.225, -0.075)) = 0.4f;
  map.move(map.getPosition() + grid_map::Position(0.39, -0.26));
  harness.send(map); harness.check(map);
}

TEST_F(HeightLayerIntegration, RollingCostmapKeepsObstaclesAtWorldPositions)
{
  LayerHarness harness(true);
  auto map = makeMap();
  map.atPosition("elevation", grid_map::Position(0.225, -0.075)) = 0.4f;
  harness.send(map); harness.check(map);
  harness.update(0.4, -0.3); harness.check(map);
}

TEST_F(HeightLayerIntegration, UnknownPoliciesKeepExistingClassification)
{
  for (bool block : {false, true}) {
    LayerHarness harness(false, block);
    auto map = makeMap();
    map["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
    harness.send(map); harness.check(map);
    map.atPosition("elevation", grid_map::Position(0.225, -0.075)) = 0;
    harness.send(map); harness.check(map);
  }
}

TEST_F(HeightLayerIntegration, StaleInputRetainsLastObstacleSnapshot)
{
  LayerHarness harness;
  auto map = makeMap();
  map.atPosition("elevation", grid_map::Position(0.225, -0.075)) = 0.4f;
  harness.send(map); harness.check(map);
  auto empty = makeMap();
  harness.sendStale(empty); harness.check(map);
}

TEST_F(HeightLayerIntegration, MixedMapsMatchReferenceAcrossResolutionsAndUnknownPolicies)
{
  for (double resolution : {0.05, 0.10, 0.13}) {
    for (double radius : {0.10, 0.17}) {
      for (bool rolling : {false, true}) {
        for (bool unknown : {false, true}) {
          SCOPED_TRACE(testing::Message() << "resolution=" << resolution << " radius=" << radius
            << " rolling=" << rolling << " unknown=" << unknown);
          LayerHarness harness(false, unknown, radius);
          auto map = makeMap(resolution);
          std::mt19937 random(20261018);
          auto & matrix = map["elevation"];
          for (int i = 0; i < matrix.size(); ++i) {
            const auto key = random();
            matrix.data()[i] = key % 9 == 0 ? std::numeric_limits<float>::quiet_NaN() :
              (key % 57 == 0 ? 0.30f : (key % 19 == 0 ? 0.29f : 0));
          }
          if (rolling) {map.move(map.getPosition() + grid_map::Position(3 * resolution, -2 * resolution));}
          harness.send(map); harness.check(map);
        }
      }
    }
  }
}
}  // namespace
