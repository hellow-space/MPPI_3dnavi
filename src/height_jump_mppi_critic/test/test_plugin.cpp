#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav2_mppi_controller/critic_manager.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_ros/buffer.h>

class PluginTest : public testing::Test
{
protected:
  virtual bool startupEnabled() const {return true;}
  virtual bool unknownIsTraversable() const {return false;}
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("FollowPath.critics", std::vector<std::string>{"HeightJumpCritic"}),
      rclcpp::Parameter("FollowPath.HeightJumpCritic.map_topic", "/height_jump_test_map"),
      rclcpp::Parameter("FollowPath.HeightJumpCritic.max_height_jump", 0.3),
      rclcpp::Parameter(
        "FollowPath.HeightJumpCritic.unknown_is_traversable", unknownIsTraversable()),
      rclcpp::Parameter("FollowPath.HeightJumpCritic.startup_unknown_enabled", startupEnabled()),
      rclcpp::Parameter("FollowPath.HeightJumpCritic.fixed_frame", "odom"),
      rclcpp::Parameter("FollowPath.HeightJumpCritic.unknown_ignore_radius", 0.7),
      rclcpp::Parameter("FollowPath.HeightJumpCritic.map_timeout", 1.0)});
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("height_jump_test", options);
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap", "", "test_costmap");
    costmap->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->on_configure(rclcpp_lifecycle::State{});
    params = std::make_unique<mppi::ParametersHandler>(node);
    manager = std::make_unique<mppi::CriticManager>();
    manager->on_configure(node, "FollowPath", costmap, params.get());
    params->start();
    publisher = node->create_publisher<grid_map_msgs::msg::GridMap>("/height_jump_test_map", 1);
    publisher->on_activate();
    executor.add_node(node->get_node_base_interface());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u);
    state.pose.header.frame_id = "odom";
    state.pose.pose.orientation.w = 1;
    trajectories.reset(2, 3);
    costs = xt::zeros<float>({2});
  }
  void TearDown() override
  {
    executor.remove_node(node->get_node_base_interface());
    manager.reset(); params.reset();
    costmap->on_cleanup(rclcpp_lifecycle::State{});
    costmap.reset(); publisher.reset(); node.reset();
  }
  grid_map_msgs::msg::GridMap makeMap(float z = 0)
  {
    grid_map::GridMap map({"elevation"});
    map.setGeometry(grid_map::Length(8, 8), 0.05);
    map.setFrameId("odom");
    map.setTimestamp(node->now().nanoseconds());
    map["elevation"].setConstant(z);
    return *grid_map::GridMapRosConverter::toMessage(map);
  }
  void send(const grid_map_msgs::msg::GridMap & msg)
  {
    publisher->publish(msg);
    pump();
  }
  void pump()
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  bool score(bool prior_failure = false)
  {
    costs.fill(0);
    state.pose.header.stamp = node->now();
    mppi::CriticData data{state, trajectories, path, costs, dt, prior_failure,
      nullptr, nullptr, std::nullopt, std::nullopt};
    manager->evalTrajectoriesScores(data);
    return data.fail_flag;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::unique_ptr<mppi::ParametersHandler> params;
  std::unique_ptr<mppi::CriticManager> manager;
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  mppi::models::Path path;
  xt::xtensor<float, 1> costs;
  float dt{0.05f};
};

TEST_F(PluginTest, Nav2LoadsPluginAndFlatTerrainLeavesCostsUnchanged)
{
  EXPECT_TRUE(score());  // no map must not be excused by the startup region
  send(makeMap());
  EXPECT_FALSE(score());
  EXPECT_EQ(costs(0), 0); EXPECT_EQ(costs(1), 0);
  EXPECT_TRUE(score(true));  // never clear another critic's failure
}

TEST_F(PluginTest, BinaryRejectAndAllFailed)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(makeMap(), map);
  map.atPosition("elevation", grid_map::Position(1, 0)) = 0.6f;
  send(*grid_map::GridMapRosConverter::toMessage(map));
  trajectories.x(1, 1) = 1;
  EXPECT_FALSE(score());
  EXPECT_EQ(costs(0), 0); EXPECT_EQ(costs(1), 1000000);
  trajectories.x(0, 1) = 1;
  EXPECT_TRUE(score());
  EXPECT_EQ(costs(0), 1000000); EXPECT_EQ(costs(1), 1000000);
}

TEST_F(PluginTest, UnknownRegionStaysFixedAcrossRobotMotionAndParameterUpdates)
{
  send(makeMap(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(score());
  state.pose.pose.position.x = 2;
  trajectories.x.fill(2);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("FollowPath.HeightJumpCritic.enabled", false)).successful);
  EXPECT_FALSE(score());
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("FollowPath.HeightJumpCritic.enabled", true)).successful);
  EXPECT_TRUE(score());
  trajectories.x.fill(0);
  EXPECT_FALSE(score());  // old region still works, current pose does not create a new region
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("FollowPath.HeightJumpCritic.jump_radius", 0.2)).successful);
}

TEST_F(PluginTest, StaleFutureMissingLayerAndOutOfMapDoNotGetStartupExemption)
{
  send(makeMap()); EXPECT_FALSE(score());
  auto stale = makeMap(); stale.header.stamp.sec -= 5;
  send(stale); EXPECT_TRUE(score());
  auto future = makeMap(); future.header.stamp.sec += 5;
  send(future); EXPECT_TRUE(score());
  auto missing = makeMap(); missing.layers[0] = "inpaint";
  send(missing); EXPECT_TRUE(score());
  send(makeMap());
  trajectories.x.fill(9);
  EXPECT_TRUE(score());
}

TEST_F(PluginTest, TransformsMapAndStartupRegionAndFailsWhenTfIsMissing)
{
  auto msg = makeMap(std::numeric_limits<float>::quiet_NaN());
  msg.header.frame_id = "terrain";
  msg.info.pose.position.x = 10;
  send(msg);
  EXPECT_TRUE(score());
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "terrain"; tf.child_frame_id = "odom";
  tf.header.stamp = node->now();
  tf.transform.translation.x = 10; tf.transform.rotation.w = 1;
  ASSERT_TRUE(costmap->getTfBuffer()->setTransform(tf, "test", true));
  EXPECT_FALSE(score());
  trajectories.x.fill(2);
  EXPECT_TRUE(score());
  trajectories.x.fill(0);
  tf.transform.rotation.x = 0.1;
  tf.transform.rotation.w = std::sqrt(0.99);
  ASSERT_TRUE(costmap->getTfBuffer()->setTransform(tf, "test", true));
  EXPECT_TRUE(score());  // cannot flatten tilted coordinate frames silently
}

TEST_F(PluginTest, RotatedMapQueriesTheCorrectPhysicalObstacle)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(makeMap(), map);
  map.setFrameId("rotated");
  // Query cell interiors: a rotated point exactly on a cell edge can round into its neighbour.
  map.atPosition("elevation", grid_map::Position(0.025, 1.025)) = 0.6f;
  send(*grid_map::GridMapRosConverter::toMessage(map));
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "rotated"; tf.child_frame_id = "odom";
  tf.header.stamp = node->now();
  tf.transform.rotation.z = std::sqrt(0.5);
  tf.transform.rotation.w = std::sqrt(0.5);
  ASSERT_TRUE(costmap->getTfBuffer()->setTransform(tf, "test", true));
  trajectories.x(1, 1) = 1.025f;
  trajectories.y(1, 1) = -0.025f;  // odom (1.025,-.025) becomes map (.025,1.025)
  EXPECT_FALSE(score());
  EXPECT_EQ(costs(0), 0); EXPECT_EQ(costs(1), 1000000);
}

TEST_F(PluginTest, ClockRollbackRequiresExplicitReinitialization)
{
  send(makeMap()); EXPECT_FALSE(score());
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  pump();  // rclcpp TimeSource applies the parameter event through its subscription
  ASSERT_EQ(node->now().nanoseconds(), 0);
  EXPECT_TRUE(score());  // time drops from wall time to zero before first /clock
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("use_sim_time", false)).successful);
  pump();
  ASSERT_GT(node->now().nanoseconds(), 0);
  send(makeMap());
  EXPECT_TRUE(score());  // rollback remains latched, cannot silently reuse the old startup origin
}

TEST_F(PluginTest, NewConfiguredInstanceRecapturesOrigin)
{
  send(makeMap(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(score());
  state.pose.pose.position.x = 2; trajectories.x.fill(2);
  EXPECT_TRUE(score());
  manager.reset(); params.reset();
  params = std::make_unique<mppi::ParametersHandler>(node);
  manager = std::make_unique<mppi::CriticManager>();
  manager->on_configure(node, "FollowPath", costmap, params.get());
  params->start();
  EXPECT_TRUE(score());  // recreated source still requires fresh input
  send(makeMap(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(score());
  trajectories.x.fill(0);
  EXPECT_TRUE(score());
}

class WithoutStartupTest : public PluginTest
{
  bool startupEnabled() const override {return false;}
};

TEST_F(WithoutStartupTest, UnknownRejectsWithoutExplicitSafeStartupOptIn)
{
  send(makeMap(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_TRUE(score());
  send(makeMap());
  EXPECT_FALSE(score());
}

class TraversableUnknownTest : public PluginTest
{
  bool startupEnabled() const override {return false;}
  bool unknownIsTraversable() const override {return true;}
};

TEST_F(TraversableUnknownTest, UnknownPassesButKnownHeightOutsideAndStaleMapStillReject)
{
  send(makeMap(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(score());

  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(makeMap(), map);
  map.atPosition("elevation", grid_map::Position(1, 0)) = 0.6f;
  send(*grid_map::GridMapRosConverter::toMessage(map));
  trajectories.x.fill(1);
  EXPECT_TRUE(score());

  send(makeMap());
  trajectories.x.fill(9);
  EXPECT_TRUE(score());

  auto stale = makeMap();
  stale.header.stamp.sec -= 5;
  send(stale);
  trajectories.x.fill(0);
  EXPECT_TRUE(score());
}

class StartupRadiusConfigurationTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(StartupRadiusConfigurationTest, EnabledExemptionRequiresExplicitPositiveRadius)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.critics", std::vector<std::string>{"HeightJumpCritic"}),
    rclcpp::Parameter("FollowPath.HeightJumpCritic.map_topic", "/height_jump_test_map"),
    rclcpp::Parameter("FollowPath.HeightJumpCritic.max_height_jump", 0.3),
    rclcpp::Parameter("FollowPath.HeightJumpCritic.startup_unknown_enabled", true),
    rclcpp::Parameter("FollowPath.HeightJumpCritic.fixed_frame", "odom")});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "height_jump_missing_radius_test", options);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "missing_radius_costmap", "", "missing_radius_costmap");
  costmap->set_parameter(rclcpp::Parameter("global_frame", "odom"));
  costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  costmap->on_configure(rclcpp_lifecycle::State{});
  auto params = std::make_unique<mppi::ParametersHandler>(node);
  mppi::CriticManager manager;

  EXPECT_THROW(
    manager.on_configure(node, "FollowPath", costmap, params.get()), std::invalid_argument);

  costmap->on_cleanup(rclcpp_lifecycle::State{});
}
