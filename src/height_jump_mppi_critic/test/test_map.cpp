#include <gtest/gtest.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <limits>
#include "height_jump_mppi_critic/terrain_map.hpp"

using namespace height_jump;

static grid_map_msgs::msg::GridMap message()
{
  grid_map::GridMap map({"elevation"});
  map.setGeometry(grid_map::Length(2, 2), 0.1, grid_map::Position(4, -2));
  map.setFrameId("terrain");
  map.setTimestamp(10000000000ULL);
  map["elevation"].setConstant(0.2f);
  return *grid_map::GridMapRosConverter::toMessage(map);
}

TEST(TerrainMap, ReadsWorldCoordinatesAndPreservesUnknown)
{
  auto msg = message();
  std::string error;
  auto map = TerrainSnapshot::fromMessage(msg, "elevation", error);
  ASSERT_NE(map, nullptr) << error;
  EXPECT_EQ(map->query(4, -2).state, SampleState::Known);
  EXPECT_FLOAT_EQ(map->query(4, -2).elevation, 0.2f);
  EXPECT_EQ(map->query(0, 0).state, SampleState::Outside);
  for (float value : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()})
  {
    std::fill(msg.data[0].data.begin(), msg.data[0].data.end(), value);
    auto unknown = TerrainSnapshot::fromMessage(msg, "elevation", error);
    ASSERT_NE(unknown, nullptr);
    EXPECT_EQ(unknown->query(4, -2).state, SampleState::Unknown);
    EXPECT_EQ(map->query(4, -2).state, SampleState::Known);  // old snapshot is immutable
  }
}

TEST(TerrainMap, RejectsMalformedOrMissingLayersAndNonHorizontalGeometry)
{
  std::string error;
  EXPECT_EQ(TerrainSnapshot::fromMessage(message(), "absent", error), nullptr);
  for (int kind = 0; kind < 8; ++kind) {
    auto msg = message();
    switch (kind) {
      case 0: msg.data[0].data.pop_back(); break;
      case 1: msg.info.resolution = 0; break;
      case 2: msg.info.pose.orientation.z = 0.3; break;
      case 3: msg.header.frame_id = ""; break;
      case 4: msg.outer_start_index = 9999; break;
      case 5: msg.data[0].layout.dim.clear(); break;
      case 6: msg.info.length_x = std::numeric_limits<double>::infinity(); break;
      case 7: msg.data[0].layout.data_offset = 1; break;
    }
    EXPECT_EQ(TerrainSnapshot::fromMessage(msg, "elevation", error), nullptr) << kind;
  }
}

TEST(TerrainMap, ChecksObservationTimestampAsWellAsReceiptAge)
{
  std::string error;
  auto map = TerrainSnapshot::fromMessage(message(), "elevation", error);
  ASSERT_NE(map, nullptr);
  EXPECT_TRUE(map->fresh(10500000000LL, 1.0));
  EXPECT_FALSE(map->fresh(12000000000LL, 1.0));
  EXPECT_FALSE(map->fresh(9000000000LL, 1.0));
  EXPECT_FALSE(map->fresh(10500000000LL, 0.0));
}

TEST(TerrainMap, RollingBufferQueriesStayAtTheSameWorldPosition)
{
  grid_map::GridMap original({"elevation"});
  original.setGeometry(grid_map::Length(2, 2), 0.1);
  original.setFrameId("terrain");
  original.setTimestamp(10000000000ULL);
  original["elevation"].setConstant(0.0f);
  original.atPosition("elevation", grid_map::Position(0.25, -0.25)) = 0.6f;
  original.move(grid_map::Position(0.3, 0.2));
  auto msg = grid_map::GridMapRosConverter::toMessage(original);
  ASSERT_TRUE(msg->outer_start_index != 0 || msg->inner_start_index != 0);
  std::string error;
  auto map = TerrainSnapshot::fromMessage(*msg, "elevation", error);
  ASSERT_NE(map, nullptr) << error;
  EXPECT_FLOAT_EQ(map->query(0.25, -0.25).elevation, 0.6f);
  EXPECT_FLOAT_EQ(map->query(-0.25, 0.25).elevation, 0.0f);
}

TEST(TerrainMap, NonSquareColumnMajorLayoutDoesNotTransposeAxes)
{
  grid_map::GridMap original({"elevation"});
  original.setGeometry(grid_map::Length(3, 2), 0.1, grid_map::Position(4, -2));
  original.setFrameId("terrain");
  original.setTimestamp(10000000000ULL);
  original["elevation"].setConstant(0.0f);
  original.atPosition("elevation", grid_map::Position(4.85, -2.35)) = 0.4f;
  auto msg = grid_map::GridMapRosConverter::toMessage(original);
  std::string error;
  auto map = TerrainSnapshot::fromMessage(*msg, "elevation", error);
  ASSERT_NE(map, nullptr) << error;
  EXPECT_FLOAT_EQ(map->query(4.85, -2.35).elevation, 0.4f);
  EXPECT_FLOAT_EQ(map->query(3.65, -1.15).elevation, 0.0f);
}
