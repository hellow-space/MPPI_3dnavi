#include <gtest/gtest.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <limits>
#include <cfenv>
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

TEST(TerrainMap, RejectsRoundedIndicesBeyondStorageAtMapEdges)
{
  grid_map::GridMap original({"elevation"});
  original.setGeometry(grid_map::Length(7, 5), 0.13, grid_map::Position(0.4, -0.3));
  original.setFrameId("terrain");
  original.setTimestamp(10000000000ULL);
  original["elevation"].setConstant(0.2f);
  std::string error;
  auto map = TerrainSnapshot::fromMessage(
    *grid_map::GridMapRosConverter::toMessage(original), "elevation", error);
  ASSERT_NE(map, nullptr) << error;
  const auto center = map->map.getPosition();
  const auto length = map->map.getLength();
  const auto size = map->map.getSize();
  size_t rounded_outside = 0;
  // Probe both axes on and immediately around all four map edges.
  for (int axis = 0; axis < 2; ++axis) {
    for (int end : {0, size[axis]}) {
      const double edge = center[axis] + length[axis] / 2 - end * 0.13;
      for (double coordinate : {std::nextafter(edge, -INFINITY), edge,
          std::nextafter(edge, INFINITY)})
      {
        grid_map::Position point = center;
        point[axis] = coordinate;
        grid_map::Index index;
        const bool inside = map->map.getIndex(point, index);
        const bool valid = inside && (index >= grid_map::Index::Zero()).all() &&
          (index < size).all();
        const auto sample = map->query(point.x(), point.y());
        if (!valid) {
          EXPECT_EQ(sample.state, SampleState::Outside) << "point " << point.transpose();
          rounded_outside += inside;
        } else {
          EXPECT_EQ(sample.state, SampleState::Known);
          EXPECT_FLOAT_EQ(sample.elevation, 0.2f);
        }
      }
    }
  }
  RecordProperty("rounded_outside_indices", static_cast<int>(rounded_outside));
  // A genuine last-cell center remains known; do not fix the bug by excluding an edge strip.
  const grid_map::Position last_center = center - length.matrix() / 2 + grid_map::Position(0.065, 0.065);
  EXPECT_EQ(map->query(last_center.x(), last_center.y()).state, SampleState::Known);
  EXPECT_FLOAT_EQ(map->query(last_center.x(), last_center.y()).elevation, 0.2f);
}

TEST(TerrainMap, PreparedQueryRetainsOriginalSnapshotAndUnknownState)
{
  std::string error;
  auto original = TerrainSnapshot::fromMessage(message(), "elevation", error);
  ASSERT_NE(original, nullptr);
  TerrainQuery query(original);
  original.reset();  // Prepared queries must own the snapshot they read.
  auto replacement_message = message();
  std::fill(replacement_message.data[0].data.begin(), replacement_message.data[0].data.end(),
    std::numeric_limits<float>::quiet_NaN());
  auto replacement = TerrainSnapshot::fromMessage(replacement_message, "elevation", error);
  ASSERT_NE(replacement, nullptr);
  TerrainQuery unknown(replacement);
  EXPECT_EQ(query(4, -2).state, SampleState::Known);
  EXPECT_FLOAT_EQ(query(4, -2).elevation, 0.2f);
  EXPECT_EQ(unknown(4, -2).state, SampleState::Unknown);
  EXPECT_EQ(query(100, -2).state, SampleState::Outside);
  EXPECT_EQ(query(std::numeric_limits<double>::quiet_NaN(), -2).state, SampleState::Outside);
  EXPECT_EQ(query(4, std::numeric_limits<double>::infinity()).state, SampleState::Outside);
  EXPECT_THROW(TerrainQuery(nullptr), std::invalid_argument);
}

TEST(TerrainMap, PreparedQueryRejectsHugeFiniteCoordinatesBeforeIntegerConversion)
{
  std::string error;
  auto map = TerrainSnapshot::fromMessage(message(), "elevation", error);
  ASSERT_NE(map, nullptr);
  TerrainQuery query(map);
  for (double value : {1e20, -1e20, std::numeric_limits<double>::max(),
      -std::numeric_limits<double>::max()})
  {
    std::feclearexcept(FE_ALL_EXCEPT);
    const auto sample = query(value, -2);
    EXPECT_EQ(std::fetestexcept(FE_INVALID), 0) << "x=" << value;
    EXPECT_EQ(sample.state, SampleState::Outside);
    std::feclearexcept(FE_ALL_EXCEPT);
    const auto other = query(4, value);
    EXPECT_EQ(std::fetestexcept(FE_INVALID), 0) << "y=" << value;
    EXPECT_EQ(other.state, SampleState::Outside);
  }
  std::feclearexcept(FE_ALL_EXCEPT);
}

TEST(TerrainMap, BatchedNeighborhoodPreservesContinuousSamplingAndCircularIndices)
{
  // Distinct heights make every wrong axis, wrap, or off-by-one observable.
  for (double resolution : {0.05, 0.10, 0.13}) {
    grid_map::GridMap original({"elevation"});
    original.setGeometry(grid_map::Length(7, 5), resolution, grid_map::Position(0.4, -0.3));
    original.setFrameId("terrain");
    original.setTimestamp(10000000000ULL);
    auto & matrix = original["elevation"];
    for (int i = 0; i < matrix.rows(); ++i) {
      for (int j = 0; j < matrix.cols(); ++j) {
        const int n = i * matrix.cols() + j;
        matrix(i, j) = n % 11 == 0 ? std::numeric_limits<float>::quiet_NaN() :
          n % 17 == 0 ? std::numeric_limits<float>::infinity() : n * 0.001f;
      }
    }
    for (const grid_map::Position & origin : {grid_map::Position(0.4, -0.3),
        grid_map::Position(-4.2, -17.8), grid_map::Position(1e9, -1e9)})
    {
      original.setPosition(origin);
      for (const grid_map::Index & start : {grid_map::Index(0, 0), grid_map::Index(0, 7),
          grid_map::Index(3, 0), grid_map::Index(3, 7)})
      {
        original.setStartIndex(start);
        std::string error;
        const auto map = TerrainSnapshot::fromMessage(
          *grid_map::GridMapRosConverter::toMessage(original), "elevation", error);
        ASSERT_NE(map, nullptr) << error;
        const TerrainQuery query(map);
        const auto center = map->map.getPosition();
        const auto length = map->map.getLength();
        for (double radius : {0.10, 0.17}) {
          const auto check = [&](const grid_map::Position & point) {
              const auto samples = query.neighborhood(point.x(), point.y(), radius);
              size_t n = 0;
              for (int x = -1; x <= 1; ++x) {
                for (int y = -1; y <= 1; ++y, ++n) {
                  const auto expected = map->query(point.x() + x * radius, point.y() + y * radius);
                  ASSERT_EQ(samples[n].state, expected.state)
                    << "resolution=" << resolution << " radius=" << radius << " start="
                    << start.transpose() << " point=" << point.transpose() << " sample=" << n;
                  if (expected.state == SampleState::Known) {
                    ASSERT_FLOAT_EQ(samples[n].elevation, expected.elevation);
                  }
                }
              }
            };
          // Probe internal grid lines as well as all map edges, including adjacent doubles.
          for (int axis = 0; axis < 2; ++axis) {
            for (int cell = 0; cell <= map->map.getSize()[axis]; ++cell) {
              const double edge = center[axis] + length[axis] / 2 - cell * resolution;
              for (double value : {std::nextafter(edge, -INFINITY), edge,
                  std::nextafter(edge, INFINITY), edge - resolution * 0.37})
              {
                auto point = center;
                point[axis] = value;
                check(point);
              }
            }
          }
          for (int n = 0; n < 300; ++n) {
            check(center + grid_map::Position(
              ((n * 17) % 809 - 404) * 0.01, ((n * 31) % 613 - 306) * 0.01));
          }
        }
      }
    }
  }
}

TEST(TerrainMap, BatchedNeighborhoodRejectsUnsafeCoordinatesBeforeIntegerConversion)
{
  std::string error;
  auto map = TerrainSnapshot::fromMessage(message(), "elevation", error);
  ASSERT_NE(map, nullptr);
  const TerrainQuery query(map);
  for (double value : {1e20, -1e20, std::numeric_limits<double>::max(),
      -std::numeric_limits<double>::max(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    for (const auto & point : {grid_map::Position(value, -2), grid_map::Position(4, value)}) {
      std::feclearexcept(FE_ALL_EXCEPT);
      const auto samples = query.neighborhood(point.x(), point.y(), 0.1);
      EXPECT_EQ(std::fetestexcept(FE_INVALID), 0) << point.transpose();
      for (const auto & sample : samples) {EXPECT_EQ(sample.state, SampleState::Outside);}
    }
  }
  for (double radius : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    for (const auto & sample : query.neighborhood(4, -2, radius)) {
      EXPECT_EQ(sample.state, SampleState::Outside);
    }
  }
  std::feclearexcept(FE_ALL_EXCEPT);
}

TEST(TerrainMap, BatchedNeighborhoodRetainsItsOriginalSnapshot)
{
  std::string error;
  auto msg = message();
  auto original = TerrainSnapshot::fromMessage(msg, "elevation", error);
  const TerrainQuery query(original);
  original.reset();
  std::fill(msg.data[0].data.begin(), msg.data[0].data.end(),
    std::numeric_limits<float>::quiet_NaN());
  const TerrainQuery replacement(TerrainSnapshot::fromMessage(msg, "elevation", error));
  for (const auto & sample : query.neighborhood(4, -2, 0.1)) {
    EXPECT_EQ(sample.state, SampleState::Known);
    EXPECT_FLOAT_EQ(sample.elevation, 0.2f);
  }
  for (const auto & sample : replacement.neighborhood(4, -2, 0.1)) {
    EXPECT_EQ(sample.state, SampleState::Unknown);
  }
}
