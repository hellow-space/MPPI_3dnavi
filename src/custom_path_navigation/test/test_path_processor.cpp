#include <cmath>
#include <initializer_list>
#include <limits>
#include <utility>

#include "gtest/gtest.h"
#include "custom_path_navigation/path_processor.hpp"

namespace custom_path_navigation
{
namespace
{

nav_msgs::msg::Path makePath(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 42;
  path.header.stamp.nanosec = 123456789;
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp.sec = 1;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 7.0;
    path.poses.push_back(pose);
  }
  return path;
}

void expectRejected(const nav_msgs::msg::Path & path, const ProcessingConfig & config = {})
{
  const auto result = processPath(path, config);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error.empty());
  EXPECT_TRUE(result.path.poses.empty());
}

double yawOf(const geometry_msgs::msg::Quaternion & quaternion)
{
  return 2.0 * std::atan2(quaternion.z, quaternion.w);
}

TEST(PathProcessor, RejectsEmptySinglePointNonFiniteAndTooShortPaths)
{
  expectRejected(makePath({}));
  expectRejected(makePath({{0.0, 0.0}}));
  expectRejected(makePath({{0.0, 0.0}, {0.199, 0.0}}));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  expectRejected(makePath({{0.0, 0.0}, {nan, 0.3}}));
  expectRejected(makePath({{0.0, 0.0}, {0.3, infinity}}));
  auto nonfinite_z = makePath({{0.0, 0.0}, {0.3, 0.0}});
  nonfinite_z.poses.back().pose.position.z = infinity;
  expectRejected(nonfinite_z);
  EXPECT_TRUE(processPath(makePath({{0.0, 0.0}, {0.20, 0.0}}), {}).valid);
}

TEST(PathProcessor, RemovesPointsCloserThanTwoCentimeters)
{
  ProcessingConfig config;
  config.smoothing_passes = 0;
  const auto result = processPath(
    makePath({{0.0, 0.0}, {0.01, 0.01}, {0.03, 0.0}, {0.04, 0.01}, {0.30, 0.0}}),
    config);
  ASSERT_TRUE(result.valid) << result.error;
  for (const auto & pose : result.path.poses) {
    EXPECT_DOUBLE_EQ(pose.pose.position.y, 0.0);
  }

  // Spacing is measured from the last retained point, not the last raw point.
  config.sample_spacing = std::hypot(0.019, 0.019);
  const auto retained = processPath(
    makePath({{0.0, 0.0}, {0.019, 0.0}, {0.019, 0.019}, {0.30, 0.0}}), config);
  ASSERT_TRUE(retained.valid) << retained.error;
  ASSERT_GE(retained.path.poses.size(), 3U);
  EXPECT_NEAR(retained.path.poses[1].pose.position.x, 0.019, 1e-12);
  EXPECT_NEAR(retained.path.poses[1].pose.position.y, 0.019, 1e-12);
}

TEST(PathProcessor, MovingAveragePreservesFirstAndLastPosition)
{
  const auto raw = makePath({{0.0, 0.0}, {0.30, 0.30}, {0.60, 0.0}});
  for (const int passes : {0, 1, 2}) {
    ProcessingConfig config;
    config.smoothing_passes = passes;
    const double expected_y = passes == 0 ? 0.30 : (passes == 1 ? 0.10 : 1.0 / 30.0);
    config.sample_spacing = std::hypot(0.30, expected_y);
    const auto result = processPath(raw, config);
    ASSERT_TRUE(result.valid) << result.error;
    ASSERT_EQ(result.path.poses.size(), 3U);
    EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x, 0.60);
    EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.y, 0.0);
    EXPECT_NEAR(result.path.poses[1].pose.position.x, 0.30, 1e-12);
    EXPECT_NEAR(result.path.poses[1].pose.position.y, expected_y, 1e-12);
  }
}

TEST(PathProcessor, ResamplesStraightLineAtFiveCentimeterArcSpacing)
{
  const ProcessingConfig config;
  EXPECT_DOUBLE_EQ(config.min_raw_spacing, 0.02);
  EXPECT_DOUBLE_EQ(config.min_path_length, 0.20);
  EXPECT_DOUBLE_EQ(config.sample_spacing, 0.05);
  EXPECT_EQ(config.smoothing_passes, 1);

  const auto result = processPath(makePath({{0.0, 0.0}, {0.23, 0.0}}), config);
  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_EQ(result.path.poses.size(), 6U);
  for (std::size_t index = 0; index < 5U; ++index) {
    EXPECT_NEAR(result.path.poses[index].pose.position.x, 0.05 * index, 1e-12);
    EXPECT_DOUBLE_EQ(result.path.poses[index].pose.position.y, 0.0);
  }
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x, 0.23);
  EXPECT_NEAR(
    result.path.poses.back().pose.position.x - result.path.poses[4].pose.position.x,
    0.03, 1e-12);

  const auto exact_multiple = processPath(makePath({{0.0, 0.0}, {0.25, 0.0}}), config);
  ASSERT_TRUE(exact_multiple.valid) << exact_multiple.error;
  ASSERT_EQ(exact_multiple.path.poses.size(), 6U);
  EXPECT_DOUBLE_EQ(exact_multiple.path.poses.back().pose.position.x, 0.25);
}

TEST(PathProcessor, AssignsFiniteTangentYawAndZeroZToEveryPose)
{
  ProcessingConfig config;
  config.smoothing_passes = 0;
  const auto raw = makePath({{0.0, 0.0}, {0.10, 0.0}, {0.10, 0.15}});
  const auto result = processPath(raw, config);
  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_EQ(result.path.poses.size(), 6U);
  EXPECT_EQ(result.path.header, raw.header);
  for (const auto & pose : result.path.poses) {
    const auto & q = pose.pose.orientation;
    EXPECT_EQ(pose.header, raw.header);
    EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.0);
    EXPECT_TRUE(std::isfinite(q.x));
    EXPECT_TRUE(std::isfinite(q.y));
    EXPECT_TRUE(std::isfinite(q.z));
    EXPECT_TRUE(std::isfinite(q.w));
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12);
  }
  const double pi = std::acos(-1.0);
  EXPECT_NEAR(yawOf(result.path.poses.front().pose.orientation), 0.0, 1e-12);
  EXPECT_NEAR(yawOf(result.path.poses[2].pose.orientation), pi / 4.0, 1e-12);
  EXPECT_NEAR(yawOf(result.path.poses[3].pose.orientation), pi / 2.0, 1e-12);
}

TEST(PathProcessor, UsesFinalSegmentYawForTheTerminalPose)
{
  ProcessingConfig config;
  config.smoothing_passes = 0;
  const auto raw = makePath({{0.0, 0.0}, {0.10, 0.0}, {0.10, -0.15}});
  const auto preview = processPath(raw, config);
  ASSERT_TRUE(preview.valid) << preview.error;
  const auto & terminal = preview.path.poses.back();
  EXPECT_DOUBLE_EQ(terminal.pose.position.x, 0.10);
  EXPECT_DOUBLE_EQ(terminal.pose.position.y, -0.15);
  EXPECT_NEAR(yawOf(terminal.pose.orientation), -std::acos(-1.0) / 2.0, 1e-12);
  const auto repeated = processPath(raw, config);
  ASSERT_TRUE(repeated.valid) << repeated.error;
  EXPECT_EQ(repeated.path, preview.path);
  EXPECT_EQ(repeated.path.poses.back(), terminal);
}

TEST(PathProcessor, DegenerateDuplicateStrokeNeverProducesNanYaw)
{
  expectRejected(makePath({{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}));
  const auto result = processPath(
    makePath({{0.0, 0.0}, {0.0, 0.0}, {0.30, 0.0}, {0.30, 0.0}}), {});
  ASSERT_TRUE(result.valid) << result.error;
  for (const auto & pose : result.path.poses) {
    EXPECT_TRUE(std::isfinite(yawOf(pose.pose.orientation)));
    EXPECT_NEAR(yawOf(pose.pose.orientation), 0.0, 1e-12);
  }
}

TEST(PathProcessor, RejectsInvalidConfiguration)
{
  const auto raw = makePath({{0.0, 0.0}, {0.30, 0.0}});
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto field : {
        &ProcessingConfig::min_raw_spacing, &ProcessingConfig::min_path_length,
        &ProcessingConfig::sample_spacing})
  {
    for (const double value : {0.0, -0.01, infinity, nan}) {
      ProcessingConfig config;
      config.*field = value;
      expectRejected(raw, config);
    }
  }
  ProcessingConfig config;
  config.smoothing_passes = -1;
  expectRejected(raw, config);
}

TEST(PathProcessor, RejectsEmptyAndInconsistentFrames)
{
  auto raw = makePath({{0.0, 0.0}, {0.30, 0.0}});
  raw.header.frame_id.clear();
  expectRejected(raw);
  raw.header.frame_id = "map";
  raw.poses.back().header.frame_id = "odom";
  expectRejected(raw);
  raw.poses.front().header.frame_id.clear();
  raw.poses.back().header.frame_id.clear();
  const auto result = processPath(raw, {});
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(result.path.header.frame_id, "map");
}

TEST(PathProcessor, PreservesRawTerminalPositionWhenItIsCloseToThePreviousPoint)
{
  const auto raw = makePath({{0.0, 0.0}, {0.30, 0.0}, {0.305, 0.003}});
  const auto result = processPath(raw, {});
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x, 0.305);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.y, 0.003);
}

TEST(PathProcessor, MeasuresResamplingDistanceAlongThePolyline)
{
  ProcessingConfig config;
  config.smoothing_passes = 0;
  const auto result = processPath(
    makePath({{0.0, 0.0}, {0.075, 0.0}, {0.075, 0.175}}), config);
  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_EQ(result.path.poses.size(), 6U);
  EXPECT_NEAR(result.path.poses[1].pose.position.x, 0.05, 1e-12);
  EXPECT_NEAR(result.path.poses[1].pose.position.y, 0.0, 1e-12);
  EXPECT_NEAR(result.path.poses[2].pose.position.x, 0.075, 1e-12);
  EXPECT_NEAR(result.path.poses[2].pose.position.y, 0.025, 1e-12);
  EXPECT_NEAR(result.path.poses[3].pose.position.y, 0.075, 1e-12);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x, 0.075);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.y, 0.175);
}

TEST(PathProcessor, FallsBackToNonzeroTangentAtAnExactTurnaround)
{
  ProcessingConfig config;
  config.smoothing_passes = 0;
  const auto result = processPath(makePath({{0.0, 0.0}, {0.10, 0.0}, {0.0, 0.0}}), config);
  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_EQ(result.path.poses.size(), 5U);
  const auto & corner = result.path.poses[2].pose.orientation;
  EXPECT_NEAR(std::abs(yawOf(corner)), std::acos(-1.0), 1e-12);
  EXPECT_NEAR(corner.z * corner.z + corner.w * corner.w, 1.0, 1e-12);
}

TEST(PathProcessor, AcceptsEndpointCoincidingWithTheLastArcSample)
{
  ProcessingConfig config;
  config.smoothing_passes = 0;
  const auto raw = makePath({{0.0, 0.0}, {0.20, 0.0}, {0.225, 0.0}, {0.20, 0.0}});
  const auto result = processPath(raw, config);
  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_EQ(result.path.poses.size(), 5U);
  EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.x, 0.20);
  EXPECT_DOUBLE_EQ(result.path.poses.back().pose.position.y, 0.0);
  for (std::size_t index = 1U; index < result.path.poses.size(); ++index) {
    EXPECT_GT(
      result.path.poses[index].pose.position.x -
      result.path.poses[index - 1U].pose.position.x, 0.0);
  }
  const auto & terminal_orientation = result.path.poses.back().pose.orientation;
  EXPECT_TRUE(std::isfinite(yawOf(terminal_orientation)));
  EXPECT_NEAR(yawOf(terminal_orientation), 0.0, 1e-12);
  EXPECT_NEAR(
    terminal_orientation.z * terminal_orientation.z +
    terminal_orientation.w * terminal_orientation.w, 1.0, 1e-12);
}

}  // namespace
}  // namespace custom_path_navigation
