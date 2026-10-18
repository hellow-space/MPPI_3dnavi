#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "height_obstacle_layer/height_difference.hpp"

TEST(HeightDifference, MarksExactThresholdAsObstacle)
{
  const std::array<float, 3> samples{0.0F, 0.30F, 0.10F};
  const auto result = height_obstacle_layer::classifyNeighborhood(samples, 0.30F);
  EXPECT_TRUE(result.has_known_height);
  EXPECT_TRUE(result.is_obstacle);
}

TEST(HeightDifference, IgnoresUnknownSamples)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::array<float, 3> samples{nan, nan, nan};
  const auto result = height_obstacle_layer::classifyNeighborhood(samples, 0.30F);
  EXPECT_FALSE(result.has_known_height);
  EXPECT_FALSE(result.is_obstacle);
  EXPECT_TRUE(std::isnan(result.height_difference));
}
