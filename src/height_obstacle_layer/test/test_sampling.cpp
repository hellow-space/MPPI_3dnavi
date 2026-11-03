#include "height_obstacle_layer/neighborhood_sampler.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

TEST(NeighborhoodSampler, PreservesNinePointOrderAcrossRollingIndices)
{
  grid_map::GridMap map({"elevation"});
  map.setGeometry(grid_map::Length(1, 1), 0.1);
  map["elevation"].setZero();
  const grid_map::Position center(0.025, 0.025);
  int value = 0;
  for (int ix = -1; ix <= 1; ++ix) {
    for (int iy = -1; iy <= 1; ++iy) {
      map.atPosition("elevation", center + grid_map::Position(ix * 0.1, iy * 0.1)) = value++;
    }
  }
  for (bool rolling : {false, true}) {
    if (rolling) {map.move(grid_map::Position(0.2, 0.1));}
    const height_obstacle_layer::NeighborhoodSampler sampler(map, "elevation", 0.1);
    const auto samples = sampler.sample(center);
    for (size_t i = 0; i < samples.size(); ++i) {EXPECT_EQ(samples[i], static_cast<float>(i));}
  }
}

TEST(NeighborhoodSampler, KeepsOutsideAndUnknownSamplesMissing)
{
  grid_map::GridMap map({"elevation"});
  map.setGeometry(grid_map::Length(1, 1), 0.1);
  map["elevation"].setZero();
  const height_obstacle_layer::NeighborhoodSampler sampler(map, "elevation", 0.1);
  const auto edge = sampler.sample(grid_map::Position(0.45, 0.45));
  EXPECT_EQ(edge[0], 0);
  EXPECT_EQ(edge[4], 0);
  for (int i : {2, 5, 6, 7, 8}) {EXPECT_TRUE(std::isnan(edge[i]));}
  map["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
  for (float value : sampler.sample(grid_map::Position(0.025, 0.025))) {
    EXPECT_TRUE(std::isnan(value));
  }
}

TEST(NeighborhoodSampler, RejectsInvalidRadiusAndDoesNotReadRoundedOutsideIndices)
{
  grid_map::GridMap map({"elevation"});
  map.setGeometry(grid_map::Length(7, 5), 0.13, grid_map::Position(0.4, -0.3));
  map["elevation"].setZero();
  EXPECT_THROW(height_obstacle_layer::NeighborhoodSampler(map, "elevation", 0), std::invalid_argument);
  EXPECT_THROW(height_obstacle_layer::NeighborhoodSampler(map, "elevation", INFINITY), std::invalid_argument);
  const height_obstacle_layer::NeighborhoodSampler sampler(map, "elevation", 0.065);
  const grid_map::Position center = map.getPosition() - map.getLength().matrix() / 2;
  for (float value : sampler.sample(center - grid_map::Position(1e20, 1e20))) {
    EXPECT_TRUE(std::isnan(value));
  }
  const auto edge = sampler.sample(center);
  EXPECT_TRUE(std::isnan(edge[4]));  // The center has no valid storage index at this edge.
}
