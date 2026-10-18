#include <gtest/gtest.h>

#include "height_obstacle_layer/snapshot_policy.hpp"

TEST(SnapshotPolicy, RejectsStaleZeroFutureAndOutOfOrderMaps)
{
  using height_obstacle_layer::shouldAcceptSnapshot;
  constexpr int64_t second = 1000000000LL;

  EXPECT_TRUE(shouldAcceptSnapshot(9 * second, 10 * second, 2.0, 0));
  EXPECT_FALSE(shouldAcceptSnapshot(0, 10 * second, 2.0, 0));
  EXPECT_FALSE(shouldAcceptSnapshot(7 * second, 10 * second, 2.0, 0));
  EXPECT_FALSE(shouldAcceptSnapshot(11 * second, 10 * second, 2.0, 0));
  EXPECT_FALSE(shouldAcceptSnapshot(9 * second, 10 * second, 2.0, 9 * second));
  EXPECT_FALSE(shouldAcceptSnapshot(8 * second, 10 * second, 2.0, 9 * second));
}
