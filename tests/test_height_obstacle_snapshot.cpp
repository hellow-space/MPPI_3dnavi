#include "height_obstacle_layer/snapshot_policy.hpp"

#include <cassert>

int main()
{
  using height_obstacle_layer::shouldAcceptSnapshot;

  constexpr int64_t second = 1000000000LL;
  assert(shouldAcceptSnapshot(9 * second, 10 * second, 2.0, 0));
  assert(!shouldAcceptSnapshot(0, 10 * second, 2.0, 0));
  assert(!shouldAcceptSnapshot(7 * second, 10 * second, 2.0, 0));
  assert(!shouldAcceptSnapshot(11 * second, 10 * second, 2.0, 0));
  assert(!shouldAcceptSnapshot(9 * second, 10 * second, 2.0, 9 * second));
  assert(!shouldAcceptSnapshot(8 * second, 10 * second, 2.0, 9 * second));
  return 0;
}
