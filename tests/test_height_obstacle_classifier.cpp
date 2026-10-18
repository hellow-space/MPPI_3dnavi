#include "height_obstacle_layer/height_difference.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <limits>

int main()
{
  using height_obstacle_layer::classifyNeighborhood;

  const std::array<float, 9> flat{1.00F, 1.01F, 0.99F, 1.00F, 1.00F,
    1.00F, 1.01F, 0.99F, 1.00F};
  const auto flat_result = classifyNeighborhood(flat, 0.30F);
  assert(flat_result.has_known_height);
  assert(!flat_result.is_obstacle);
  assert(std::abs(flat_result.height_difference - 0.02F) < 1.0e-5F);

  const std::array<float, 9> exact_threshold{0.0F, 0.30F, 0.1F, 0.2F, 0.0F,
    0.1F, 0.2F, 0.1F, 0.0F};
  const auto threshold_result = classifyNeighborhood(exact_threshold, 0.30F);
  assert(threshold_result.is_obstacle);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::array<float, 9> all_unknown{nan, nan, nan, nan, nan, nan, nan, nan, nan};
  const auto unknown_result = classifyNeighborhood(all_unknown, 0.30F);
  assert(!unknown_result.has_known_height);
  assert(!unknown_result.is_obstacle);
  assert(std::isnan(unknown_result.height_difference));

  const std::array<float, 9> partly_known{nan, 0.0F, nan, 0.31F, nan, nan, nan, nan, nan};
  const auto partly_known_result = classifyNeighborhood(partly_known, 0.30F);
  assert(partly_known_result.has_known_height);
  assert(partly_known_result.is_obstacle);
  assert(std::abs(partly_known_result.height_difference - 0.31F) < 1.0e-5F);

  return 0;
}
