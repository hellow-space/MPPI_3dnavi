#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace height_obstacle_layer
{

struct NeighborhoodResult
{
  bool has_known_height{false};
  bool is_obstacle{false};
  float height_difference{std::numeric_limits<float>::quiet_NaN()};
};

// Unknown (NaN) samples do not create obstacles. Any known local max-min
// difference at or above the threshold is a binary obstacle.
template<typename Samples>
NeighborhoodResult classifyNeighborhood(const Samples & samples, const float threshold)
{
  NeighborhoodResult result;
  float minimum = std::numeric_limits<float>::infinity();
  float maximum = -std::numeric_limits<float>::infinity();

  for (const float height : samples) {
    if (!std::isfinite(height)) {
      continue;
    }
    result.has_known_height = true;
    minimum = std::min(minimum, height);
    maximum = std::max(maximum, height);
  }

  if (result.has_known_height) {
    result.height_difference = maximum - minimum;
    result.is_obstacle = result.height_difference >= threshold;
  }
  return result;
}

}  // namespace height_obstacle_layer
