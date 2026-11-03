#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <grid_map_core/GridMap.hpp>

namespace height_obstacle_layer
{

// Borrows a map that stays alive and keeps its layers/geometry during one scan.
class NeighborhoodSampler
{
public:
  NeighborhoodSampler(const grid_map::GridMap & map, const std::string & layer, double radius)
  : map_(map), elevation_(map[layer]), radius_(radius)
  {
    if (!std::isfinite(radius_) || radius_ <= 0) {
      throw std::invalid_argument("neighborhood radius must be finite and positive");
    }
  }

  std::array<float, 9> sample(const grid_map::Position & center) const
  {
    static constexpr std::array<std::pair<int, int>, 9> offsets{{
      {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 0}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}};
    std::array<float, 9> values;
    values.fill(std::numeric_limits<float>::quiet_NaN());
    for (size_t i = 0; i < offsets.size(); ++i) {
      const grid_map::Position p(center.x() + offsets[i].first * radius_,
        center.y() + offsets[i].second * radius_);
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !map_.isInside(p)) {continue;}
      grid_map::Index index;
      if (!map_.getIndex(p, index) || (index < grid_map::Index::Zero()).any() ||
        (index >= map_.getSize()).any()) {continue;}
      values[i] = elevation_(index.x(), index.y());
    }
    return values;
  }

private:
  const grid_map::GridMap & map_;
  const grid_map::Matrix & elevation_;
  double radius_;
};

}  // namespace height_obstacle_layer
