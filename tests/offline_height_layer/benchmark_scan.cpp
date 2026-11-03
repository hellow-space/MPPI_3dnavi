#include "height_obstacle_layer/neighborhood_sampler.hpp"
#include "height_obstacle_layer/height_difference.hpp"
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using height_obstacle_layer::NeighborhoodResult;

// Frozen pre-optimization sampling loop. Benchmark geometry is preflighted so
// this exact old path never receives the dependency's invalid boundary index.
std::array<float, 9> legacySamples(grid_map::GridMap & map, const grid_map::Position & center,
  const std::string & layer, double radius)
{
  static constexpr std::array<std::pair<int, int>, 9> offsets{{
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 0}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}};
  std::array<float, 9> values;
  values.fill(std::numeric_limits<float>::quiet_NaN());
  for (size_t i = 0; i < offsets.size(); ++i) {
    const grid_map::Position p(center.x() + offsets[i].first * radius,
      center.y() + offsets[i].second * radius);
    if (map.isInside(p)) {values[i] = map.atPosition(layer, p);}
  }
  return values;
}

template<class Read>
std::vector<NeighborhoodResult> scan(grid_map::GridMap & map, const Read & read)
{
  std::vector<NeighborhoodResult> results;
  results.reserve(map.getSize().prod());
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position center;
    if (!map.getPosition(*iterator, center)) {continue;}
    results.push_back(height_obstacle_layer::classifyNeighborhood(read(center), 0.30f));
  }
  return results;
}

void preflight(grid_map::GridMap & map, double radius)
{
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position center;
    map.getPosition(*iterator, center);
    for (int ix = -1; ix <= 1; ++ix) {
      for (int iy = -1; iy <= 1; ++iy) {
        const grid_map::Position p(center.x() + ix * radius, center.y() + iy * radius);
        if (!map.isInside(p)) {continue;}
        grid_map::Index index;
        if (!map.getIndex(p, index) || (index < grid_map::Index::Zero()).any() ||
          (index >= map.getSize()).any()) {throw std::runtime_error("unsafe legacy benchmark geometry");}
      }
    }
  }
}

void run()
{
  const std::string layer = "elevation";
  const double radius = 0.10;
  for (double resolution : {0.10, 0.05}) {
    for (int scene = 0; scene < 3; ++scene) {
      grid_map::GridMap map({"elevation"});
      map.setGeometry(grid_map::Length(7, 7), resolution);
      map["elevation"].setZero();
      if (scene == 1) {
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          grid_map::Position p;
          map.getPosition(*iterator, p);
          map.at("elevation", *iterator) = p.x() > 0 ? 0.4f : 0;
        }
      }
      if (scene == 2) {map["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());}
      preflight(map, radius);
      std::array<std::vector<double>, 2> times;
      size_t checksum = 0;
      for (int round = 0; round < 110; ++round) {
        std::array<std::vector<NeighborhoodResult>, 2> results;
        for (int mode : {round % 2, 1 - round % 2}) {
          const auto start = std::chrono::steady_clock::now();
          if (mode == 0) {
            results[mode] = scan(map, [&](const auto & p) {return legacySamples(map, p, layer, radius);});
          } else {
            const height_obstacle_layer::NeighborhoodSampler sampler(map, layer, radius);
            results[mode] = scan(map, [&](const auto & p) {return sampler.sample(p);});
          }
          const auto end = std::chrono::steady_clock::now();
          if (round >= 10) {times[mode].push_back(std::chrono::duration<double, std::milli>(end - start).count());}
        }
        if (results[0].size() != results[1].size()) {throw std::runtime_error("scan size mismatch");}
        for (size_t i = 0; i < results[0].size(); ++i) {
          const auto & a = results[0][i];
          const auto & b = results[1][i];
          const bool same_height = a.height_difference == b.height_difference ||
            (std::isnan(a.height_difference) && std::isnan(b.height_difference));
          if (!same_height || a.is_obstacle != b.is_obstacle || a.has_known_height != b.has_known_height) {
            throw std::runtime_error("scan result mismatch");
          }
          checksum += b.has_known_height + b.is_obstacle;
        }
      }
      for (auto & values : times) {std::sort(values.begin(), values.end());}
      std::cout << "resolution=" << resolution << " scene=" << scene
                << " cells=" << map.getSize().prod() << " baseline_p50_ms=" << times[0][49]
                << " optimized_p50_ms=" << times[1][49] << " baseline_p99_ms=" << times[0][98]
                << " optimized_p99_ms=" << times[1][98] << " checksum=" << checksum << '\n';
    }
  }
  std::cout << "warmup=10 measured=100 order=alternating radius=0.10 threshold=0.30 "
            << "scene0=flat scene1=step scene2=unknown; scan kernel only, no conversion, ROS, "
            << "difference publication, costmap update or full navigation.\n";
}
}  // namespace

int main()
{
  try {run(); return 0;} catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
