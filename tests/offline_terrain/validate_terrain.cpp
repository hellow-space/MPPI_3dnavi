#include "height_jump_mppi_critic/terrain_map.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>

namespace
{
using height_jump::Decision;
using height_jump::Sample;
using height_jump::SampleState;
using height_jump::TerrainSnapshot;
constexpr unsigned kSeed = 20261018;
constexpr size_t kCount = 1000;
constexpr size_t kSteps = 56;
size_t rounded_outside_queries = 0;

void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

grid_map::GridMap makeMap(double resolution, int scene, bool rolling)
{
  grid_map::GridMap map({"elevation"});
  map.setGeometry(grid_map::Length(7.0, 5.0), resolution, grid_map::Position(0.4, -0.3));
  map.setFrameId("terrain");
  map.setTimestamp(10000000000ULL);
  auto & matrix = map["elevation"];
  std::mt19937 random(kSeed);
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      grid_map::Position position;
      require(map.getPosition(grid_map::Index(i, j), position), "fixture index invalid");
      float z = 0;
      if (scene == 1) {z = position.x() > 0 ? 0.4f : 0;}
      if (scene == 2) {
        z = static_cast<float>(static_cast<int>(random() % 100) - 50) / 100;
        if (random() % 5 == 0) {z = std::numeric_limits<float>::quiet_NaN();}
      }
      if (scene == 3) {z = std::numeric_limits<float>::quiet_NaN();}
      matrix(i, j) = z;
    }
  }
  if (rolling) {
    map.move(map.getPosition() + grid_map::Position(3 * resolution, -2 * resolution));
    require((map.getStartIndex() != grid_map::Index::Zero()).any(), "fixture did not roll");
  }
  return map;
}

// Independent of TerrainSnapshot: read the original map before message conversion.
Sample referenceQuery(const grid_map::GridMap & map, double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {return {SampleState::Outside, 0};}
  if (!map.isInside(grid_map::Position(x, y))) {return {SampleState::Outside, 0};}
  grid_map::Index index;
  if (!map.getIndex(grid_map::Position(x, y), index)) {return {SampleState::Outside, 0};}
  // Invalid storage indices have an explicit Outside contract. Count rather than
  // hide the dependency anomaly; a regression test separately reproduces the old bug.
  if ((index < grid_map::Index::Zero()).any() || (index >= map.getSize()).any()) {
    ++rounded_outside_queries;
    return {SampleState::Outside, 0};
  }
  const float value = map.at("elevation", index);
  return {std::isfinite(value) ? SampleState::Known : SampleState::Unknown, value};
}

// Gather first, then decide. This does not call the production evaluate function.
height_jump::Result referenceEvaluate(
  const grid_map::GridMap & map, double x, double y, const height_jump::Config & config,
  const height_jump::StartupRegion & startup, bool exempt)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {return {};}
  std::vector<float> known;
  int unknown = 0;
  bool outside = false;
  bool all_exempt = true;
  for (int ix = -1; ix <= 1; ++ix) {
    for (int iy = -1; iy <= 1; ++iy) {
      const double px = x + ix * config.jump_radius;
      const double py = y + iy * config.jump_radius;
      const auto sample = referenceQuery(map, px, py);
      outside |= sample.state == SampleState::Outside;
      if (sample.state == SampleState::Known) {known.push_back(sample.elevation);} else {
        ++unknown;
        all_exempt &= exempt && startup.contains(px, py, 1.0);
      }
    }
  }
  const float jump = known.empty() ? 0 :
    *std::max_element(known.begin(), known.end()) - *std::min_element(known.begin(), known.end());
  Decision decision = Decision::Pass;
  if (!known.empty() && jump >= config.max_height_jump) {decision = Decision::HeightJump;}
  else if (outside) {decision = Decision::Outside;}
  else if ((known.empty() || unknown >= 3) && !config.unknown_is_traversable && !all_exempt) {
    decision = Decision::Unknown;
  }
  return {decision, jump, unknown};
}

std::shared_ptr<const TerrainSnapshot> snapshot(const grid_map::GridMap & map)
{
  std::string error;
  const auto message = grid_map::GridMapRosConverter::toMessage(map);
  auto result = TerrainSnapshot::fromMessage(*message, "elevation", error);
  require(result != nullptr, "conversion failed: " + error);
  return result;
}

void correctness()
{
  size_t queries = 0, evaluations = 0, failed_fixtures = 0;
  std::mt19937 random(kSeed);
  std::uniform_real_distribution<double> random_x(-4, 4), random_y(-3.5, 3.5);
  height_jump::StartupRegion startup;
  startup.capture(0.4, -0.3);
  for (double resolution : {0.05, 0.10, 0.13}) {
    for (bool rolling : {false, true}) {
      for (int scene = 0; scene < 4; ++scene) {
        try {
        const auto original = makeMap(resolution, scene, rolling);
        const auto actual = snapshot(original);
        const height_jump::TerrainQuery prepared(actual);
        std::vector<grid_map::Position> points;
        for (int i = 0; i < 800; ++i) {points.emplace_back(random_x(random), random_y(random));}
        // Include exact and adjacent representable values around map/cell boundaries.
        const auto center = original.getPosition();
        const auto length = original.getLength();
        const double edge = center.x() + length.x() / 2;
        for (int i = 0; i <= original.getSize().x(); ++i) {
          const double x = edge - i * resolution;
          for (double px : {std::nextafter(x, -INFINITY), x, std::nextafter(x, INFINITY)}) {
            points.emplace_back(px, center.y());
          }
        }
        const double y_edge = center.y() + length.y() / 2;
        for (int i = 0; i <= original.getSize().y(); ++i) {
          const double y = y_edge - i * resolution;
          for (double py : {std::nextafter(y, -INFINITY), y, std::nextafter(y, INFINITY)}) {
            points.emplace_back(center.x(), py);
          }
        }
        points.emplace_back(std::numeric_limits<double>::quiet_NaN(), 0);
        points.emplace_back(0, std::numeric_limits<double>::infinity());
        points.emplace_back(std::numeric_limits<double>::max(), 0);
        points.emplace_back(0, -std::numeric_limits<double>::max());
        for (const auto & point : points) {
          const auto expected = referenceQuery(original, point.x(), point.y());
          const auto observed = actual->query(point.x(), point.y());
          const auto optimized = prepared(point.x(), point.y());
          const std::string context = " resolution=" + std::to_string(resolution) +
            " rolling=" + std::to_string(rolling) + " scene=" + std::to_string(scene) +
            " x=" + std::to_string(point.x()) + " y=" + std::to_string(point.y());
          require(expected.state == observed.state, "query state mismatch" + context);
          require(optimized.state == observed.state, "prepared query state mismatch" + context);
          if (expected.state == SampleState::Known) {
            require(expected.elevation == observed.elevation, "query height mismatch" + context);
            require(optimized.elevation == observed.elevation, "prepared height mismatch" + context);
          }
          ++queries;
          for (double radius : {0.10, 0.17}) {
            for (bool unknown_pass : {false, true}) {
              for (bool exempt : {false, true}) {
                height_jump::Config config{radius, 0.30f, unknown_pass};
                const auto wanted = referenceEvaluate(
                  original, point.x(), point.y(), config, startup, exempt);
                const auto got = height_jump::evaluate(point.x(), point.y(), config,
                  [&actual](double px, double py) {return actual->query(px, py);},
                  [&](double px, double py) {return exempt && startup.contains(px, py, 1.0);});
                const auto optimized_result = height_jump::evaluate(point.x(), point.y(), config,
                  [&prepared](double px, double py) {return prepared(px, py);},
                  [&](double px, double py) {return exempt && startup.contains(px, py, 1.0);});
                require(wanted.decision == got.decision && wanted.unknown_count == got.unknown_count &&
                  wanted.observed_jump == got.observed_jump, "evaluation mismatch" + context);
                require(optimized_result.decision == got.decision &&
                  optimized_result.unknown_count == got.unknown_count &&
                  optimized_result.observed_jump == got.observed_jump,
                  "prepared evaluation mismatch" + context);
                ++evaluations;
              }
            }
          }
        }
        std::cout << "PASS fixture resolution=" << resolution << " rolling=" << rolling
                  << " scene=" << scene << " queries=" << queries << " evaluations="
                  << evaluations << '\n';
        } catch (const std::exception & error) {
          ++failed_fixtures;
          std::cerr << "FAIL fixture resolution=" << resolution << " rolling=" << rolling
                    << " scene=" << scene << ": " << error.what() << '\n';
        }
      }
    }
  }
  std::cout << "seed=" << kSeed << " queries=" << queries
            << " evaluations=" << evaluations << " rounded_outside_queries=" << rounded_outside_queries
            << " failed_fixtures=" << failed_fixtures << '\n';
  require(failed_fixtures == 0, "one or more correctness fixtures failed; see diagnostics above");
}

void printTiming(const char * phase, int scene, std::vector<double> times, size_t checksum)
{
  std::sort(times.begin(), times.end());
  std::cout << "phase=" << phase << " scene=" << scene << " samples=" << times.size()
            << " p50_ms=" << times[49] << " p95_ms=" << times[94]
            << " p99_ms=" << times[98] << " checksum=" << checksum << '\n';
}

template<class Query>
std::vector<Decision> scoreBatch(const std::vector<grid_map::Position> & points, Query query)
{
  std::vector<Decision> decisions(kCount, Decision::Pass);
  for (size_t k = 0; k < kCount; ++k) {
    for (size_t t = 0; t < kSteps; ++t) {
      const auto & p = points[k * kSteps + t];
      decisions[k] = height_jump::evaluate(p.x(), p.y(), height_jump::Config{}, query,
        [](double, double) {return false;}).decision;
      if (decisions[k] != Decision::Pass) {break;}
    }
  }
  return decisions;
}

void benchmark()
{
  std::mt19937 random(kSeed);
  std::uniform_real_distribution<double> x(-2.8, 3.4), y(-2.4, 1.8);
  std::vector<grid_map::Position> points;
  for (size_t i = 0; i < kCount * kSteps; ++i) {points.emplace_back(x(random), y(random));}
  for (int scene : {0, 1, 3}) {
    const auto original = makeMap(0.10, scene, false);
    const auto message = grid_map::GridMapRosConverter::toMessage(original);
    auto map = snapshot(original);
    std::vector<double> conversions;
    std::array<std::vector<double>, 2> scores;
    std::array<size_t, 2> checksums{};
    for (int round = 0; round < 110; ++round) {
      std::string error;
      auto start = std::chrono::steady_clock::now();
      auto next = TerrainSnapshot::fromMessage(*message, "elevation", error);
      auto end = std::chrono::steady_clock::now();
      require(next != nullptr, error);
      if (round >= 10) {conversions.push_back(std::chrono::duration<double, std::milli>(end - start).count());}
      map = std::move(next);
      std::array<std::vector<Decision>, 2> decisions;
      // Alternate order to reduce first-run and cache bias in the paired measurement.
      for (int mode : {round % 2, 1 - round % 2}) {
        start = std::chrono::steady_clock::now();
        if (mode == 0) {
          decisions[mode] = scoreBatch(points, [&map](double px, double py) {return map->query(px, py);});
        } else {
          const height_jump::TerrainQuery prepared(map);
          decisions[mode] = scoreBatch(points, [&prepared](double px, double py) {return prepared(px, py);});
        }
        end = std::chrono::steady_clock::now();
        const size_t passed = std::count(decisions[mode].begin(), decisions[mode].end(), Decision::Pass);
        checksums[mode] += passed;
        if (scene == 0) {require(passed == kCount, "flat benchmark must pass all trajectories");}
        if (scene == 3) {require(passed == 0, "unknown benchmark must reject all trajectories");}
        if (round >= 10) {scores[mode].push_back(std::chrono::duration<double, std::milli>(end - start).count());}
      }
      require(decisions[0] == decisions[1], "baseline/prepared trajectory decisions differ");
    }
    printTiming("map_conversion", scene, conversions, checksums[0]);
    printTiming("baseline_rollout", scene, scores[0], checksums[0]);
    printTiming("prepared_rollout", scene, scores[1], checksums[1]);
  }
  std::cout << "seed=" << kSeed << " map_length=7x5 resolution=0.10 scene0=flat scene1=step scene3=unknown"
            << " trajectories=" << kCount << " steps=" << kSteps
            << " warmup=10 measured=100\n"
            << "Actual TerrainSnapshot query/evaluate and early exit; random synthetic positions, "
            << "not motion-model rollouts. Excludes TF, ROS transport and full MPPI.\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc == 1) {correctness();}
    else if (argc == 2 && std::string(argv[1]) == "--benchmark") {benchmark();}
    else {throw std::runtime_error("usage: validate_terrain [--benchmark]");}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
