#include "height_jump_mppi_critic/evaluator.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <vector>

// Synthetic query deliberately excludes GridMap/TF/ROS overhead.
int main()
{
  using namespace height_jump;
  Config config;
  std::vector<double> x(1000 * 56), y(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    x[i] = static_cast<double>((i * 17) % 600) / 100 - 3;
    y[i] = static_cast<double>((i * 31) % 600) / 100 - 3;
  }
  for (int scene = 0; scene < 3; ++scene) {
    const auto query = [scene](double px, double py) -> Sample {
        if (scene == 2 && px > 1) {
          return {SampleState::Unknown, std::numeric_limits<float>::quiet_NaN()};
        }
        return {SampleState::Known, scene == 1 && px > 0 ? 0.4f :
          static_cast<float>(0.01 * py)};
      };
    std::vector<double> times;
    size_t checksum = 0;
    for (int round = 0; round < 110; ++round) {
      const auto start = std::chrono::steady_clock::now();
      size_t passed = 0;
      for (size_t i = 0; i < x.size(); ++i) {
        passed += evaluate(x[i], y[i], config, query,
          [](double, double) {return false;}).decision == Decision::Pass;
      }
      const auto end = std::chrono::steady_clock::now();
      checksum += passed;
      if (scene == 0 && passed != x.size()) {return 1;}
      if (round >= 10) {
        times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
      }
    }
    std::sort(times.begin(), times.end());
    std::cout << "scene=" << scene << " points_per_batch=" << x.size()
              << " p50_ms=" << times[49] << " p95_ms=" << times[94]
              << " p99_ms=" << times[98] << " checksum=" << checksum << '\n';
  }
  std::cout << "Synthetic evaluator only; no trajectory early exit, GridMap, TF or controller.\n";
}
