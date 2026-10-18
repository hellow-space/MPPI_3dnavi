#pragma once

#include <nav2_mppi_controller/critic_function.hpp>
#include "height_jump_mppi_critic/evaluator.hpp"
#include "height_jump_mppi_critic/terrain_map.hpp"

namespace mppi::critics
{
class HeightJumpCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  height_jump::Config config_;
  height_jump::StartupRegion startup_;
  std::unique_ptr<height_jump::TerrainMapSource> source_;
  std::string fixed_frame_;
  double map_timeout_{1.0}, startup_radius_{0.0};
  bool startup_enabled_{false}, clock_invalid_{false};
  int64_t last_time_ns_{0};
};
}  // namespace mppi::critics
