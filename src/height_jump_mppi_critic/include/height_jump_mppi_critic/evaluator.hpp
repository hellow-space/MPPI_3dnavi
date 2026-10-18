#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

// ROS-independent binary feasibility. There is deliberately no terrain cost function.
namespace height_jump
{
enum class SampleState {Known, Unknown, Outside};
struct Sample
{
  SampleState state{SampleState::Unknown};
  float elevation{0};
};
enum class Decision {Pass, HeightJump, Unknown, Outside, Invalid};
struct Config
{
  double jump_radius{0.10};
  float max_height_jump{0.30f};
  bool unknown_is_traversable{false};
  void validate() const
  {
    if (!std::isfinite(jump_radius) || jump_radius <= 0 ||
      !std::isfinite(max_height_jump) || max_height_jump <= 0)
    {
      throw std::invalid_argument("HeightJump radii and threshold must be finite and positive");
    }
  }
};
struct Result
{
  Decision decision{Decision::Invalid};
  float observed_jump{0};
  int unknown_count{0};
};

class StartupRegion
{
public:
  bool capture(double x, double y)
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {return false;}
    if (!captured_) {x_ = x; y_ = y; captured_ = true;}
    return true;
  }
  bool contains(double x, double y, double radius) const
  {
    return captured_ && std::isfinite(x) && std::isfinite(y) &&
           std::isfinite(radius) && radius > 0 && std::hypot(x - x_, y - y_) <= radius;
  }
  bool captured() const {return captured_;}

private:
  bool captured_{false};
  double x_{0}, y_{0};
};

template<class Query, class AllowUnknown>
Result evaluate(double x, double y, const Config & cfg, Query query, AllowUnknown allow_unknown)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {return {};}
  bool have_height = false, outside = false, all_unknown_exempt = true;
  float lo = 0, hi = 0;
  Result result;
  for (int ix = -1; ix <= 1; ++ix) {
    for (int iy = -1; iy <= 1; ++iy) {
      const double px = x + ix * cfg.jump_radius, py = y + iy * cfg.jump_radius;
      const Sample sample = query(px, py);
      if (sample.state == SampleState::Outside) {outside = true;}
      if (sample.state != SampleState::Known || !std::isfinite(sample.elevation)) {
        ++result.unknown_count;
        all_unknown_exempt = all_unknown_exempt && allow_unknown(px, py);
        continue;
      }
      if (!have_height) {lo = hi = sample.elevation; have_height = true;}
      lo = std::min(lo, sample.elevation);
      hi = std::max(hi, sample.elevation);
    }
  }
  result.observed_jump = have_height ? hi - lo : 0;
  const bool unknown_blocks = !have_height || result.unknown_count >= 3;
  if (have_height && result.observed_jump >= cfg.max_height_jump) {
    result.decision = Decision::HeightJump;
  } else if (outside) {
    result.decision = Decision::Outside;
  } else if (unknown_blocks && !cfg.unknown_is_traversable && !all_unknown_exempt) {
    result.decision = Decision::Unknown;
  } else {
    result.decision = Decision::Pass;
  }
  return result;
}
}  // namespace height_jump
