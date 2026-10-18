#pragma once

#include <cmath>
#include <cstdint>

namespace height_obstacle_layer
{

inline bool isSnapshotTimestampFresh(
  const int64_t stamp_ns, const int64_t now_ns, const double timeout_seconds)
{
  if (stamp_ns <= 0 || now_ns <= 0 || stamp_ns > now_ns ||
    !std::isfinite(timeout_seconds) || timeout_seconds <= 0.0)
  {
    return false;
  }
  const double age_seconds = static_cast<double>(now_ns - stamp_ns) * 1.0e-9;
  return age_seconds <= timeout_seconds;
}

inline bool shouldAcceptSnapshot(
  const int64_t candidate_stamp_ns, const int64_t now_ns,
  const double timeout_seconds, const int64_t latest_stamp_ns)
{
  return isSnapshotTimestampFresh(candidate_stamp_ns, now_ns, timeout_seconds) &&
         (latest_stamp_ns <= 0 || candidate_stamp_ns > latest_stamp_ns);
}

}  // namespace height_obstacle_layer
