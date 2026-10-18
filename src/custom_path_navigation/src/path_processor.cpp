#include "custom_path_navigation/path_processor.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace custom_path_navigation
{
namespace
{

struct Point2D
{
  double x;
  double y;
};

bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

double distance(const Point2D & first, const Point2D & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

ProcessingResult invalid(const std::string & error)
{
  ProcessingResult result;
  result.error = error;
  return result;
}

bool cumulativeLengths(const std::vector<Point2D> & points, std::vector<double> & lengths)
{
  lengths.clear();
  lengths.reserve(points.size());
  lengths.push_back(0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    const double length = lengths.back() + distance(points[index - 1], points[index]);
    if (!std::isfinite(length)) {
      return false;
    }
    lengths.push_back(length);
  }
  return true;
}

}  // namespace

ProcessingResult processPath(
  const nav_msgs::msg::Path & raw_path, const ProcessingConfig & config)
{
  if (!finitePositive(config.min_raw_spacing) || !finitePositive(config.min_path_length) ||
    !finitePositive(config.sample_spacing) || config.smoothing_passes < 0)
  {
    return invalid("Distances must be finite and positive; smoothing_passes must be nonnegative.");
  }
  if (raw_path.header.frame_id.empty()) {
    return invalid("Path frame_id must not be empty.");
  }
  for (const auto & pose : raw_path.poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != raw_path.header.frame_id) {
      return invalid("All poses must use the path frame.");
    }
    const auto & position = pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      return invalid("Path coordinates must be finite.");
    }
  }
  if (raw_path.poses.size() < 2U) {
    return invalid("Path requires at least two points.");
  }

  std::vector<Point2D> filtered;
  filtered.reserve(raw_path.poses.size());
  filtered.push_back(
    {raw_path.poses.front().pose.position.x,
      raw_path.poses.front().pose.position.y});
  for (std::size_t index = 1; index + 1U < raw_path.poses.size(); ++index) {
    const auto & position = raw_path.poses[index].pose.position;
    const Point2D point{position.x, position.y};
    if (distance(filtered.back(), point) >= config.min_raw_spacing) {
      filtered.push_back(point);
    }
  }
  const auto & terminal_position = raw_path.poses.back().pose.position;
  const Point2D terminal{terminal_position.x, terminal_position.y};
  // Keep the exact user-selected endpoint. Prefer it over nearby interior samples.
  while (filtered.size() > 1U && distance(filtered.back(), terminal) < config.min_raw_spacing) {
    filtered.pop_back();
  }
  if (distance(filtered.back(), terminal) > 0.0) {
    filtered.push_back(terminal);
  }
  if (filtered.size() < 2U) {
    return invalid("Filtering left no nonzero path segment.");
  }

  std::vector<double> lengths;
  if (!cumulativeLengths(filtered, lengths)) {
    return invalid("Path length is not finite.");
  }
  if (lengths.back() < config.min_path_length) {
    return invalid("Path is shorter than min_path_length.");
  }

  for (int pass = 0; pass < config.smoothing_passes && filtered.size() > 2U; ++pass) {
    auto smoothed = filtered;
    for (std::size_t index = 1; index + 1U < filtered.size(); ++index) {
      // Wider accumulation prevents overflow when averaging finite coordinates.
      smoothed[index].x = static_cast<double>(
        (static_cast<long double>(filtered[index - 1].x) + filtered[index].x +
        filtered[index + 1].x) / 3.0L);
      smoothed[index].y = static_cast<double>(
        (static_cast<long double>(filtered[index - 1].y) + filtered[index].y +
        filtered[index + 1].y) / 3.0L);
    }
    filtered.swap(smoothed);
  }

  // Skip coincident points produced by smoothing before interpolation or tangent calculation.
  std::vector<Point2D> polyline;
  polyline.reserve(filtered.size());
  polyline.push_back(filtered.front());
  for (std::size_t index = 1; index < filtered.size(); ++index) {
    if (distance(polyline.back(), filtered[index]) > 0.0) {
      polyline.push_back(filtered[index]);
    }
  }
  if (polyline.size() < 2U) {
    return invalid("Smoothing left no nonzero path segment.");
  }
  if (!cumulativeLengths(polyline, lengths) || lengths.back() <= 0.0) {
    return invalid("Processed path length must be finite and nonzero.");
  }

  const double total_length = lengths.back();
  const double full_samples = std::floor(total_length / config.sample_spacing);
  std::vector<Point2D> sampled;
  if (!std::isfinite(full_samples) ||
    static_cast<long double>(full_samples) > static_cast<long double>(sampled.max_size() - 2U))
  {
    return invalid("sample_spacing would produce too many path points.");
  }
  const auto sample_count = static_cast<std::size_t>(full_samples);
  sampled.reserve(sample_count + 2U);
  sampled.push_back(polyline.front());
  std::size_t segment = 1U;
  const double endpoint_roundoff = 8.0 * std::numeric_limits<double>::epsilon() * total_length;
  for (std::size_t index = 1U; index <= sample_count; ++index) {
    const double arc = static_cast<double>(index) * config.sample_spacing;
    if (arc >= total_length || total_length - arc <= endpoint_roundoff) {
      break;
    }
    while (segment + 1U < lengths.size() && lengths[segment] < arc) {
      ++segment;
    }
    const double segment_length = lengths[segment] - lengths[segment - 1U];
    if (segment_length <= 0.0) {
      continue;
    }
    const double fraction = (arc - lengths[segment - 1U]) / segment_length;
    const auto & first = polyline[segment - 1U];
    const auto & second = polyline[segment];
    const Point2D point{
      (1.0 - fraction) * first.x + fraction * second.x,
      (1.0 - fraction) * first.y + fraction * second.y};
    if (distance(sampled.back(), point) > 0.0) {
      sampled.push_back(point);
    }
  }
  // Reuse a coincident arc sample so the terminal tangent has a nonzero segment.
  if (distance(sampled.back(), polyline.back()) > 0.0) {
    sampled.push_back(polyline.back());
  } else {
    sampled.back() = polyline.back();
  }

  ProcessingResult result;
  result.path.header = raw_path.header;
  result.path.poses.reserve(sampled.size());
  for (std::size_t index = 0; index < sampled.size(); ++index) {
    const std::size_t previous = index == 0U ? 0U : index - 1U;
    const std::size_t next = index + 1U < sampled.size() ? index + 1U : index;
    double dx = sampled[next].x - sampled[previous].x;
    double dy = sampled[next].y - sampled[previous].y;
    if (dx == 0.0 && dy == 0.0 && next != index) {
      dx = sampled[next].x - sampled[index].x;
      dy = sampled[next].y - sampled[index].y;
    }
    if (dx == 0.0 && dy == 0.0 && previous != index) {
      dx = sampled[index].x - sampled[previous].x;
      dy = sampled[index].y - sampled[previous].y;
    }
    if (!std::isfinite(dx) || !std::isfinite(dy) || (dx == 0.0 && dy == 0.0)) {
      return invalid("Resampled path has no finite nonzero tangent.");
    }
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, std::atan2(dy, dx));
    orientation.normalize();
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.path.header;
    pose.pose.position.x = sampled[index].x;
    pose.pose.position.y = sampled[index].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(orientation);
    result.path.poses.push_back(pose);
  }
  result.valid = true;
  return result;
}

}  // namespace custom_path_navigation
