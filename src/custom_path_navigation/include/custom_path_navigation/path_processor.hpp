#ifndef CUSTOM_PATH_NAVIGATION__PATH_PROCESSOR_HPP_
#define CUSTOM_PATH_NAVIGATION__PATH_PROCESSOR_HPP_

#include <string>

#include "nav_msgs/msg/path.hpp"

namespace custom_path_navigation
{

struct ProcessingConfig
{
  double min_raw_spacing{0.02};
  double min_path_length{0.20};
  double sample_spacing{0.05};
  int smoothing_passes{1};
};

struct ProcessingResult
{
  bool valid{false};
  std::string error;
  nav_msgs::msg::Path path;
};

/// Process an entire stroke in its header frame without changing either endpoint.
/// Empty pose frame IDs inherit the path frame; conflicting frame IDs are rejected.
/// The input path timestamp is reused for every output pose, keeping this API pure.
ProcessingResult processPath(
  const nav_msgs::msg::Path & raw_path, const ProcessingConfig & config);

}  // namespace custom_path_navigation

#endif  // CUSTOM_PATH_NAVIGATION__PATH_PROCESSOR_HPP_
