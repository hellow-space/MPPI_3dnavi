#include "height_jump_mppi_critic/terrain_map.hpp"

#include <algorithm>
#include <limits>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace height_jump
{
namespace
{
bool validMessage(const grid_map_msgs::msg::GridMap & msg, const std::string & layer)
{
  const auto & info = msg.info;
  const auto & p = info.pose.position;
  const auto & q = info.pose.orientation;
  const bool neutral_rotation = std::isfinite(q.x) && std::isfinite(q.y) &&
    std::isfinite(q.z) && std::isfinite(q.w) && std::abs(q.x) < 1e-6 &&
    std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6 && std::abs(std::abs(q.w) - 1) < 1e-6;
  if (msg.header.frame_id.empty() || !neutral_rotation || !std::isfinite(p.x) ||
    !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(info.resolution) ||
    info.resolution <= 0 || !std::isfinite(info.length_x) || !std::isfinite(info.length_y) ||
    info.length_x <= 0 || info.length_y <= 0 || msg.layers.size() != msg.data.size())
  {
    return false;
  }
  const auto found = std::find(msg.layers.begin(), msg.layers.end(), layer);
  if (found == msg.layers.end() || std::count(msg.layers.begin(), msg.layers.end(), layer) != 1) {
    return false;
  }
  const double nx = std::round(info.length_x / info.resolution);
  const double ny = std::round(info.length_y / info.resolution);
  if (!std::isfinite(nx) || !std::isfinite(ny) || nx < 1 || ny < 1 ||
    nx > std::numeric_limits<int>::max() || ny > std::numeric_limits<int>::max() ||
    msg.outer_start_index >= nx || msg.inner_start_index >= ny)
  {
    return false;
  }
  const auto & data = msg.data[std::distance(msg.layers.begin(), found)];
  if (data.layout.dim.size() != 2 || data.layout.data_offset != 0 ||
    nx * ny != static_cast<double>(data.data.size())) {return false;}
  const auto & dims = data.layout.dim;
  const bool column_major = dims[0].label == "column_index" && dims[1].label == "row_index";
  const bool row_major = dims[0].label == "row_index" && dims[1].label == "column_index";
  return (column_major || row_major) &&
         dims[0].size == (column_major ? ny : nx) &&
         dims[1].size == (column_major ? nx : ny) &&
         dims[0].stride == data.data.size() && dims[1].stride == dims[1].size;
}
}  // namespace

std::shared_ptr<const TerrainSnapshot> TerrainSnapshot::fromMessage(
  const grid_map_msgs::msg::GridMap & msg, const std::string & layer, std::string & error)
{
  error.clear();
  if (!validMessage(msg, layer)) {error = "invalid GridMap geometry, layout or elevation layer"; return {};}
  auto next = std::make_shared<TerrainSnapshot>();
  try {
    if (!grid_map::GridMapRosConverter::fromMessage(msg, next->map, {layer}, false, false)) {
      error = "GridMap conversion failed"; return {};
    }
  } catch (const std::exception & ex) {error = ex.what(); return {};}
  next->layer = layer;
  next->received = std::chrono::steady_clock::now();
  return next;
}

Sample TerrainSnapshot::query(double x, double y) const
{
  if (!std::isfinite(x) || !std::isfinite(y)) {return {SampleState::Outside, 0};}
  const grid_map::Position point(x, y);
  if (!map.isInside(point)) {return {SampleState::Outside, 0};}
  const float z = map.atPosition(layer, point);
  return {std::isfinite(z) ? SampleState::Known : SampleState::Unknown, z};
}

bool TerrainSnapshot::fresh(int64_t now_ns, double timeout_seconds) const
{
  const auto stamp = map.getTimestamp();
  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0 || now_ns <= 0 || stamp == 0 ||
    stamp > static_cast<uint64_t>(now_ns)) {return false;}
  const double observation_age = (static_cast<uint64_t>(now_ns) - stamp) * 1e-9;
  const double receipt_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - received).count();
  return observation_age <= timeout_seconds && receipt_age <= timeout_seconds;
}

TerrainMapSource::TerrainMapSource(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & topic, const std::string & layer)
{
  subscription_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    topic, rclcpp::SensorDataQoS().keep_last(1),
    [cache = cache_, layer, logger = node->get_logger()](grid_map_msgs::msg::GridMap::ConstSharedPtr msg) {
      std::string error;
      auto next = TerrainSnapshot::fromMessage(*msg, layer, error);
      {
        std::lock_guard<std::mutex> lock(cache->mutex);
        cache->latest = std::move(next);  // malformed new input invalidates the old map too
      }
      if (!error.empty()) {RCLCPP_DEBUG(logger, "HeightJump map rejected: %s", error.c_str());}
    });
}

std::shared_ptr<const TerrainSnapshot> TerrainMapSource::snapshot() const
{
  std::lock_guard<std::mutex> lock(cache_->mutex);
  return cache_->latest;
}
}  // namespace height_jump
