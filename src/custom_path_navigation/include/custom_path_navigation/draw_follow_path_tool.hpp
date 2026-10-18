#ifndef CUSTOM_PATH_NAVIGATION__DRAW_FOLLOW_PATH_TOOL_HPP_
#define CUSTOM_PATH_NAVIGATION__DRAW_FOLLOW_PATH_TOOL_HPP_

#include <memory>
#include <optional>

#include "custom_path_navigation/interaction_model.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace custom_path_navigation
{
class DrawFollowPathTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  DrawFollowPathTool();
  ~DrawFollowPathTool() override;
  void activate() override;
  void deactivate() override;
  void update(float wall_dt, float ros_dt) override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

protected:
  void onInitialize() override;
  bool eventFilter(QObject * receiver, QEvent * event) override;

private:
  using Trigger = std_srvs::srv::Trigger;
  void checkFixedFrame();
  std::optional<Point2D> projectPoint(rviz_common::ViewportMouseEvent & event) const;
  void publishCandidate();
  void requestService(const rclcpp::Client<Trigger>::SharedPtr & client, const char * label);

  InteractionModel interaction_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_publisher_;
  rclcpp::Client<Trigger>::SharedPtr execute_client_;
  rclcpp::Client<Trigger>::SharedPtr cancel_client_;
  bool active_{false};
};
}  // namespace custom_path_navigation
#endif
