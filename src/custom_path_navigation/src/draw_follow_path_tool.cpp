#include "custom_path_navigation/draw_follow_path_tool.hpp"

#include <cmath>
#include <string>

#include <QKeyEvent>
#include <QCoreApplication>
#include <QWidget>
#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreViewport.h>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_rendering/render_window.hpp"

namespace custom_path_navigation
{
DrawFollowPathTool::DrawFollowPathTool()
{
  shortcut_key_ = 'd';
  access_all_keys_ = true;
}

DrawFollowPathTool::~DrawFollowPathTool()
{
  deactivate();
}

void DrawFollowPathTool::onInitialize()
{
  setName("Draw Follow Path");
  const auto abstraction = context_->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("Draw Follow Path: RViz ROS node is unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  raw_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/custom_path/raw_path", rclcpp::QoS(1).reliable());
  execute_client_ = node_->create_client<Trigger>("/custom_path/execute");
  cancel_client_ = node_->create_client<Trigger>("/custom_path/cancel");
}

void DrawFollowPathTool::activate()
{
  active_ = true;
  if (auto * application = QCoreApplication::instance()) {
    application->installEventFilter(this);
  }
  checkFixedFrame();
  setStatus("Drag left mouse to draw; Enter executes; Esc clears/cancels; Ctrl+Z undoes");
}

void DrawFollowPathTool::deactivate()
{
  active_ = false;
  if (auto * application = QCoreApplication::instance()) {
    application->removeEventFilter(this);
  }
  interaction_.cancelStroke();
}

bool DrawFollowPathTool::eventFilter(QObject * receiver, QEvent * event)
{
  if (!active_ || !context_ || event->type() != QEvent::KeyPress ||
    static_cast<QKeyEvent *>(event)->key() != Qt::Key_Escape)
  {
    return false;
  }
  auto * manager = context_->getToolManager();
  if (manager && manager->getCurrentTool() != this) {
    return false;
  }
  // Limit the application filter to this RViz window and its render panels.
  // Other top-level windows (including dialogs) keep their own Escape behavior.
  bool rviz_receiver = false;
  auto * window_manager = context_->getWindowManager();
  auto * widget = qobject_cast<QWidget *>(receiver);
  if (widget && window_manager && window_manager->getParentWindow()) {
    rviz_receiver = widget->window() == window_manager->getParentWindow()->window();
  }
  for (auto * object = receiver; !rviz_receiver && object; object = object->parent()) {
    if (auto * panel = qobject_cast<rviz_common::RenderPanel *>(object)) {
      rviz_receiver = panel->getManager() == context_;
    }
  }
  if (!rviz_receiver) {
    return false;
  }
  // Humble ToolManager handles Escape before processKeyEvent/accessAllKeys.
  // Consume it here so clear/cancel happens before the default-Tool switch.
  processKeyEvent(static_cast<QKeyEvent *>(event), nullptr);
  event->accept();
  context_->queueRender();
  return true;
}

void DrawFollowPathTool::checkFixedFrame()
{
  if (context_) {
    const bool was_drawing = interaction_.drawing();
    interaction_.frameChanged(context_->getFixedFrame().toStdString());
    if (was_drawing && !interaction_.drawing()) {
      setStatus("Fixed Frame changed: current stroke discarded; draw again");
    }
  }
}

void DrawFollowPathTool::update(float, float)
{
  checkFixedFrame();
}

std::optional<Point2D> DrawFollowPathTool::projectPoint(
  rviz_common::ViewportMouseEvent & event) const
{
  if (!event.panel) {
    return std::nullopt;
  }
  auto * window = event.panel->getRenderWindow();
  if (!window) {
    return std::nullopt;
  }
  auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
  auto * camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(window);
  if (!viewport || !camera || viewport->getActualWidth() == 0 ||
    viewport->getActualHeight() == 0)
  {
    return std::nullopt;
  }
  // Humble ViewportMouseEvent coordinates already include the device pixel ratio.
  const auto x = static_cast<Ogre::Real>(event.x - viewport->getActualLeft()) /
    viewport->getActualWidth();
  const auto y = static_cast<Ogre::Real>(event.y - viewport->getActualTop()) /
    viewport->getActualHeight();
  const auto ray = camera->getCameraToViewportRay(x, y);
  const auto intersection = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, 0));
  if (!intersection.first || intersection.second < 0) {
    return std::nullopt;
  }
  const auto point = ray.getPoint(intersection.second);
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return std::nullopt;
  }
  return Point2D{point.x, point.y};
}

int DrawFollowPathTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!context_) {
    return 0;
  }
  checkFixedFrame();
  if (event.leftDown()) {
    interaction_.beginStroke(context_->getFixedFrame().toStdString());
  }
  if (interaction_.drawing() && (event.left() || event.leftUp())) {
    if (const auto point = projectPoint(event)) {
      interaction_.append(*point);
    }
  }
  if (event.leftUp()) {
    if (interaction_.finishStroke()) {
      publishCandidate();
      setStatus("Candidate published for preview; Enter executes; Ctrl+Z undoes");
    }
    return Render;
  }
  return interaction_.drawing() ? Render : 0;
}

void DrawFollowPathTool::publishCandidate()
{
  const auto & candidate = interaction_.currentPath();
  if (!candidate || !raw_path_publisher_ || !node_) {
    return;
  }
  nav_msgs::msg::Path path;
  path.header.frame_id = candidate->frame;
  path.header.stamp = node_->now();
  path.poses.reserve(candidate->points.size());
  for (const auto & point : candidate->points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  raw_path_publisher_->publish(path);
}

void DrawFollowPathTool::requestService(
  const rclcpp::Client<Trigger>::SharedPtr & client, const char * label)
{
  if (!client || !client->service_is_ready()) {
    setStatus(QString("%1 service is unavailable").arg(label));
    return;
  }
  // The ROS callback may outlive the selected Tool; capture no GUI object.
  client->async_send_request(
    std::make_shared<Trigger::Request>(),
    [node = node_, operation = std::string(label)](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      if (!response->success) {
        RCLCPP_WARN(
          node->get_logger(), "%s rejected: %s", operation.c_str(),
          response->message.c_str());
      }
    });
}

int DrawFollowPathTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel *)
{
  if (!event) {
    return 0;
  }
  checkFixedFrame();
  if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
    requestService(execute_client_, "Execute");
    return Render;
  }
  if (event->key() == Qt::Key_Escape) {
    interaction_.clear();
    requestService(cancel_client_, "Cancel");
    return Render;
  }
  if (event->key() == Qt::Key_Z && event->modifiers().testFlag(Qt::ControlModifier)) {
    if (interaction_.undo()) {
      publishCandidate();
    }
    return Render;
  }
  return 0;
}
}  // namespace custom_path_navigation

PLUGINLIB_EXPORT_CLASS(custom_path_navigation::DrawFollowPathTool, rviz_common::Tool)
