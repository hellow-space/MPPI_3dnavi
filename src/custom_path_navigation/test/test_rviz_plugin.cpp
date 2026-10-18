#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include <QApplication>
#include <QKeyEvent>
#include <QWidget>
#include <gtest/gtest.h>
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
constexpr char kClassId[] = "custom_path_navigation/DrawFollowPath";
using Trigger = std_srvs::srv::Trigger;

class TestNode : public rviz_common::ros_integration::RosNodeAbstractionIface
{
public:
  TestNode()
  : node(std::make_shared<rclcpp::Node>("draw_follow_path_tool_test")) {}
  std::string get_node_name() const override {return node->get_name();}
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const override
  {return node->get_topic_names_and_types();}
  rclcpp::Node::SharedPtr get_raw_node() override {return node;}
  rclcpp::Node::SharedPtr node;
};

// Supplies the ROS boundary without constructing a RenderPanel or Ogre scene.
class HeadlessContext : public rviz_common::DisplayContext
{
public:
  HeadlessContext()
  : ros_node(std::make_shared<TestNode>()) {}
  Ogre::SceneManager * getSceneManager() const override {return nullptr;}
  rviz_common::WindowManagerInterface * getWindowManager() const override {return window_manager;}
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
  getSelectionManager() const override {return {};}
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
  getHandlerManager() const override {return {};}
  std::shared_ptr<rviz_common::interaction::ViewPickerIface>
  getViewPicker() const override {return {};}
  rviz_common::FrameManagerIface * getFrameManager() const override {return nullptr;}
  QString getFixedFrame() const override {return "map";}
  uint64_t getFrameCount() const override {return 0;}
  rviz_common::DisplayFactory * getDisplayFactory() const override {return nullptr;}
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
  getRosNodeAbstraction() const override {return ros_node;}
  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}
  rviz_common::ToolManager * getToolManager() const override {return tool_manager;}
  rviz_common::ViewManager * getViewManager() const override {return nullptr;}
  rviz_common::transformation::TransformationManager * getTransformationManager() override
  {return nullptr;}
  rviz_common::DisplayGroup * getRootDisplayGroup() const override {return nullptr;}
  uint32_t getDefaultVisibilityBit() const override {return 0;}
  rviz_common::BitAllocator * visibilityBits() override {return nullptr;}
  void setStatus(const QString & text) override {status = text;}
  QString getHelpPath() const override {return {};}
  std::shared_ptr<rclcpp::Clock> getClock() override {return ros_node->node->get_clock();}
  void lockRender() override {}
  void unlockRender() override {}
  void queueRender() override {}
  std::shared_ptr<TestNode> ros_node;
  QString status;
  rviz_common::WindowManagerInterface * window_manager{nullptr};
  rviz_common::ToolManager * tool_manager{nullptr};
};

class DefaultTool : public rviz_common::Tool
{
public:
  void activate() override {}
  void deactivate() override {}
};

// Replaces only the render widget, not keyboard routing: this calls the installed
// Humble ToolManager, whose Escape branch otherwise switches to the default Tool.
class QtToolManagerReceiver : public QWidget
{
public:
  explicit QtToolManagerReceiver(rviz_common::ToolManager & manager)
  : manager_(manager) {}
  int delivered_keys{0};

protected:
  void keyPressEvent(QKeyEvent * event) override
  {
    ++delivered_keys;
    manager_.handleChar(event, nullptr);
  }

private:
  rviz_common::ToolManager & manager_;
};

class TestWindowManager : public rviz_common::WindowManagerInterface
{
public:
  explicit TestWindowManager(QWidget & window)
  : window_(window) {}
  QWidget * getParentWindow() override {return &window_;}
  rviz_common::PanelDockWidget * addPane(
    const QString &, QWidget *, Qt::DockWidgetArea, bool) override {return nullptr;}
  void setStatus(const QString &) override {}

private:
  QWidget & window_;
};

void sendEscape(QWidget & receiver)
{
  QKeyEvent escape(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
  QApplication::sendEvent(&receiver, &escape);
}

bool spinUntil(const rclcpp::Node::SharedPtr & node, const std::function<bool()> & done)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}
}  // namespace

TEST(RvizPlugin, DiscoversAndConstructsToolWithoutRenderPanel)
{
  pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common", "rviz_common::Tool");
  const auto classes = loader.getDeclaredClasses();
  ASSERT_NE(std::find(classes.begin(), classes.end(), kClassId), classes.end());
  EXPECT_EQ(loader.getClassType(kClassId), "custom_path_navigation::DrawFollowPathTool");
  auto tool = loader.createSharedInstance(kClassId);
  ASSERT_TRUE(tool);
  EXPECT_STREQ(tool->metaObject()->className(), "custom_path_navigation::DrawFollowPathTool");
  EXPECT_TRUE(tool->accessAllKeys());
}

TEST(RvizPlugin, EnterExecutesEscapeCancelsAndKeysKeepToolSelected)
{
  pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common", "rviz_common::Tool");
  ASSERT_TRUE(loader.isClassAvailable(kClassId));
  HeadlessContext context;
  auto node = context.ros_node->node;
  int execute_requests = 0;
  int cancel_requests = 0;
  auto execute = node->create_service<Trigger>(
    "/custom_path/execute", [&execute_requests](
      const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      ++execute_requests;
      response->success = true;
    });
  auto cancel = node->create_service<Trigger>(
    "/custom_path/cancel", [&cancel_requests](
      const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      ++cancel_requests;
      response->success = true;
    });
  auto tool = loader.createSharedInstance(kClassId);
  tool->initialize(&context);
  tool->activate();
  auto probe = node->create_client<Trigger>("/custom_path/execute");
  ASSERT_TRUE(probe->wait_for_service(std::chrono::seconds(3)));
  QKeyEvent enter(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
  EXPECT_EQ(tool->processKeyEvent(&enter, nullptr) & rviz_common::Tool::Finished, 0);
  ASSERT_TRUE(spinUntil(node, [&]() {return execute_requests == 1;}));
  QKeyEvent keypad_enter(QEvent::KeyPress, Qt::Key_Enter, Qt::NoModifier);
  EXPECT_EQ(tool->processKeyEvent(&keypad_enter, nullptr) & rviz_common::Tool::Finished, 0);
  ASSERT_TRUE(spinUntil(node, [&]() {return execute_requests == 2;}));
  QKeyEvent escape(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
  EXPECT_EQ(tool->processKeyEvent(&escape, nullptr) & rviz_common::Tool::Finished, 0);
  ASSERT_TRUE(spinUntil(node, [&]() {return cancel_requests == 1;}));
  QKeyEvent undo(QEvent::KeyPress, Qt::Key_Z, Qt::ControlModifier);
  EXPECT_EQ(tool->processKeyEvent(&undo, nullptr) & rviz_common::Tool::Finished, 0);
  QKeyEvent unrelated(QEvent::KeyPress, Qt::Key_A, Qt::NoModifier);
  EXPECT_EQ(tool->processKeyEvent(&unrelated, nullptr), 0);
  rclcpp::spin_some(node);
  EXPECT_EQ(execute_requests, 2);
  EXPECT_EQ(cancel_requests, 1);
  tool->deactivate();
}

TEST(RvizPlugin, QtDispatchCancelsBeforeHumbleToolManagerCanSwitchTheActiveTool)
{
  pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common", "rviz_common::Tool");
  HeadlessContext context;
  auto node = context.ros_node->node;
  int cancel_requests = 0;
  auto cancel = node->create_service<Trigger>(
    "/custom_path/cancel", [&cancel_requests](
      const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      ++cancel_requests;
      response->success = true;
    });
  auto tool = loader.createSharedInstance(kClassId);
  tool->initialize(&context);
  DefaultTool default_tool;
  rviz_common::ToolManager manager(&context);
  context.tool_manager = &manager;
  QtToolManagerReceiver receiver(manager);
  TestWindowManager window_manager(receiver);
  context.window_manager = &window_manager;
  manager.setDefaultTool(&default_tool);
  manager.setCurrentTool(tool.get());
  auto probe = node->create_client<Trigger>("/custom_path/cancel");
  ASSERT_TRUE(probe->wait_for_service(std::chrono::seconds(3)));

  // An unrelated top-level widget must retain its own Escape behavior.
  QWidget unrelated;
  sendEscape(unrelated);
  rclcpp::spin_some(node);
  EXPECT_EQ(cancel_requests, 0);

  sendEscape(receiver);
  EXPECT_EQ(manager.getCurrentTool(), tool.get());
  EXPECT_EQ(receiver.delivered_keys, 0);
  EXPECT_TRUE(spinUntil(node, [&]() {return cancel_requests == 1;}));

  // Deactivation removes the filter: the real ToolManager receives Escape again.
  manager.setCurrentTool(&default_tool);
  sendEscape(receiver);
  rclcpp::spin_some(node);
  EXPECT_EQ(manager.getCurrentTool(), &default_tool);
  EXPECT_EQ(receiver.delivered_keys, 1);
  EXPECT_EQ(cancel_requests, 1);

  // Reactivation must install the filter again without duplicating cancellation.
  manager.setCurrentTool(tool.get());
  sendEscape(receiver);
  EXPECT_EQ(manager.getCurrentTool(), tool.get());
  EXPECT_EQ(receiver.delivered_keys, 1);
  EXPECT_TRUE(spinUntil(node, [&]() {return cancel_requests == 2;}));
  manager.setCurrentTool(nullptr);
  manager.setDefaultTool(nullptr);
}

TEST(RvizPlugin, DestroyingAnActiveToolLeavesNoQtEventFilter)
{
  pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common", "rviz_common::Tool");
  HeadlessContext context;
  DefaultTool default_tool;
  rviz_common::ToolManager manager(&context);
  QtToolManagerReceiver receiver(manager);
  TestWindowManager window_manager(receiver);
  context.window_manager = &window_manager;
  manager.setDefaultTool(&default_tool);
  manager.setCurrentTool(&default_tool);
  auto tool = loader.createSharedInstance(kClassId);
  tool->initialize(&context);
  tool->activate();
  tool.reset();
  sendEscape(receiver);
  EXPECT_EQ(receiver.delivered_keys, 1);
  EXPECT_EQ(manager.getCurrentTool(), &default_tool);
  manager.setCurrentTool(nullptr);
  manager.setDefaultTool(nullptr);
}

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
