#include "custom_path_navigation/path_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<custom_path_navigation::PathExecutor>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("custom_path_executor"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
