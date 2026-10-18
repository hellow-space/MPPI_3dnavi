#ifndef CUSTOM_PATH_NAVIGATION__PATH_EXECUTOR_HPP_
#define CUSTOM_PATH_NAVIGATION__PATH_EXECUTOR_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace custom_path_navigation
{
class PathExecutor : public rclcpp::Node
{
public:
  explicit PathExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PathExecutor() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace custom_path_navigation
#endif
