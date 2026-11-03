cmake_minimum_required(VERSION 3.16)
project(jump_costmap_reset_test)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

add_executable(test_jump_costmap_reset test_jump_costmap_reset.cpp)
target_compile_features(test_jump_costmap_reset PRIVATE cxx_std_17)
ament_target_dependencies(test_jump_costmap_reset nav2_costmap_2d)

enable_testing()
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_test(NAME jump_costmap_reset COMMAND test_jump_costmap_reset
  "${PROJECT_ROOT}/src/fishbot_navigation2/config/nav2_params_height_jump.yaml")
set_tests_properties(jump_costmap_reset PROPERTIES
  ENVIRONMENT "ROS_DOMAIN_ID=87;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/jump_costmap_reset_test_logs"
  TIMEOUT 30)
