cmake_minimum_required(VERSION 3.16)
project(custom_path_navigation)

find_package(ament_cmake REQUIRED)
set(dependencies geometry_msgs nav_msgs tf2 tf2_geometry_msgs nav2_msgs rclcpp
  rclcpp_action std_msgs std_srvs visualization_msgs tf2_ros)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(path_processor SHARED src/path_processor.cpp)
target_compile_features(path_processor PUBLIC cxx_std_17)
target_compile_options(path_processor PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math)
target_include_directories(path_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_processor ${dependencies})

add_library(execution_model SHARED src/execution_model.cpp)
target_compile_features(execution_model PUBLIC cxx_std_17)
target_compile_options(execution_model PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(execution_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(interaction_model SHARED src/interaction_model.cpp)
target_compile_features(interaction_model PUBLIC cxx_std_17)
target_compile_options(interaction_model PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(interaction_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(draw_follow_path_tool SHARED src/draw_follow_path_tool.cpp
  include/custom_path_navigation/draw_follow_path_tool.hpp)
set_target_properties(draw_follow_path_tool PROPERTIES AUTOMOC ON)
target_compile_features(draw_follow_path_tool PUBLIC cxx_std_17)
target_compile_options(draw_follow_path_tool PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(draw_follow_path_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(draw_follow_path_tool interaction_model Qt5::Widgets
  rviz_ogre_vendor::OgreMain)
ament_target_dependencies(draw_follow_path_tool rviz_common rviz_rendering pluginlib
  rclcpp nav_msgs std_srvs)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

add_library(path_executor SHARED src/path_executor.cpp)
target_compile_features(path_executor PUBLIC cxx_std_17)
target_compile_options(path_executor PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(path_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(path_executor path_processor execution_model)
ament_target_dependencies(path_executor ${dependencies})

add_executable(custom_path_executor src/path_executor_main.cpp)
target_link_libraries(custom_path_executor path_executor)
ament_target_dependencies(custom_path_executor rclcpp)

install(TARGETS path_processor execution_model interaction_model path_executor
  draw_follow_path_tool EXPORT export_path_processor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS custom_path_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_path_processor HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies} rviz_common rviz_rendering rviz_ogre_vendor pluginlib Qt5)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_processor test/test_path_processor.cpp)
  target_compile_features(test_path_processor PRIVATE cxx_std_17)
  target_compile_options(test_path_processor PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math)
  target_link_libraries(test_path_processor path_processor)
  ament_target_dependencies(test_path_processor ${dependencies})
  ament_add_gtest(test_execution_model test/test_execution_model.cpp)
  target_compile_features(test_execution_model PRIVATE cxx_std_17)
  target_include_directories(test_execution_model PRIVATE include)
  target_link_libraries(test_execution_model execution_model)
  ament_add_gtest(test_path_executor test/test_path_executor.cpp TIMEOUT 120)
  target_compile_features(test_path_executor PRIVATE cxx_std_17)
  target_include_directories(test_path_executor PRIVATE include)
  ament_target_dependencies(test_path_executor ${dependencies})
  target_link_libraries(test_path_executor path_executor)
  set_tests_properties(test_path_executor PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=88;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/custom_path_executor_test_logs;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  ament_add_gtest(test_interaction_model test/test_interaction_model.cpp)
  target_compile_features(test_interaction_model PRIVATE cxx_std_17)
  target_include_directories(test_interaction_model PRIVATE include)
  target_link_libraries(test_interaction_model interaction_model)
  ament_add_gtest(test_rviz_plugin test/test_rviz_plugin.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_compile_features(test_rviz_plugin PRIVATE cxx_std_17)
  ament_target_dependencies(test_rviz_plugin rviz_common pluginlib rclcpp std_srvs)
  target_link_libraries(test_rviz_plugin Qt5::Widgets)
  set_tests_properties(test_rviz_plugin PROPERTIES ENVIRONMENT
    "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=89;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/custom_path_rviz_test_logs;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
endif()

ament_package()
