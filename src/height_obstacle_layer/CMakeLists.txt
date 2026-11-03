cmake_minimum_required(VERSION 3.16)
project(height_obstacle_layer)

find_package(ament_cmake REQUIRED)
set(dependencies
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/height_obstacle_layer.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_ros REQUIRED)
  foreach(test height_difference snapshot_policy)
    ament_add_gtest(test_${test} test/test_${test}.cpp)
    target_compile_features(test_${test} PRIVATE cxx_std_17)
    target_include_directories(test_${test} PRIVATE include)
  endforeach()
  ament_add_gtest(test_sampling test/test_sampling.cpp)
  target_compile_features(test_sampling PRIVATE cxx_std_17)
  target_compile_options(test_sampling PRIVATE -fno-fast-math)
  target_include_directories(test_sampling PRIVATE include)
  ament_target_dependencies(test_sampling grid_map_core)

  ament_add_gtest(test_layer test/test_layer.cpp TIMEOUT 90)
  target_compile_features(test_layer PRIVATE cxx_std_17)
  target_compile_options(test_layer PRIVATE -fno-fast-math)
  target_link_libraries(test_layer ${PROJECT_NAME})
  ament_target_dependencies(test_layer ${dependencies} tf2_ros)
endif()

ament_package()
