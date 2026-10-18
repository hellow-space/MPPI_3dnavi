cmake_minimum_required(VERSION 3.16)
project(height_jump_mppi_critic)

find_package(ament_cmake REQUIRED)
set(dependencies
  nav2_mppi_controller nav2_costmap_2d rclcpp rclcpp_lifecycle rcl_interfaces
  pluginlib grid_map_core grid_map_ros grid_map_msgs tf2 tf2_ros)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/terrain_map.cpp src/height_jump_critic.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
# NaN is part of the map contract: never enable finite-math-only assumptions.
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test evaluator map plugin)
    ament_add_gtest(test_${test} test/test_${test}.cpp TIMEOUT 90)
    target_compile_features(test_${test} PRIVATE cxx_std_17)
    target_compile_options(test_${test} PRIVATE -fno-fast-math)
    target_link_libraries(test_${test} ${PROJECT_NAME})
    ament_target_dependencies(test_${test} ${dependencies})
  endforeach()
endif()
ament_package()
