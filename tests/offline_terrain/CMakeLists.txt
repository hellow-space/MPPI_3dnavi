cmake_minimum_required(VERSION 3.16)
project(offline_terrain_validation LANGUAGES CXX)

# Standalone build: never installs or changes the navigation packages.
find_package(ament_cmake REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(GTest REQUIRED)

set(critic_root "${CMAKE_CURRENT_SOURCE_DIR}/../../src/height_jump_mppi_critic")
add_executable(validate_terrain validate_terrain.cpp "${critic_root}/src/terrain_map.cpp")
target_compile_features(validate_terrain PRIVATE cxx_std_17)
target_compile_options(validate_terrain PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math)
target_include_directories(validate_terrain PRIVATE "${critic_root}/include")
ament_target_dependencies(validate_terrain grid_map_ros grid_map_core grid_map_msgs rclcpp_lifecycle)

enable_testing()
add_test(NAME offline_terrain_correctness COMMAND validate_terrain)

add_executable(test_terrain_map "${critic_root}/test/test_map.cpp" "${critic_root}/src/terrain_map.cpp")
target_compile_features(test_terrain_map PRIVATE cxx_std_17)
target_compile_options(test_terrain_map PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math)
target_include_directories(test_terrain_map PRIVATE "${critic_root}/include")
target_link_libraries(test_terrain_map GTest::gtest_main)
ament_target_dependencies(test_terrain_map grid_map_ros grid_map_core grid_map_msgs rclcpp_lifecycle)
add_test(NAME terrain_map_regression COMMAND test_terrain_map)
