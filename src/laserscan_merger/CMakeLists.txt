cmake_minimum_required(VERSION 3.8)
project(laserscan_merger)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tinyxml2_vendor REQUIRED)

add_library(laserscan_merger SHARED
  src/laserscan_merger.cpp
)

ament_target_dependencies(laserscan_merger
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  tinyxml2_vendor
)

target_include_directories(laserscan_merger
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

rclcpp_components_register_nodes(laserscan_merger "laserscan_merger::LaserScanMerger")

add_executable(laserscan_merger_node
  src/laserscan_merger_node.cpp
)

target_link_libraries(laserscan_merger_node 
  laserscan_merger
)

install(TARGETS
  laserscan_merger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  laserscan_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
