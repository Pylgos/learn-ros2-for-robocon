cmake_minimum_required(VERSION 3.8)
project(ransac_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(ransac_localization_node SHARED
  src/ransac_localization_node.cpp
  src/multi_scan_subscription.cpp
  src/corner_localization.cpp
)

ament_target_dependencies(ransac_localization_node
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

rclcpp_components_register_nodes(ransac_localization_node "ransac_localization::RansacLocalization")

add_executable(ransac_localization
  src/standalone.cpp
)

target_link_libraries(ransac_localization
  ransac_localization_node
)

install(TARGETS
  ransac_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ransac_localization
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
