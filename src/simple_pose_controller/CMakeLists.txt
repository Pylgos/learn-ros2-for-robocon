cmake_minimum_required(VERSION 3.8)
project(simple_pose_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(simple_pose_controller_node SHARED
  src/simple_pose_controller_node.cpp
)

ament_target_dependencies(simple_pose_controller_node
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

rclcpp_components_register_nodes(simple_pose_controller_node "simple_pose_controller::SimplePoseControllerNode")

add_executable(simple_pose_controller
  src/standalone.cpp
)

target_link_libraries(simple_pose_controller
  simple_pose_controller_node
)

install(TARGETS
  simple_pose_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  simple_pose_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
