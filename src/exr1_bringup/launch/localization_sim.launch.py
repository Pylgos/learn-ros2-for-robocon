import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("exr1_bringup")
    config_dir = os.path.join(bringup_dir, "config")
    launch_dir = os.path.join(bringup_dir, "launch")
    world_dir = os.path.join(bringup_dir, "world")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(world_dir, "world.sdf"),
            description="Full path to world model file to load",
        ),
        DeclareLaunchArgument(
            "robot_name", default_value="exr1", description="name of the robot"
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(config_dir, "exr1_sim_node.yaml"),
            description="Full path to the ROS2 parameters file to use",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    params_file = LaunchConfiguration("params_file")

    minimal_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "minimal_sim.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "robot_name": robot_name,
            "params_file": params_file,
        }.items(),
    )

    ransac_localization = Node(
        package="ransac_localization",
        executable="ransac_localization",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_frame": "base_link",
                "odom_frame": "odom",
                "merge_window": 0.1,
                "scan_topics": ["scan_left", "scan_right"],
                "min_range": 0.6,
                "line_detection.min_score_ratio": 0.05,
                "line_detection.inlier_threshold": 0.075,
                "line_detection.ransac_iterations": 10000,
                "initial_pose.x": 3.0,
                "initial_pose.y": 1.0,
                "initial_pose.yaw": math.pi / 2,
                "corner_pose.x": 4.0,
                "corner_pose.y": 0.0,
                "corner_pose.yaw": 3 * math.pi / 4,
                "position_lpf": 0.2,
                "rotation_lpf": 0.2,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bringup_dir, "config", "config.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription(args + [minimal_sim, rviz, ransac_localization])
