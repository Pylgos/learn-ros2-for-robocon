import os
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

    localization_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "localization_sim.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "robot_name": robot_name,
            "params_file": params_file,
        }.items(),
    )

    simple_pose_controller = Node(
        package="simple_pose_controller",
        executable="simple_pose_controller",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_frame": "base_link",
                "odom_frame": "odom",
                "max_linear_speed": 2.0,
                "max_angular_speed": 3.0,
                "max_linear_deceleration": 1.0,
                "max_angular_deceleration": 1.0,
            }
        ],
    )

    return LaunchDescription(args + [localization_sim, simple_pose_controller])
