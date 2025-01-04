import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
        DeclareLaunchArgument("use_sim_time", default_value="true"),
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

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "common.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    gz_sim = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-v4",
            "-r",
            world,
        ],
        name="gz_sim",
        output="screen",
        additional_env={"EXR1_PARAMS_FILE": params_file},
    )

    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "0",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(args + [common, gz_sim, gz_spawn])
