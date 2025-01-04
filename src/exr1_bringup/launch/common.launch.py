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
    desc_dir = get_package_share_directory("exr1_description")

    args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation (Gazebo) clock if true",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bringup_dir, "config", "config.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_dir, "launch", "robot_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    field_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "field_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    laserscan_merger = Node(
        package="laserscan_merger",
        executable="laserscan_merger_node",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "publish_frequency": 10.0,
                "out_points": 1000,
                "scan_input_topics": ["scan_left", "scan_right"],
                "scan_timeout": 0.5,
                "out_range_min": 0.5,
            }
        ],
        remappings=[("scan_out", "scan")],
    )

    return LaunchDescription(
        args + [rviz, robot_description, field_description, laserscan_merger]
    )
