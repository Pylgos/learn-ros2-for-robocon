from math import pi
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
                "initial_pose.yaw": pi / 2,
                "corner_pose.x": 4.0,
                "corner_pose.y": 0.0,
                "corner_pose.yaw": 3 * pi / 4,
                "position_lpf": 0.2,
                "rotation_lpf": 0.2,
            }
        ],
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

    return LaunchDescription(
        args
        + [
            rviz,
            robot_description,
            field_description,
            ransac_localization,
            simple_pose_controller,
        ]
    )
