import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
]


def generate_launch_description():
    exr1_description_dir = get_package_share_directory("exr1_description")
    xacro_file = os.path.join(exr1_description_dir, "urdf", "exr1.urdf.xacro")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "robot_description": Command(
                    ["xacro", " ", xacro_file, " ", "namespace:=", namespace]
                )
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    return ld
