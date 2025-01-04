import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    sim_dir = get_package_share_directory("exr1_sim")
    desc_dir = get_package_share_directory("exr1_description")
    launch_dir = os.path.join(sim_dir, "launch")
    config_dir = os.path.join(sim_dir, "config")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_simulator = LaunchConfiguration("use_simulator")
    world = LaunchConfiguration("world")
    pose = {
        "x": LaunchConfiguration("x_pose", default="-8.00"),
        "y": LaunchConfiguration("y_pose", default="0.00"),
        "z": LaunchConfiguration("z_pose", default="0.1"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    params_file = LaunchConfiguration("params_file")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(desc_dir, "rviz", "config.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start rviz",
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator",
        default_value="True",
        description="Whether to start the simulator",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sim_dir, "worlds", "world.sdf"),
        description="Full path to world model file to load",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="exr1", description="name of the robot"
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(desc_dir, "urdf", "exr1.urdf.xacro"),
        description="Full path to robot sdf file to spawn the robot in gazebo",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(config_dir, "exr1_node.yaml"),
        description="Full path to the ROS2 parameters file to use",
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
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

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "spawn.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_simulator": use_simulator,
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "robot_sdf": robot_sdf,
            "x_pose": pose["x"],
            "y_pose": pose["y"],
            "z_pose": pose["z"],
            "roll": pose["R"],
            "pitch": pose["P"],
            "yaw": pose["Y"],
        }.items(),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_dir, "launch", "robot_description.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(gz_robot)
    ld.add_action(gz_sim)
    ld.add_action(robot_description)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(rviz_cmd)

    return ld
