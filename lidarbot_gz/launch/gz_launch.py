# Launches the lidarbot in Gazebo Fortress to be controlled using a joystick.
#
# File adapted from https://automaticaddison.com

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="lidarbot_gz").find("lidarbot_gz")
    pkg_description = FindPackageShare(package="lidarbot_description").find(
        "lidarbot_description"
    )
    pkg_teleop = FindPackageShare(package="lidarbot_teleop").find("lidarbot_teleop")
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")
    pkg_navigation = FindPackageShare(package="lidarbot_navigation").find(
        "lidarbot_navigation"
    )

    bridge_params = os.path.join(pkg_path, "config/lidarbot_bridge.yaml")
    ekf_params_file = os.path.join(pkg_navigation, "config/ekf.yaml")
    urdf_model_path = os.path.join(pkg_path, "urdf/lidarbot_gz.urdf.xacro")
    twist_mux_params_file = os.path.join(pkg_teleop, "config/twist_mux.yaml")
    world_filename = "obstacles.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_path, "models")
    )

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    urdf_path = LaunchConfiguration("urdf_path")
    world = LaunchConfiguration("world")
    use_robot_localization = LaunchConfiguration("use_robot_localization")

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name="use_ros2_control",
        default_value="True",
        description="Use ros2_control if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model to load",
    )

    declare_urdf_path = DeclareLaunchArgument(
        name="urdf_path",
        default_value=urdf_model_path,
        description="Path to the main urdf model",
    )

    declare_use_robot_localization_cmd = DeclareLaunchArgument(
        name="use_robot_localization",
        default_value="True",
        description="Use robot_localization package if true",
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
            "urdf_path": urdf_path,
        }.items(),
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot in Gazebo
    start_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "lidarbot"],
        output="screen",
    )

    # Start Gazebo ROS bridge
    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # Start Gazebo ROS Image bridge
    start_gazebo_ros_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_broadcaster",
        ],
        # 'joint_state_broadcaster'],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_controller",
        ],
        # 'diff_drive_base_controller'],
        output="screen",
    )

    # Spawn diff_controller
    start_diff_controller_cmd = Node(
        condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = Node(
        condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Start robot localization using an Extended Kalman Filter
    start_robot_localization_cmd = Node(
        condition=IfCondition(use_robot_localization),
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_file],
    )

    # Start joystick node for use with ros2_control
    start_joystick_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_teleop, "launch", "joystick_launch.py")]
        )
    )

    # Start twist mux
    start_twist_mux_cmd = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params_file, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_urdf_path)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_robot_localization_cmd)
    ld.add_action(set_env_vars_resources)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    # ld.add_action(start_diff_controller_cmd)
    # ld.add_action(start_joint_broadcaster_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(load_diff_drive_controller)
    ld.add_action(load_joint_state_broadcaster)
    # ld.add_action(start_joystick_cmd)
    # ld.add_action(start_twist_mux_cmd)

    # Bring up rviz2 and mapping/localization stuff too (?)

    return ld
