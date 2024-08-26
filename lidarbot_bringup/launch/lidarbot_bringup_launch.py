# This launch file brings up the physical lidarbot, raspberry pi camera v1.3,
# RPLIDAR A1 and also integrates ros2_control, twist_mux, robot_localization
# and joystick control

# File adapted from https://automaticaddison.com

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="lidarbot_bringup").find("lidarbot_bringup")
    pkg_description = FindPackageShare(package="lidarbot_description").find(
        "lidarbot_description"
    )
    pkg_teleop = FindPackageShare(package="lidarbot_teleop").find("lidarbot_teleop")
    pkg_navigation = FindPackageShare(package="lidarbot_navigation").find(
        "lidarbot_navigation"
    )

    controller_params_file = os.path.join(pkg_path, "config/controllers.yaml")
    twist_mux_params_file = os.path.join(pkg_teleop, "config/twist_mux.yaml")
    ekf_params_file = os.path.join(pkg_navigation, "config/ekf.yaml")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_robot_localization = LaunchConfiguration("use_robot_localization")

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name="use_ros2_control",
        default_value="True",
        description="Use ros2_control if true",
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
        }.items(),
    )

    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )

    # Launch controller manager
    start_controller_manager_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_params_file],
    )

    # Spawn diff_controller
    start_diff_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn imu_sensor_broadcaster
    start_imu_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delayed controller manager action
    start_delayed_controller_manager = TimerAction(
        period=2.0, actions=[start_controller_manager_cmd]
    )

    # Delayed diff_drive_spawner action
    start_delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_diff_controller_cmd],
        )
    )

    # Delayed joint_broadcaster_spawner action
    start_delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_joint_broadcaster_cmd],
        )
    )

    # Delayed imu_broadcaster_spawner action
    start_delayed_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_imu_broadcaster_cmd],
        )
    )

    # Start robot localization using an Extended Kalman Filter
    start_robot_localization_cmd = Node(
        condition=IfCondition(use_robot_localization),
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_file],
        remappings=[("/odometry/filtered", "/odom")],
    )

    # Start joystick node
    start_joystick_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_teleop, "launch", "joystick_launch.py")]
        )
    )

    # Start rplidar node
    start_rplidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, "launch", "rplidar_launch.py")]
        )
    )

    # Start camera node
    start_camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, "launch", "camera_launch.py")]
        )
    )

    # Start twist mux
    start_twist_mux_cmd = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params_file],
        remappings=[("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_use_robot_localization_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_delayed_controller_manager)
    ld.add_action(start_delayed_diff_drive_spawner)
    ld.add_action(start_delayed_joint_broadcaster_spawner)
    ld.add_action(start_delayed_imu_broadcaster_spawner)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_joystick_cmd)
    ld.add_action(start_rplidar_cmd)
    ld.add_action(start_camera_cmd)
    ld.add_action(start_twist_mux_cmd)

    return ld
