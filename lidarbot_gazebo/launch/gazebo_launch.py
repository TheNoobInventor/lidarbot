# Launches the lidarbot in Gazebo to be controlled using a joystick. There are a number of launch arguments that can be toggled.
# Such as using the gazebo_ros package or the ros2_control package, using Gazebo's sim time or not.
#
# File adapted from https://automaticaddison.com

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="lidarbot_gazebo").find("lidarbot_gazebo")
    pkg_description = FindPackageShare(package="lidarbot_description").find(
        "lidarbot_description"
    )
    pkg_teleop = FindPackageShare(package="lidarbot_teleop").find("lidarbot_teleop")
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    pkg_navigation = FindPackageShare(package="lidarbot_navigation").find(
        "lidarbot_navigation"
    )

    gazebo_params_file = os.path.join(pkg_path, "config/gazebo_params.yaml")
    twist_mux_params_file = os.path.join(pkg_teleop, "config/twist_mux.yaml")
    ekf_params_file = os.path.join(pkg_navigation, "config/ekf.yaml")
    world_filename = "obstacles.world"
    world_path = os.path.join(pkg_path, "worlds", world_filename)

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
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

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
        }.items(),
    )

    # Spawn robot in Gazebo
    start_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "lidarbot"],
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
        remappings=[("/odometry/filtered", "/odom")],
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
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_robot_localization_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_diff_controller_cmd)
    ld.add_action(start_joint_broadcaster_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_joystick_cmd)
    ld.add_action(start_twist_mux_cmd)

    return ld
