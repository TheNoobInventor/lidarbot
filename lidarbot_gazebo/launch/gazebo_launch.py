# Launches the lidarbot in Gazebo. There are a number of launch arguments that can be toggled. 
# Such as using the gazebo_ros plugin or the ros2_control plugin, using a joystick or not, using Gazebo's sim time
# or not. 
# 
# File adapted from https://automaticaddison.com

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # TODO: use lidarbot_teleop package for joystick control
    # Set the path to different files and folders
    pkg_path= FindPackageShare(package='lidarbot_gazebo').find('lidarbot_gazebo')
    pkg_description = FindPackageShare(package='lidarbot_description').find('lidarbot_description')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    gazebo_params_file = os.path.join(pkg_path, 'config/gazebo_params.yaml')
    world_filename = 'obstacles.world'
    world_path = os.path.join(pkg_path, 'worlds', world_filename)

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    # use_joystick = LaunchConfiguration('use_joystick')
    world = LaunchConfiguration('world')
    
    # Declare the launch arguments  
    # declare_joystick_cmd = DeclareLaunchArgument(
    #     name='use_joystick',
    #     default_value='True',
    #     description='Whether to run joystick node')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='True',
        description='Use ros2_control if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model to load')
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'robot_state_publisher_launch.py')]), 
        launch_arguments={'use_sim_time': use_sim_time, 
                          'use_ros2_control': use_ros2_control}.items())

    # Launch Gazebo 
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items())   

    # Spawn robot in Gazebo
    start_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'lidarbot'])
    
    # Spawn diff_controller
    start_diff_controller_cmd = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=['diff_controller'])

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broadcaster'])

    # # Launch the inbuilt ros2 joy node
    # start_joy_node_cmd = Node(
    #     condition=IfCondition(use_joystick),
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node')
 
    # # Launch inbuilt teleop_twist_joy node with remappings for diff_controller when using ros2_control plugin
    # start_ros2_joystick_cmd =  Node(
    #     condition=IfCondition(
    #                 PythonExpression(["'", use_joystick, "' and '", use_ros2_control, "'"])),
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_node_ros2_control',
    #     remappings=[('/cmd_vel', '/diff_controller/cmd_vel_unstamped')])
    
    # # Launch inbuilt teleop_twist_joy node when using gazebo control plugin (not using ros2_control plugin)
    # start_gazebo_joystick_cmd =  Node(
    #     condition=UnlessCondition(
    #                 PythonExpression(["'", use_joystick, "' and '", use_ros2_control, "'"])),
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_node_gazebo')
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_world_cmd)
    # ld.add_action(declare_joystick_cmd)
    
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_diff_controller_cmd)
    ld.add_action(start_joint_broadcaster_cmd)
    # ld.add_action(start_joy_node_cmd)
    # ld.add_action(start_gazebo_joystick_cmd)
    # ld.add_action(start_ros2_joystick_cmd)
    
    return ld
