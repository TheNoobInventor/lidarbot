# TODO: Launch file summary

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # TODO: use lidarbot_teleop package for joystick control
    # TODO: include imu
    # TODO: include lidar 
    # TODO: include camera 
    
    # Set the path to different files and folders
    pkg_share = FindPackageShare(package='lidarbot_bringup').find('lidarbot_bringup')
    pkg_description = FindPackageShare(package='lidarbot_description').find('lidarbot_description')
    controller_params_file = os.path.join(pkg_share, 'config/controllers.yaml')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz/lidarbot_sim.rviz')
    default_urdf_model_path = os.path.join(pkg_description, 'urdf/lidarbot.urdf.xacro')

    # Launch configuration variables specific to simulation
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    # use_joystick = LaunchConfiguration('use_joystick')
    
    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    # declare_joystick_cmd = DeclareLaunchArgument(
    #     name='use_joystick',
    #     default_value='True',
    #     description='Whether to run joystick node')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='True',
        description='Use ros2_control if true')
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'robot_state_publisher_launch.py')]), 
        launch_arguments={'use_sim_time': use_sim_time, 
                          'urdf_model': urdf_model, 
                          'use_ros2_control': use_ros2_control}.items())

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])
    
    # Delayed RViz launch action
    start_delayed_rviz_cmd = TimerAction(period=3.0, actions=[start_rviz_cmd])

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # Launch controller manager
    start_controller_manager_cmd = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description},
                        controller_params_file])

    # Delayed controller manager action   
    start_delayed_controller_manager = TimerAction(period=3.0, actions=[start_controller_manager_cmd])

    # Spawn diff_controller
    start_diff_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_controller'])
    
    # Delayed diff_drive_spawner action
    start_delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_diff_controller_cmd]))

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = Node(
        condition=IfCondition(use_ros2_control),
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broadcaster'])

    # Delayed joint_broadcaster_spawner action
    start_delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_joint_broadcaster_cmd]))
    
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
    
        # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    # ld.add_action(declare_joystick_cmd)
    
    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_delayed_controller_manager)
    ld.add_action(start_delayed_diff_drive_spawner)
    ld.add_action(start_delayed_joint_broadcaster_spawner)
    ld.add_action(start_delayed_rviz_cmd)
    # ld.add_action(start_joy_node_cmd)
    # ld.add_action(start_ros2_joystick_cmd)
    
    return ld