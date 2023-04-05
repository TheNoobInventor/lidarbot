# Launch file to start the RPLidar A1 sensor

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        name='serial_port',
        default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
        description='Lidar serial port'
    )
    frame_id_arg = DeclareLaunchArgument(
        name='frame_id',
        default_value='lidar_link',
        description='Lidar transform frame'
    )
    angle_compensate_arg = DeclareLaunchArgument(
        name='angle_compensate',
        default_value='True',
        description=''
    )
    scan_mode_arg = DeclareLaunchArgument(
        name='scan_mode',
        default_value='Standard',
        description='Lidar scan mode'
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        angle_compensate_arg,
        scan_mode_arg, 

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'angle_compensate': LaunchConfiguration('angle_compensate'),
                'scan_mode': LaunchConfiguration('scan_mode')
            }]
        )
    ])