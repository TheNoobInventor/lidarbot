# Launch file to start the webcam usb driver and aruco trajectory visualizer node
# to track the robot as it moves

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    usb_cam_params = os.path.join(
        get_package_share_directory("lidarbot_aruco"), "config", "params_1.yaml"
    )

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[usb_cam_params],
    )

    aruco_visualizer_node = Node(
        package="lidarbot_aruco",
        executable="aruco_trajectory_visualizer_node",
        output="screen",
    )

    return LaunchDescription(
        [
            usb_cam_node,
            aruco_visualizer_node,
        ]
    )
