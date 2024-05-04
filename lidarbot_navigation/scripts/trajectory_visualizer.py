#!/usr/bin/python3

# Node to visualize the robot trajectory in Rviz.
# Adapted from https://github.com/RBinsonB/trajectory_visualization

# WIP

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import (
    Point,
    PoseStamped,
)


class TrajectoryVisualizer(Node):

    def __init__(self, name):
        super().__init__(name)

        # Declare parameters
        self.max_poses_param = self.declare_parameter("max_poses", 1000).value
        self.threshold_param = self.declare_parameter("threshold", 0.001).value
        self.frame_id_param = self.declare_parameter("frame_id", "map").value

        self.trajectory_path_msg = Path()
        self.previous_pose_position = Point()

        # Setup trajectory path publisher
        self.trajectory_path_pub = self.create_publisher(Path, "/trajectory_path", 10)

        # Setup subscriber to odometry
        self.odom_sub = self.create_subscription(
            Path,
            "/transformed_global_plan",
            # Odometry,
            # /odometry/filtered,
            self.odom_callback,
            10,
        )

    # Callback function for odometry type messages
    def odom_callback(self, odom_msg):

        # Process message position and add it to path
        # self.publish_trajectory_path(odom_msg.pose.pose.position)
        self.publish_trajectory_path(odom_msg.poses[0].pose.position)

    # Add pose and publish trajectory path message
    def publish_trajectory_path(self, position):

        # If the pose has moved more than a set threshold, add it ot the path message and publish
        if (abs(self.previous_pose_position.x - position.x) > self.threshold_param) or (
            abs(self.previous_pose_position.y - position.y) > self.threshold_param
        ):

            # Add current pose to path
            self.trajectory_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_path_msg.header.frame_id = self.frame_id_param
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            pose_stamped_msg.pose.position.x = position.x
            pose_stamped_msg.pose.position.y = position.y
            pose_stamped_msg.pose.orientation.w = 1.0

            # If max number of poses in path has not been reach, just add pose to message
            if len(self.trajectory_path_msg.poses) < self.max_poses_param:
                self.trajectory_path_msg.poses.append(pose_stamped_msg)

            # Else rotate the list to dismiss oldest value and add newer value at the end
            else:
                self.get_logger().debug(
                    "Max number of poses reached, erasing oldest pose"
                )
                self.trajectory_path_msg.poses = self.trajectory_path_msg.poses[1:]
                self.trajectory_path_msg.poses.append(pose_stamped_msg)

            self.previous_pose_position = pose_stamped_msg.pose.position
            self.trajectory_path_pub.publish(self.trajectory_path_msg)


def main(args=None):
    rclpy.init(args=args)

    pose_to_path_node = TrajectoryVisualizer("pose_to_path_node")
    rclpy.spin(pose_to_path_node)

    pose_to_path_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
