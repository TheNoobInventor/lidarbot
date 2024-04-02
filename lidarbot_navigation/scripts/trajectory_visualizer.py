#!/usr/bin/python3

# Node to visualize the robot trajectory in Rviz.
# Adapted from https://github.com/RBinsonB/trajectory_visualization

import rclpy
from rclpy.node import Node
from rclpy.node import Parameter

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
)


class TrajectoryVisualizer(Node):

    def __init__(self, name):
        super().__init__(name)
        self.load_params()
        self.trajectory_path_msg = Path()
        self.previous_pose_position = Point()

        # Declare parameters
        self.declare_parameter("max_poses", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("threshold", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("frame_id", rclpy.Parameter.Type.STRING)

        # Setup trajectory path publisher
        self.trajectory_path_pub = self.create_publisher(Path, "trajectory_path", 10)

        # Setup subscriber to pose
        self.pose_sub = self.create_subscription(
            Pose, "pose_topic", self.pose_callback, 10
        )

        # Setup subscriber to pose stamped
        self.pose_stamped_sub = self.create_subscription(
            PoseStamped, "pose_stamped_topic", self.pose_stamped_callback, 10
        )

        # Setup subscriber to pose with covariance
        self.pose_cov_sub = self.create_subscription(
            PoseWithCovariance, "pose_cov_topic", self.pose_cov_callback, 10
        )

        # Setup subscriber to pose with covariance stamped
        self.pose_cov_stamped_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "pose_cov_stamped_topic",
            self.pose_cov_stamped_callback,
            10,
        )

        # Setup subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry, "odom_topic", self.odom_callback, 10
        )

    # Load parameters
    def load_params(self):
        # Load maximum number of poses in actual path
        self.max_poses = Parameter("max_poses", Parameter.Type.INTEGER, 1000)

        # Load threshold for adding a ose to actual path
        self.threshold = Parameter("threshold", Parameter.Type.DOUBLE, 0.001)

        # Load parent frame id for the trajectory
        self.frame_id = Parameter("frame_id", Parameter.Type.STRING, 'map')

    # Pose callback function
    def pose_callback(self, pose_msg):

        self.get_logger().debug(
            f"Received pose message with position {pose_msg.pose.position}"
        )
        # Process message position and add it to path
        self.publish_trajectory_path(pose_msg.position)

    def pose_stamped_callback(self, pose_stamped_msg):
        self.get_logger().debug(
            f"Received pose message with position {pose_stamped_msg.pose.position}"
        )
        if pose_stamped_msg.header.frame_id == self.frame_id:
            self.publish_trajectory_path(pose_stamped_msg.pose.position)
        else:
            self.get_logger().error(
                "PoseStamped message frame: {0} does not correspond to trajectory frame {1}",
                format(pose_stamped_msg.header.frame_id, self.frame_id),
            )

    def pose_cov_callback(self, pose_cov_msg):
        self.get_logger().debug(
            "Received pose message with position {pose_msg.position}"
        )

    def pose_cov_stamped_callback(self, pose_cov_stamped_msg):
        self.get_logger().debug(
            "Received pose message with position {pose_msg.position}"
        )

    def odom_callback(self, odom_msg):
        self.get_logger().debug(
            "Received pose message with position {pose_msg.position}"
        )

    def publish_trajectory_path(self, position):
        pass


def main(args=None):
    rclpy.init(args=args)

    pose_to_path = TrajectoryVisualizer("pose_to_path")

    rclpy.spin()
    rclpy.shutdown(pose_to_path)


if __name__ == "__main__":
    main()
