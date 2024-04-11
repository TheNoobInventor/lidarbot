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

        # Declare parameters
        self.declare_parameter("max_poses", 1000)
        self.declare_parameter("threshold", 0.001)
        self.declare_parameter("frame_id", "map")

        # self.declare_parameter("max_poses", rclpy.Parameter.Type.INTEGER)
        # self.declare_parameter("threshold", rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter("frame_id", rclpy.Parameter.Type.STRING)

        # self.load_params()

        self.max_poses_param = self.get_parameter("max_poses")
        self.threshold_param = self.get_parameter("threshold")
        self.frame_id_param = self.get_parameter("frame_id")

        self.trajectory_path_msg = Path()
        self.previous_pose_position = Point()

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
            # Odometry, "odometry/filtered", self.odom_callback, 10
            Odometry,
            "odom",
            self.odom_callback,
            10,
        )

    # Load parameters
    def load_params(self):
        # Load maximum number of poses in actual path
        self.max_poses = Parameter("max_poses", Parameter.Type.INTEGER, 1000)

        # Load threshold for adding a ose to actual path
        self.threshold = Parameter("threshold", Parameter.Type.DOUBLE, 0.001)

        # Load parent frame id for the trajectory
        self.frame_id = Parameter("frame_id", Parameter.Type.STRING, "map")

    # Pose callback function
    def pose_callback(self, pose_msg):

        self.get_logger().debug(
            f"Received pose message with position {pose_msg.pose.position}"
        )
        # Process message position and add it to path
        self.publish_trajectory_path(pose_msg.position)

    #
    def pose_stamped_callback(self, pose_stamped_msg):
        self.get_logger().debug(
            f"Received pose stamped message with position {pose_stamped_msg.pose.position}"
        )

        # Process message position and add it to path
        if pose_stamped_msg.header.frame_id == self.frame_id:
            self.publish_trajectory_path(pose_stamped_msg.pose.position)
        else:
            self.get_logger().error(
                "PoseStamped message frame: %f does not correspond to trajectory frame %f"
                % (pose_stamped_msg.header.frame_id, self.frame_id),
            )

    #
    def pose_cov_callback(self, pose_cov_msg):
        self.get_logger().debug(
            "Received pose cov message with position {pose_cov_msg.pose.position}"
        )
        self.publish_trajectory_path(pose_cov_msg.pose.position)

    #
    def pose_cov_stamped_callback(self, pose_cov_stamped_msg):
        self.get_logger().debug(
            "Received pose cov stamped message with position {pose_cov_stamped_msg.pose.position}"
        )
        # Process message position and add it to path
        if pose_cov_stamped_msg.header.frame_id == self.frame_id:
            self.publish_trajectory_path(pose_cov_stamped_msg.pose.position)
        else:
            self.get_logger().error(
                "PoseWithCovarianceStamped message frame: %f does not correspond to trajectory frame %f"
                % (pose_cov_stamped_msg.header.frame_id, self.frame_id),
            )

    #
    def odom_callback(self, odom_msg):
        self.get_logger().debug(
            "Received odom message with position {odom_msg.pose.position}"
        )
        # Process message position and add it to path
        if odom_msg.header.frame_id == self.frame_id_param.value:
            self.publish_trajectory_path(odom_msg.pose.position)
        else:
            self.get_logger().error(
                "Odometry message frame: %f does not correspond to trajectory frame %f"
                % (odom_msg.header.frame_id, self.frame_id_param.value),
            )

    #
    def publish_trajectory_path(self, position):
        # If the pose has moved more than a set threshold, add it ot the path message and publish
        if (
            (abs(self.previous_pose_position.x - position.x) > self.threshold)
            or (abs(self.previous_pose_position.y - position.y) > self.threshold)
            or (abs(self.previous_pose_position.z - position.z) > self.threshold)
        ):
            self.get_logger().debug("Exceding threshold, adding pose to path")

            # Add current pose to path
            self.trajectory_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_path_msg.header.frame_id = self.frame_id
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            pose_stamped_msg.pose.position.x = position.x
            pose_stamped_msg.pose.position.y = position.y
            pose_stamped_msg.pose.position.z = position.z
            pose_stamped_msg.pose.orientation.w = 1.0

            # If max number of poses in path has not been reach, just add pose to message
            if len(self.trajectory_path_msg.poses) < self.max_poses_param.value:
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

    pose_to_path = TrajectoryVisualizer("pose_to_path")

    rclpy.spin(pose_to_path)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
