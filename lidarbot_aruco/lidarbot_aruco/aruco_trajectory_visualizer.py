#!/usr/bin/env python3

# This node is used to visualize the trajectory of lidarbot by tracking the ArUco marker
# on it

# Adapted from: https://automaticaddison.com/estimate-aruco-marker-pose-using-opencv-gazebo-and-ros-2/

# Import Python libraries
import cv2
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R

# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # Uses Best Effort reliability for camera
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped  # Handles TransformStamped message
from sensor_msgs.msg import Image  # Image is the message type
from tf2_ros import TransformBroadcaster

# Construct argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument(
    "-t", "--type", type=str, default="DICT_4X4_50", help="type of ArUco tag to detect"
)

args = vars(ap.parse_args())

# The different ArUco dictionaries built into the OpenCV library
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare parameters
        self.declare_parameter("aruco_dictionary_name", "DICT_4X4_50")
        self.declare_parameter("aruco_marker_side_length", 0.063)
        self.declare_parameter(
            "camera_calibration_parameters_filename",
            "/home/noobinventor/lidarbot_ws/src/lidarbot_aruco/config/chessboard_calibration.yaml",
        )
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("aruco_marker_name", "aruco_marker_frame")

        # Read parameters
        aruco_dictionary_name = (
            self.get_parameter("aruco_dictionary_name")
            .get_parameter_value()
            .string_value
        )
        self.aruco_marker_side_length = (
            self.get_parameter("aruco_marker_side_length")
            .get_parameter_value()
            .double_value
        )
        self.camera_calibration_parameters_filename = (
            self.get_parameter("camera_calibration_parameters_filename")
            .get_parameter_value()
            .string_value
        )
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.aruco_marker_name = (
            self.get_parameter("aruco_marker_name").get_parameter_value().string_value
        )

        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            self.get_logger().info(
                "[INFO] ArUCo tag of '{}' is not supported".format(args["type"])
            )

        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(
            self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ
        )
        self.camera_matrix = cv_file.getNode("camera_matrix").mat()
        self.dist_coeffs = cv_file.getNode("distortion_coefficients").mat()
        cv_file.release()

        # Load the ArUco dictionary
        self.get_logger().info(
            "[INFO] detecting '{}' markers...".format(aruco_dictionary_name)
        )
        self.arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
        self.arucoParams = cv2.aruco.DetectorParameters()

        # Create the subscriber. This subscriber will receive an Image
        # from the image_topic
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.subscription  # prevent unused variable warning

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Callback function
    def listener_callback(self, data):
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            current_frame,
            self.arucoDict,
            parameters=self.arucoParams,
        )

        # Check that at least one ArUco marker was detected
        if marker_ids is not None:

            # Draw a square around detected markers in the video frame
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_side_length,
                self.camera_matrix,
                self.dist_coeffs,
            )

            # The pose of the marker is with respect to the camera lens frame.
            # Imagine you are looking through the camera viewfinder,
            # the camera lens frame's:
            # x-axis points to the right
            # y-axis points straight down towards your toes
            # z-axis points straight ahead away from your eye, out of the camera
            for i, marker_id in enumerate(marker_ids):

                # Create the coordinate transform
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "webcam_frame"
                t.child_frame_id = self.aruco_marker_name

                # Store the translation (i.e. position) information
                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]

                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                # Quaternion format
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                # Send the transform
                self.tfbroadcaster.sendTransform(t)

                # Draw the axes on the marker
                cv2.drawFrameAxes(
                    current_frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    0.05,
                )

                # Draw circle at the center of ArUco tag
                # x_sum = (
                #     corners[0][0][0][0]
                #     + corners[0][0][1][0]
                #     + corners[0][0][2][0]
                #     + corners[0][0][3][0]
                # )
                # y_sum = (
                #     corners[0][0][0][1]
                #     + corners[0][0][1][1]
                #     + corners[0][0][2][1]
                #     + corners[0][0][3][1]
                # )
                #
                # x_centerPixel = x_sum * 0.25
                # y_centerPixel = y_sum * 0.25
                #
                # cv2.circle(
                #     current_frame, (x_centerPixel, y_centerPixel), 4, (0, 0, 255), -1
                # )

            # Display image for testing
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    aruco_node = ArucoNode()

    # Spin the node so the callback function is called
    rclpy.spin(aruco_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
