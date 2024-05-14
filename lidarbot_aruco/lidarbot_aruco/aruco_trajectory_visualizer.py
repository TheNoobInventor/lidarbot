#!/usr/bin/env python3

# This node is used to visualize the trajectory of lidarbot by tracking the ArUco marker
# on it and drawing lines through the center of the marker in subsequent image frames

# Adapted from: https://automaticaddison.com/estimate-aruco-marker-pose-using-opencv-gazebo-and-ros-2/

# Import Python libraries
import cv2
import numpy as np

# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # Uses Best Effort reliability for camera
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image  # Image is the message type

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
        super().__init__("aruco_visualizer_node")

        # Declare parameters
        self.declare_parameter("aruco_dictionary_name", "DICT_4X4_50")
        self.declare_parameter("aruco_marker_side_length", 0.063)
        self.declare_parameter(
            "camera_calibration_parameters_filename",
            "/home/noobinventor/lidarbot_ws/src/lidarbot_aruco/config/chessboard_calibration.yaml",
        )
        self.declare_parameter("image_topic", "/image_raw")

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

        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            self.get_logger().info(
                "[INFO] ArUCo tag of '{}' is not supported".format(
                    aruco_dictionary_name
                )
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
        self.arucoDict = cv2.aruco.getPredefinedDictionary(
            ARUCO_DICT[aruco_dictionary_name]
        )
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

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # List of points to draw curves through
        self.points = []

    # Callback function
    def listener_callback(self, data):
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            current_frame,
            self.arucoDict,
            parameters=self.arucoParams,
        )

        # Check that at least one ArUco marker was detected
        if marker_ids is not None:

            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_side_length,
                self.camera_matrix,
                self.dist_coeffs,
            )

            # Loop over the detected ArUCo corners
            for markerCorner, markerID in zip(corners, marker_ids):
                # Extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Convert the (x,y) coordinates pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # Draw the bounding box of the ArUCo detection
                cv2.line(current_frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(current_frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(current_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(current_frame, bottomLeft, topLeft, (0, 255, 0), 2)

                # Compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                self.points.append([cX, cY])
                cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)

                # Convert points list to numpy array
                points_array = np.array(self.points)
                points_array = points_array.reshape((-1, 1, 2))

                # Connect center coordinates of detected marker in respective frames
                # NOTE: this section works for only one marker
                cv2.polylines(
                    current_frame, [points_array], False, (0, 255, 0), thickness=5
                )

            # Display image for testing
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    aruco_visualizer_node = ArucoNode()

    # Spin the node so the callback function is called
    rclpy.spin(aruco_visualizer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_visualizer_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
