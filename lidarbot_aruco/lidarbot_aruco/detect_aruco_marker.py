#!/usr/bin/env python

# This detects ArUco markers from a webcam stream using OpenCV and Python

# Adapted from https://automaticaddison.com/how-to-detect-aruco-markers-using-opencv-and-python/
# and https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

import argparse
import cv2
import sys

# Construct argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument(
    "-t", "--type", type=str, default="DICT_4X4_50", help="type of ArUco tag to detect"
)

args = vars(ap.parse_args())

# The different ArUco dictinaries built into OpenCV
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


# Main method
def main():

    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(args["type"], None) is None:
        print("[INFO] ArUco tag of '{}' is not supported".format(args["type"]))
        sys.exit(0)

    # Load the ArUco dictionary
    print("[INFO] detecting '{}' tags...".format(args["type"]))
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters()

    # Start the video stream
    cap = cv2.VideoCapture(0)

    while True:

        # Capture frame-by-frame
        # This method returns True/False as well as the video frame
        ret, frame = cap.read()

        # Detect ArUco markers in the video frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            frame, arucoDict, parameters=arucoParams
        )

        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
            # Flatten the ArUco IDs list of lists to a list
            ids = ids.flatten()

            # Loop over the detected ArUCo corners
            for markerCorner, markerID in zip(corners, ids):
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
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                # Compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                # Draw the ArUco marker ID on the image
                cv2.putText(
                    frame,
                    str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
                print("[INFO] ArUco marker ID: {}".format(markerID))

        # Display the resulting frame
        cv2.imshow("frame", frame)

        # If "q" is pressed on the keyboard,
        # exit this loop
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Close down the video stream
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
