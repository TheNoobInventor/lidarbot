#!/usr/bin/env python

# This script generates ArUco markers using OpenCV and Python

# Adapted from https://automaticaddison.com/how-to-create-an-aruco-marker-using-opencv-python/
# and https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/

import cv2
import argparse
import numpy as np
import sys

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument(
    "-o", "--output", required=True, help="path to output image containing ArUCo tag"
)
ap.add_argument(
    "-i",
    "--id",
    type=int,
    required=True,
    help="ID of ArUCo tag to generate",
)
ap.add_argument(
    "-t",
    "--type",
    type=str,
    default="DICT_4X4_50",
    help="type of ArUCo tag to generate",
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
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])

    # Allocate memory for the output ArUCo tag and then draw the ArUCo
    # tag on the output image
    print(
        "[INFO] generating ArUCo tag type '{}' with ID '{}'".format(
            args["type"], args["id"]
        )
    )
    tag = np.zeros((180, 180, 1), dtype="uint8")
    cv2.aruco.generateImageMarker(arucoDict, args["id"], 180, tag, 1)

    # Write the generated ArUCo tag to disk and then display it to our
    # screen
    cv2.imwrite(args["output"], tag)
    cv2.imshow("ArUCo Tag", tag)

    # If "q" is pressed on the keyboard on the opened window with the aruco tag
    # close the window
    if cv2.waitKey(0) & 0xFF == ord("q"):
        cv2.destroyAllWindows


if __name__ == "__main__":
    main()
