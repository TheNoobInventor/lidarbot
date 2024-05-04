# Lidarbot Aruco package

## Generate ArUco marker

The `opencv-contrib-python` module needs to be installed and not `opencv-python`: 

```
pip uninstall opencv-python
pip install opencv-contrib-python
```
The computer might need to be restarted for the install to be effected.

Next navigate to the path in the `lidarbot_aruco` directory:

```
cd ~/dev_ws/lidarbot_aruco/lidarbot_aruco
```

Then run the following script:

```python
python generate_aruco_marker.py --id 24 --type DICT_4X4_50 \
	--output ../tags/DICT_4X4_50_id24.png
```

The script arguments:

`--id` : The unique identifier of the ArUco tag — this is a required argument and ID must be a valid ID in the ArUco dictionary used to generate the tag
    
`--type` : The name of the ArUco dictionary used to generate the tag; the default type is `DICT_4X4_50`

`--output` : The path to the output image where the generated ArUco tag will be saved; this is a required argument

Running the previous script opens a window with the generated ArUco tag displayed,

<p align='center'>
    <img src=../docs/images/generated_aruco_marker.png width="400">
</p>

To close the window, press the **q** key on the keyboard on the opened window.

## Camera calibration

The Logitech webcam C270 HD is used in this project and needs to be calibrated.

First install the [ROS usb camera driver](https://index.ros.org/r/usb_cam/#humble) package:

```bash
sudo apt install ros-humble-usb-cam
```

Camera calibration was done following the steps outlined this [guide](https://automaticaddison.com/how-to-perform-pose-estimation-using-an-aruco-marker/)

Execute the command below to run the usb-cam driver node:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ~/dev_ws/src/lidarbot_aruco/config/params_1.yaml
```

## Aruco trajectory visualizer node

```bash
ros2 run lidarbot_aruco aruco_trajectory_visualizer_node
```

Launch file
```bash
ros2 launch lidarbot_aruco trajectory_visualizer_launch.py
```

TODO: add gif showing node in action

Move this to main README
