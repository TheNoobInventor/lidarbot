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

