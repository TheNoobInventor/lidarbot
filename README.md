# Lidarbot

A differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle is equipped with an RPLIDAR A1 sensor and an Intel RealSense D415 depth camera used for autonomous navigation, obstacle avoidance and Simultaneous Localization and Mapping (SLAM) operations. 

***(Work in Progress)***

<p align='center'>
  <img title='Top View' src=docs/images/top_view.jpg width="400">
</p>

<p align="center">
  <img title='Side View' src=docs/images/side_view.jpg width="400">
</p>

## Package Overview
- [`lidarbot_base`](./lidarbot_base/) :
- [`lidarbot_bringup`](./lidarbot_bringup/) :
- [`lidarbot_description`](./lidarbot_description/) :
- [`lidarbot_gazebo`](./lidarbot_gazebo/) :
- [`lidarbot_teleop`](./lidarbot_teleop/) :

## Hardware
### Part list
The following components were used in this project:

| | Part |
| --| --|
|1| [Raspberry Pi 4 (4 GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)|
|2| SanDisk 64 GB SD Card|
|3| [Two wheel drive robot chassis kit (with wheel encoders)](https://www.amazon.com/perseids-Chassis-Encoder-Wheels-Battery/dp/B07DNYQ3PX/ref=sr_1_9?crid=3T8FVRRMPFCIX&keywords=two+wheeled+drive+robot+chassis&qid=1674141374&sprefix=two+wheeled+drive+robot+chas%2Caps%2C397&sr=8-9)|
|4| [Waveshare Motor Driver HAT](https://www.waveshare.com/wiki/Motor_Driver_HAT)|
|5| [2 x Photo interrupters for wheel encoders](https://www.aliexpress.com/item/32773600460.html?spm=a2g0o.order_list.order_list_main.5.21ef1802uhtGk4)|
|6| MPU6050 board|
|7| [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)|
|8| Intel RealSense D415 depth camera|
|9| [3D printed stands for RPLIDAR A1 and RPi 4](https://www.thingiverse.com/thing:3970110)|
|10| Mount for D415 depth camera|
|11| Powerbank for RPi 4 (minimum output: 5V 3A)|
|12| 3 Slot 18650 battery holder|
|13| 3 x 18650 batteries to power Motor Driver HAT|
|14| Female to Female Dupoint jumper cables|
|15| Spare wires|
|16| On/Off switch (included in robot chassis kit)|

Some other tools or parts used in the project are as follows:

| | Tool/Part |
| --| --|
|1| Soldering iron|
|2| 3D printer|
|3| Screwdriver set|
|4| Double-sided tape|

### Project Wiring and Assembly

The electronic components of the lidarbot are connected as shown below.

<p align="center">
  <img title='Wiring diagram' src=docs/images/lidarbot_wiring.png width="800">
</p>

## Installation

## Simulation

| Gazebo | RViz |
| :------: | :----: |
| <img title='Lidarbot Gazebo' src=docs/images/lidarbot_gazebo.jpg width=400>| <img title=' Lidarbot RViz' src='docs/images/lidarbot_rviz.png' width=400> |