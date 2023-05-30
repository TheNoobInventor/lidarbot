# Lidarbot

![ROS2 CI](https://github.com/TheNoobInventor/lidarbot/actions/workflows/.github/workflows/lidarbot_ci_action.yml/badge.svg)

A differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle is equipped with a raspberry pi camera for visual feedback and an RPLIDAR A1 sensor used for Simultaneous Localization and Mapping (SLAM), autonomous navigation and obstacle avoidance. Additionally, an MPU6050 inertial measurement unit (IMU) is employed by the robot localization package on the robot, to fuse IMU sensor data and the wheel encoders data, using an extended kalman filter (EKF) node, to provide more accurate robot odometry estimates.

Hardware interfaces are written for the Waveshare motor driver HAT and MPU6050 sensor to be accessed by the `ros2_control` differential drive controller and Imu sensor broadcaster respectively, via the `ros2_control` resource manager.

***(Work in Progress)***

## Package Overview
- [`lidarbot_base`](./lidarbot_base/) : Contains the ROS2 control hardware interface for the lidarbot with low-level code for the Waveshare Motor Driver HAT.
- [`lidarbot_bringup`](./lidarbot_bringup/) : Contains launch files to bring up the camera, lidar and the real lidarbot.
- [`lidarbot_description`](./lidarbot_description/) : Contains the URDF description files for lidarbot, sensors and `ros2 control`.
- [`lidarbot_gazebo`](./lidarbot_gazebo/) : Contains configuration, launch and world files needed to simulate lidarbot in Gazebo.
- [`lidarbot_navigation`](./lidarbot_navigation/) : Contains launch, configuration and map files used for lidarbot navigation.
- [`lidarbot_slam`](./lidarbot_slam/) : Contains configuration files for the slam toolbox and RViz, launch file to generate maps using SLAM.
- [`lidarbot_teleop`](./lidarbot_teleop/) : Contains configuration and launch files used to enable joystick control of the lidarbot in simulation and physically.

## Hardware
### Part list
The following components were used in this project:

| | Part |
| --| --|
|1| [Raspberry Pi 4 (4 GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)|
|2| SanDisk 32 GB SD Card|
|3| [Two wheel drive robot chassis kit (with wheel encoders)](https://www.amazon.com/perseids-Chassis-Encoder-Wheels-Battery/dp/B07DNYQ3PX/ref=sr_1_9?crid=3T8FVRRMPFCIX&keywords=two+wheeled+drive+robot+chassis&qid=1674141374&sprefix=two+wheeled+drive+robot+chas%2Caps%2C397&sr=8-9)|
|4| [Waveshare Motor Driver HAT](https://www.waveshare.com/wiki/Motor_Driver_HAT)|
|5| [2 x Photo interrupters for wheel encoders](https://www.aliexpress.com/item/32773600460.html?spm=a2g0o.order_list.order_list_main.5.21ef1802uhtGk4)|
|6| MPU6050 board|
|7| [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)|
|8| Raspberry Pi camera v1.3|
|9| [3D printed stands for RPLIDAR A1 and RPi 4](https://www.thingiverse.com/thing:3970110)|
|10| Mount for Raspberry Pi camera|
|11| Powerbank for RPi 4 (minimum output: 5V 3A)|
|12| 3 Slot 18650 battery holder|
|13| 3 x 18650 batteries to power Motor Driver HAT|
|14| Female to Female Dupont jumper cables|
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

The MPU6050 board pins were connected to the Raspberry Pi 4 GPIO pins as follows:

| MPU6050 board | GPIO.BOARD| GPIO.BCM|
| ----------- | ------------| ------ |
| VCC         | 3.3V | 3.3V |
| GND         | GND | GND |
| SCL         | 05 | GPIO03 |
| SDA         | 03 | GPIO02 |

The right and left photo interrupter sensors are connected to GPIO pins as follows:

| Photo interrupter (R) | GPIO.BOARD | GPIO.BCM|
| ----------- | ------------| ------ |
| OUT | 18 | GPIO24 |
| VCC | 5V | 5V |
| GND | GND | GND |

| Photo interrupter (L) | GPIO.BOARD | GPIO.BCM|
| ----------- | ------------| ------ |
| OUT | 22 | GPIO25 |
| VCC | 5V | 5V |
| GND | GND | GND |

<p align="center">
  <img title='MPU6050' src=docs/images/mpu6050.jpg width="400">
  <img title='Encoders' src=docs/images/encoders.jpg width="400">
</p>

The screw terminal blocks on the Motor Driver HAT ([shown below](https://www.waveshare.com/wiki/Motor_Driver_HAT)) are connected to the motor wires and battery holder cables as follows: 

| Motor Driver HAT pin | Connected to| 
| -- | -- |
| MA1 | Red wire (Left motor)| 
| MA2 | Black wire (Left motor)| 
| GND | Black wire (battery holder) | 
| VIN | Red wire (battery holder) | 
| MB1 | Red wire(Right motor)| 
| MB2 | Black wire (Right motor)| 

<p align="center">
  <img title='Motor Driver HAT' src=docs/images/Motor_Driver_HAT.png width="400">
</p>

Solder the cables (provided) to the motors. Might need to use spare wires if the provided ones are too short to reach the motor hat. Should the wheel(s) move in the direction opposite of what is expected, exchange the respective motor cables screwed into the terminal blocks.

Finally, the Raspberry Pi camera is connected to the ribbon slot on the Raspberry Pi 4 and the RPLIDAR A1 sensor is plugged into one of the RPi 4's USB ports.

<p align='center'>
  <img title='Top View' src=docs/images/top_view.jpg width="400">
</p>

<p align="center">
  <img title='Side View' src=docs/images/side_view.jpg width="400">
</p>

## Installation

### Development Machine setup

A development machine or PC (laptop or desktop) is used to run more computationally intensive applications like Gazebo and Rviz. Additionally, the PC can be used to control lidarbot remotely. 

Ubuntu 22.04 LTS is required for this project to work with ROS2 Humble. Ubuntu 22.04 LTS can be installed on a PC by following [this guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview). The ROS2 Humble installation installation procedure is avaiable [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). The Desktop version is installed on the PC (which includes RViz):

```
sudo apt install ros-humble-desktop
```

After the ROS2 Humble installation, create a workspace on the PC/development machine and clone this repository:

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws
git clone https://github.com/TheNoobInventor/lidarbot.git
```

Next install ROS dependencies:

```
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```
colcon build --symlink-install
```

Gazebo classic, version 11, is the robot simulator used in the project and can be installed [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install).

### Lidarbot setup

To install ROS2 Humble on the Raspberry Pi, Ubuntu Server 22.04 was first flashed on a 32GB micro SD card, this process is detailed in this [guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview).

After inserting the SD card and booting up the Pi, the base installation of the ROS2 Humble is installed:

```
sudo apt install ros-humble-ros-base
```

Similarly, after the ROS2 Humble installation, create a workspace on the Raspberry Pi and clone this repository:

```
mkdir -p ~/robot_ws/src
cd ~/robot_ws
git clone https://github.com/TheNoobInventor/lidarbot.git
```

Install ROS dependencies:

```
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```
colcon build --symlink-install
```

## Robot localization

## SLAM

| Gazebo | RViz |
| :------: | :----: |
| <img title='Lidarbot Gazebo' src=docs/images/lidarbot_gazebo.png width=400>| <img title=' Lidarbot RViz' src='docs/images/lidarbot_rviz.png' width=400> |


### Gazebo

To start mapping in a simulation environment, launch the Gazebo simulation of lidarbot which includes the joystick node for teleoperation:

```
ros2 launch lidarbot_gazebo gazebo_launch.py
```

In a separate terminal, navigate to the workspace directory, `lidarbot_ws` for example, and launch `slam_toolbox`:

```
ros2 launch slam_toolbox online_async_launch.py \ 
slam_params_file:=src/lidarbot_slam/config/mapper_params_online_async.yaml \ 
use_sim_time:=true
```

In another terminal, navigate to the workspace directory again and start `rviz`:

```
rviz2 -d src/lidarbot_slam/rviz/lidarbot_slam.rviz
```

Drive around the obstacles to get a good map of the environment:

<p align='center'>
    <img src=docs/images/gazebo_mapping.gif width="800">
</p>

Save map

### Lidarbot

The following command brings up lidarbot with camera, lidar and joystick

```
ros2 launch lidarbot_bringup bringup_launch.py
```

Open a new terminal, navigate to the workspace directory and launch `slam_toolbox` with the `use_sim_time` parameter set to `false`:

```
ros2 launch slam_toolbox online_async_launch.py \ 
slam_params_file:=src/lidarbot_slam/config/mapper_params_online_async.yaml \ 
use_sim_time:=false
```

## Navigation

### Gazebo

### Lidarbot

## Acknowledgment

- [Diffbot](https://github.com/ros-mobile-robots/diffbot)
- [Mini_pupper_ros](https://github.com/mangdangroboticsclub/mini_pupper_ros)