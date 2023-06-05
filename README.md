# Lidarbot

![ROS2 CI](https://github.com/TheNoobInventor/lidarbot/actions/workflows/.github/workflows/lidarbot_ci_action.yml/badge.svg)

A differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle is equipped with a raspberry pi camera for visual feedback and an RPLIDAR A1 sensor used for Simultaneous Localization and Mapping (SLAM), autonomous navigation and obstacle avoidance. Additionally, an MPU6050 inertial measurement unit (IMU) is employed by the robot localization package on the robot, to fuse IMU sensor data and the wheel encoders data, using an extended kalman filter (EKF) node, to provide more accurate robot odometry estimates.

Hardware interfaces are written for the Waveshare motor driver HAT and MPU6050 sensor to be accessed by the `ros2_control` differential drive controller and Imu sensor broadcaster respectively, via the `ros2_control` resource manager.

<p align='center'>
    <img src=docs/images/real_mapping.gif width="600">
</p>

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

The MPU6050 board pins were connected to the Raspberry Pi 4 GPIO pins as follows for use with the I2C communication protocol:

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
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```
colcon build --symlink-install
```

To avoid manually sourcing the ROS installation (or underlay) in each terminal window opened, and if ROS2 Humble is the only distribution on the PC, the command to source the underlay is added to the respective shell configuration file. 

Using bash:
```
echo "source /opt/ros/$ROSDISTRO/setup.bash" >> $HOME/.bashrc
```

Using zsh:
```
echo "source /opt/ros/$ROSDISTRO/setup.zsh" >> $HOME/.zshrc
```

Additionally, to avoid manually sourcing our workspace (or overlay), add the command to source the workspace to the respective configuration file. 

Using bash:
```
echo "source ~/dev_ws/install/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc
```

Using zsh:
```
echo "source ~/dev_ws/install/setup.zsh" >> $HOME/.zshrc
source $HOME/.zshrc
```

The command: `source $HOME/.zshrc` sources the configuration file for use in the current terminal. However, this step is not necessary for terminal windows opened hereafter.

#### Gazebo and `ros2_control`
Gazebo classic, version 11, is the robot simulator used in the project and can be installed [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install). To use the `ros2_control` framework with Gazebo, the following packages are installed:

```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs
```

#### Display lidarbot model in RViz

The following packages are installed to be able to display the model in RViz:

```
sudo apt install ros-humble-joint-state-publisher \
ros-humble-joint-state-publisher-gui ros-humble-xacro
```

The `xacro` tool, is installed to process the lidarbot URDF files and combine them into a single complete URDF file.

The `description_launch.py` launch file displays the model in RViz:

```
ros2 launch lidarbot_description description_launch.py
```

<p align='center'>
  <img src=docs/images/lidarbot_rviz.png width="600">
</p>

The `joint_state_publisher_gui` package is used to bringup a window with sliders to move non-static links in RViz. Set the `use_gui` argument to `true` to rotate the left and right wheels of lidarbot:

```
ros2 launch lidarbot_description description_launch.py use_gui:=true
```

<p align='center'>
  <img src=docs/images/joint_state_publisher_gui.png width="600">
</p>

The different arguments for the launch file, and their default values, can be viewed by adding `--show-args` at the end of launch command:

```
ros2 launch lidarbot_description description_launch.py --show-args
```

#### Teleoperation

A [wireless gamepad](https://www.aliexpress.com/item/1005005354226710.html), like the one shown below, is used to control lidarbot both in simulation and physically. 

<p align='center'>
  <img src=docs/images/wireless_gamepad.jpg width="400">
</p>

To be able to use the gamepad, the following ROS packages are installed:

```
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

To map the keys on the gamepad to control lidarbot, first install the following package:

```
sudo apt install ros-humble-joy-tester
```

This package is used to test the joystick controls. To use it, plug in the USB dongle in the PC, then run:

```
ros2 run joy joy_node
``` 

And the following, in a new terminal:
```
ros2 run joy_tester test_joy
```

This opens a GUI window like the one shown below,

<p align='center'>
  <img src=docs/images/joy_tester.png width="400">
</p>

Click each button and move each stick of the gamepad to confirm that the actions are shown in GUI. The numbers correspond to the axis of the buttons and joystics (sticks) that will be used in mapping the movements of lidarbot. 

The gamepad configuration for this project is in [`joystick.yaml`](./lidarbot_teleop/config/joystick.yaml), where:

| Button/stick | Button/stick axis | Function | 
| :--: | :--: | -- |
| L1 button | 4 | Hold this enable button to move robot at normal speed| 
| Left stick | 2 | Move stick forward or backward for linear motion of the robot | 
| Right stick | 1 | Move stick left or right for angular motion of the robot| 

Setting `require_enable_button` to `true` ensures that L1 has to be held before using the sticks to move the robot and stops the robot once L1 is no longer pressed. 

To enable turbo mode for faster speed, the `enable_turbo_button` option in the config file can be set to an unused button axis.

#### Twist mux

The [`twist_mux`](http://wiki.ros.org/twist_mux) package is used to multiplex several velocity command sources, used to move the robot with an unstamped [geometry_msgs::Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) message, into a single one. These sources are assigned priority values to allow a velocity source to be used or disabled. In this project, the command velocity sources are from the joystick and navigation.

Run the following command to install `twist_mux`:

```
sudo apt install ros-humble-twist-mux
```

The `twist_mux` configuration file is in [`twist_mux.yaml`](./lidarbot_teleop/config/twist_mux.yaml), and is used in the gazebo and lidarbot bringup launch files,[`gazebo_launch.py`](./lidarbot_gazebo/launch/gazebo_launch.py) and [`lidarbot_bringup_launch.py`](./lidarbot_bringup/launch/lidarbot_bringup_launch.py)respectively.

#### Robot localization
TODO

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
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys=rviz2
```

Build the workspace:

```
colcon build --symlink-install
```

Likewise, add run the commands, from the development machine setup section, to source the underlay and overlay in the respective shell configuration file --- replacing `dev_ws` with `robot_ws` where necessary.

The `xacro` tool is also required by lidarbot:

```
sudo apt install ros-humble-xacro
```

#### Teleoperation and Twist mux

The `joy`, `teleop_twist_joy` and `twist_mux` packages are requisite by lidarbot as well:

```
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy ros-humble-twist-mux
```

There is no need to map the gamepad buttons and sticks again.

#### ros2_control
Install the `ros2_control` packages on the robot:

```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers 
```

#### WiringPi

To be able to utilize the GPIO pins of the Raspberry Pi 4 and program them using C/C++, an unofficial WiringPi was installed. This is required as hardware interfaces used by `ros2_control` are currently written only in C++ and low-level communication between Waveshare's Motor Driver HAT and `ros2_control` is needed. 

The library is installed by executing the following commands in a terminal:

```
cd ~/Downloads
git clone https://github.com/wbeebe/WiringPi.git
cd WiringPi/
./build
```
To check the current gpio version run this:
`gpio -v`

The reference article for the WiringPi library is found [here](https://arcanesciencelab.wordpress.com/2020/10/29/getting-wiringpi-to-work-under-ubuntu-20-10-on-a-raspberry-pi-4b-w-4gb/). The library is also installed on the development machine to avoid build errors.

#### RPLIDAR

Install the `rplidar-ros` package to use the RPLIDAR A1 sensor on lidarbot:

```
sudo apt install ros-humble-rplidar-ros
```

#### Raspberry Pi Camera

The following packages are installed to use the Raspberry Pi Camera v1.3:
```
sudo apt install libraspberrypi-bin v4l-utils ros-humble-v4l2-camera
```

Furthermore, [changes to configuration options](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304) are needed to get the RPi camera v1.3 to work. 
In `/boot/config.txt` set the following options:

```
camera_autodetect=0
start_x=1
```
### MPU6050 library

Alex Mous' [C/C++ MPU6050 library](https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi
) for Raspberry Pi 4 was used to setup the `ros2_control` Imu sensor broadcaster.

Recall that the MPU6050 module uses the I2C communication protocol, the i2c dependencies for using this library are installed with:

```
sudo apt install libi2c-dev i2c-tools libi2c0
```

#### Robot localization

TO DO

## Network Configuration

Both the development machine and lidarbot need to be connected to the same local network as a precursor for bidirectional communication between the two systems. This [guide](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/) by Robotics Backend was used in configuring the network communication. 

To ensure communication between the dev machine and lidarbot, the firewall on the development machine had to be disabled (the firewall on the Ubuntu server was diabled by default):

```
sudo ufw disable
```

Also the same `ROS_DOMAIN_ID`, between 1 and 232, was exported to the shell configuration files of both systems:

```
echo "export ROS_DOMAIN_ID=31" >> ~/.zshrc
```

Then source the shell configuration file:
```
source $HOME/.zshrc
```

However, both systems might need to be rebooted to effect these changes.

## Test Drive

### Gazebo

### Lidarbot

## SLAM

### Gazebo

The `slam_toolbox` package is first installed on the development machine:

```
sudo apt install ros-humble-slam-toolbox
```

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

In another terminal, navigate to the workspace directory again and start `rviz2` with the `lidarbot_slam.rviz` config file:

```
rviz2 -d src/lidarbot_slam/rviz/lidarbot_slam.rviz
```

Drive around the obstacles to generate a map of the environment:

<p align='center'>
  <img src=docs/images/gazebo_mapping.gif width="600">
</p>

After generating the map, in the **SlamToolboxPlugin** in RViz, type in a name for the map in the field beside the **Save Map** button, then click on it. 

<p align='center'>
    <img src=docs/images/save_map.png width="400">
</p>

The saved map can be found in the workspace directory and will be used by [Nav2 stack](https://navigation.ros.org/) for navigation. 

### Lidarbot

Run the following command on lidarbot to brings up the camera, lidar and joystick:

```
ros2 launch lidarbot_bringup bringup_launch.py
```

Open a new terminal, on the development machine, navigate to the workspace directory and launch `slam_toolbox` with the `use_sim_time` parameter set to `false`:

```
ros2 launch slam_toolbox online_async_launch.py \ 
slam_params_file:=src/lidarbot_slam/config/mapper_params_online_async.yaml \ 
use_sim_time:=false
```
In a new terminal, also on the development machine, navigate to the workspace directory again and start `rviz2`:

```
rviz2 -d src/lidarbot_slam/rviz/lidarbot_slam.rviz
```

Drive around the environment to generate a map:

<p align='center'>
    <img src=docs/images/real_mapping.gif width="600">
</p>

Then save the generated map.

## Navigation

### Gazebo

### Lidarbot

## Acknowledgment

- [Articulated Robotics](https://articulatedrobotics.xyz/)
- [Automatic Addison](https://automaticaddison.com/)
- [Diffbot](https://github.com/ros-mobile-robots/diffbot)
- [Linorobot2](https://github.com/linorobot/linorobot2)
- [Mini pupper](https://github.com/mangdangroboticsclub/mini_pupper_ros)
- [Robotics Backend](https://roboticsbackend.com/)