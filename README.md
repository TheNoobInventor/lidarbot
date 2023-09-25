# Lidarbot

![ROS2 CI](https://github.com/TheNoobInventor/lidarbot/actions/workflows/.github/workflows/lidarbot_ci_action.yml/badge.svg)

A differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle is equipped with a raspberry pi camera for visual feedback and an RPLIDAR A1 sensor used for Simultaneous Localization and Mapping (SLAM), autonomous navigation and obstacle avoidance. Additionally, an MPU6050 inertial measurement unit (IMU) is employed by the robot localization package on the robot, to fuse IMU sensor data and the wheel encoders data, using an extended kalman filter (EKF) node, to provide more accurate robot odometry estimates.

Hardware interfaces are written for the Waveshare Motor Driver HAT and MPU6050 sensor to be accessed by the `ros2_control` differential drive controller and Imu sensor broadcaster respectively, via the `ros2_control` resource manager.

<p align='center'>
    <img src=docs/images/real_mapping.gif width="600">
</p>

🚧	***(Work in Progress)*** 

## 🗃️ Package Overview
- [`lidarbot_base`](./lidarbot_base/) : Contains the ROS2 control hardware interface for the lidarbot with low-level code for the Waveshare Motor Driver HAT.
- [`lidarbot_bringup`](./lidarbot_bringup/) : Contains launch files to bring up the camera, lidar and the real lidarbot.
- [`lidarbot_description`](./lidarbot_description/) : Contains the URDF description files for lidarbot, sensors and `ros2 control`.
- [`lidarbot_gazebo`](./lidarbot_gazebo/) : Contains configuration, launch and world files needed to simulate lidarbot in Gazebo.
- [`lidarbot_navigation`](./lidarbot_navigation/) : Contains launch, configuration and map files used for lidarbot navigation.
- [`lidarbot_slam`](./lidarbot_slam/) : Contains configuration files for the slam toolbox and RViz, launch file to generate maps using SLAM.
- [`lidarbot_teleop`](./lidarbot_teleop/) : Contains configuration and launch files used to enable joystick control of the lidarbot in simulation and physically.

## 🧰	Hardware
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
|5| [Silicone rubber bumpers](https://www.aliexpress.com/item/1005002995402961.html) to absorb RPLIDAR vibration|


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

## 🔌	 Installation

### Development Machine setup

A development machine or PC (laptop or desktop) is used to run more computationally intensive applications like Gazebo and Rviz. Additionally, the PC can be used to remotely control lidarbot.  

Ubuntu 22.04 LTS is required for this project to work with ROS2 Humble. Ubuntu 22.04 LTS can be installed on a PC by following [this guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview). The ROS2 Humble installation procedure is available [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). The Desktop version is installed on the PC (which includes RViz):

```
sudo apt install ros-humble-desktop
```

Then install the ROS development tools:

```
sudo apt install ros-dev-tools
```

After the ROS2 Humble installation, create a workspace on the PC/development machine and clone this repository:

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/TheNoobInventor/lidarbot.git .
```

Next install all the [ROS dependencies](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) for the lidarbot packages:

```
cd ~/dev_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Any ROS packages referred to subsequently are assumed to be installed using the `rosdep install` command above unless it is explicitly specified. 

Two more dependencies need to be met before building the workspace: installing the WiringPi i2c library to use the Raspberry Pi 4 GPIO pins and dependencies for the MPU6050 RPi 4 C++ library.

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
```
gpio -v
```

The reference article for the WiringPi library can be found [here](https://arcanesciencelab.wordpress.com/2020/10/29/getting-wiringpi-to-work-under-ubuntu-20-10-on-a-raspberry-pi-4b-w-4gb/).


#### MPU6050 library

Alex Mous' [C/C++ MPU6050 library](https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi
) for Raspberry Pi 4, with modifications to incorporate quaternions, was used to setup the `ros2_control` Imu sensor broadcaster in the [`lidarbot_bringup`](./lidarbot_bringup/) package.

Recall that the MPU6050 module uses the I2C communication protocol, the i2c dependencies for using this library are installed with:

```
sudo apt install libi2c-dev i2c-tools libi2c0
```

#### Sourcing ROS Installation

To avoid manually sourcing the ROS installation (or underlay) in each terminal window opened, and if ROS2 Humble is the only distribution on the PC, the command to source the underlay is added to the respective shell configuration file. 

Using bash:
```
echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
```

Using zsh:
```
echo "source /opt/ros/humble/setup.zsh" >> $HOME/.zshrc
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

---

Finally, navigate to the workspace directory and run the build command:

```
cd ~/dev_ws
colcon build --symlink-install
```

The `--symlink-install` argument uses symlinks instead of copies which saves you from having to rebuild every time you [tweak certain files](https://articulatedrobotics.xyz/ready-for-ros-5-packages/).


#### Gazebo
Gazebo classic, version 11, is the robot simulator used in the project and can be installed [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install). 

#### Display lidarbot model in RViz

The installed [xacro](https://index.ros.org/p/xacro/github-ros-xacro/#humble) tool dependency is used to process the lidarbot URDF files and combine them into a single complete URDF file.

The `description_launch.py` launch file displays the model in RViz:

```
ros2 launch lidarbot_description description_launch.py
```

<p align='center'>
  <img src=docs/images/lidarbot_rviz.png width="800">
</p>

The [joint_state_publisher_gui](https://index.ros.org/p/joint_state_publisher_gui/github-ros-joint_state_publisher/#humble) package is used to bringup a window with sliders to move non-static links in RViz. Set the `use_gui` argument to `true` to turn the left and right wheels of lidarbot:

```
ros2 launch lidarbot_description description_launch.py use_gui:=true
```

<p align='center'>
  <img src=docs/images/joint_state_publisher_gui.png width="800">
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

The [joy_tester](https://index.ros.org/p/joy_tester/github-joshnewans-joy_tester/#humble) package is used to test and map the gamepad (joystick) keys to control lidarbot. To use it, plug in the USB dongle in the PC, then run:

```
ros2 run joy joy_node
``` 

And the following, in a new terminal:
```
ros2 run joy_tester test_joy
```

This opens a GUI window like the one shown below,

<p align='center'>
  <img src=docs/images/joy_tester.png width="600">
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

The `joy_node` parameter, `deadzone`, specifies the amount a joystick has to be moved for it to be [considered to be away from the center](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md). This parameter is normalized between -1 and 1. A value of `0.25` indicates that the joytsick has to be moved 25% of the way to the edge of an axis's range before that axis will output a non-zero value. 

The `deadzone` parameter should be tuned to suit the performance of one's game controller.


#### Twist mux

The [`twist_mux`](https://index.ros.org/p/twist_mux/github-ros-teleop-twist_mux/#humble) package is used to multiplex several velocity command sources, used to move the robot with an unstamped [geometry_msgs::Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) message, into a single one. These sources are assigned priority values to allow a velocity source to be used or disabled. In this project, the command velocity sources are from the joystick and navigation.

The `twist_mux` configuration file is in [`twist_mux.yaml`](./lidarbot_teleop/config/twist_mux.yaml), and is used in the gazebo and lidarbot bringup launch files, [`gazebo_launch.py`](./lidarbot_gazebo/launch/gazebo_launch.py) and [`lidarbot_bringup_launch.py`](./lidarbot_bringup/launch/lidarbot_bringup_launch.py) respectively.

It can be observed from the configuration file, that the joystick commmand velocity source has a higher priority, with an assigned value of `100`, compared to the navigation velocity source that is assigned a value of `10`.

```
twist_mux:
  ros__parameters:
    topics:
      navigation:
        topic   : cmd_vel
        timeout : 0.5
        priority: 10
      joystick:
        topic   : cmd_vel_joy
        timeout : 0.5
        priority: 100
```

#### Robot localization
TODO:

ekf.yaml

```
ros2 launch lidarbot_bringup lidarbot_bringup_launch.py use_robot_localization:=false
```

### Lidarbot setup

To install ROS2 Humble on the Raspberry Pi, Ubuntu Server 22.04 was first flashed on a 32GB micro SD card, this process is detailed in this [guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview).

After inserting the SD card and booting up the Pi, the environment for ROS2 Humble is setup by following this [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Afterwards, ROS-Base (Bare Bones) and ROS development tools are installed:

```
sudo apt install ros-humble-ros-base ros-dev-tools
```

Similarly, after the ROS2 Humble installation, create a workspace on the Raspberry Pi and clone this repository:

```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/TheNoobInventor/lidarbot.git .
```

Install ROS dependencies:

```
cd ~/robot_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y \ 
--skip-keys "rviz2 gazebo_ros2_control gazebo_ros_pkgs" --rosdistro humble
```

`rviz2` and the gazebo related packages are skipped in the ROS dependency installation process as they are only run on the PC and not on the robot --- the rosdep keys for the ROS2 Humble distribution are available [here](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml).

[WiringPi i2c library](#wiringpi) and [MPU6050 RPi 4 C++ library](#mpu6050-library) are also installed before building the workspace --- a `Downloads` directory will need to be created to clone the WiringPi files.

Likewise to avoid manually sourcing the underlay and overlay, the same steps employed in the [development machine setup](#sourcing-ros-installation) are followed but replacing `dev_ws` with `robot_ws` where necessary.

Afterwards, navigate to the workspace directory then run build command:

```
cd ~/robot_ws
colcon build --symlink-install
```

#### Motor Driver HAT

Waveshare's Motor Driver HAT was used to control the motors of lidarbot. The relevant files found in the [`include`](./lidarbot_base/include/lidarbot_base/) and [`src`](./lidarbot_base/src/) directories of the [`lidarbot_base`](./lidarbot_base/) package. These files were modified, from those made [available by Waveshare](https://www.waveshare.com/wiki/Motor_Driver_HAT) with new ones added as well, to find a workaround to determining the motor direction of rotation --- this is a relatively straightforward task if the motors used here had hall effect encoders affixed to them. 

The library dependencies for using the Motor Driver HAT were met after installing the MPU6050 library.

TODO: 

brief on how hall effect sensor work

This could be an upgrade idea for lidarbot.

`ros2_control` hardware interface was written to explain

explain further down how this links with diff_drive_controller

Pull up resistor with relevant links

explain code structure

#### Raspberry Pi Camera

The following packages are installed to use the Raspberry Pi Camera v1.3:
```
sudo apt install libraspberrypi-bin v4l-utils raspi-config
```

Support for the RPi camera v1.3 will need to be enabled by navigating the menu options after running the following command:

```
sudo raspi-config
```
<p align='center'>
  <img src=docs/images/raspi_config_1.png width="400">
  <img src=docs/images/raspi_config_2.png width="400">
</p>

<p align='center'>
  <img src=docs/images/raspi_config_3.png width="400">
  <img src=docs/images/raspi_config_4.png width="400">
</p>

Confirm the RPi camera is connected by running this command:

```
vcgencmd get_camera
```

This should output the following result:
```
supported=1 detected=1, libcamera interfaces=0
```

#### MPU6050 offsets

Prior to using the [Imu sensor broadcaster](https://index.ros.org/p/imu_sensor_broadcaster/github-ros-controls-ros2_controllers/#humble), the MPU6050 module needs to be calibrated to filter out its sensor noise/offsets. This is done in the following steps:

- Place the lidarbot on a flat and level surface and unplug the RPLIDAR. 
- Generate the MPU6050 offsets. A Cpp executable is created in the CMakeLists.txt file of the `lidarbot_bringup` package before generating the MPU6050 offsets. This section of the [CMakeLists.txt](./lidarbot_bringup/CMakeLists.txt) file is shown below:

  ```
  # Create Cpp executable
  add_executable(mpu6050_offsets src/mpu6050_lib.cpp src/mpu6050_offsets.cpp)

  # Install Cpp executables
  install(TARGETS
    mpu6050_offsets
    DESTINATION lib/${PROJECT_NAME}
  )
  ```

  Build the `lidarbot_bringup` package:
  ```
  colcon build --symlin-install --packages-select lidarbot_bringup
  ```

  Run the executable:

  ```
  ros2 run lidarbot_bringup mpu6050_offsets
  ```

  Which outputs something like this:

  ```
  Please keep the MPU6050 module level and still. This could take a few minutes.

  Calculating offsets ...

  Gyroscope offsets:
  ------------------
  X: -104.689
  Y: 651.005
  Z: -158.596

  Accelerometer offsets:
  ----------------------
  X: -13408.8
  Y: 2742.39
  Z: -14648.9

  Include the obtained offsets in the respective macros of the mpu6050_lib.h file.
  ```

- Calibrate the MPU6050 module. Substitute the generated offsets into this section of the [`mpu6050_lib.h`](./lidarbot_bringup/include/lidarbot_bringup/mpu6050_lib.h) file:

  ```
  //Offsets - supply your own here (calculate offsets with getOffsets function)
  //    Gyroscope
  #define G_OFF_X -105
  #define G_OFF_Y 651
  #define G_OFF_Z -159
  //     Accelerometer
  #define A_OFF_X -13409 
  #define A_OFF_Y 2742 
  #define A_OFF_Z -14649
  ```
- Afterwards, the `lidarbot_bringup` package is rebuilt to reflect any changes made to the `mpu6050_lib.h` file.

The MPU6050 module is set to its most sensitive gyroscope and accelerometer ranges, which can be confirmed (or changed) at the top of the `mpu6050_lib.h` file.

## 📡	Network Configuration

Both the development machine and lidarbot need to be connected to the same local network as a precursor to bidirectional communication between the two systems. This [guide](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/) by Robotics Backend was used in configuring the network communication. 

To ensure communication between the dev machine and lidarbot, firstly, the firewall on the development machine had to be disabled (the firewall on the Ubuntu server was disabled by default):

```
sudo ufw disable
```

The next step is to export `ROS_DOMAIN_ID` numbers, between 1 and 232, to the shell configurations of both systems:

```
echo "export ROS_DOMAIN_ID=31" >> ~/.zshrc
```

Then source the shell configuration file:
```
source $HOME/.zshrc
```

Both systems might need to be rebooted to effect these changes.

Additionally, a static IP address was assigned to lidarbot on the router for easy discoverability on the network.

***NOTE:*** 
Best practices might not have been employed in establishing communication between the two systems. The approach proposed [here](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html) was unable to be replicated in this project but others might find better success.

## Differential Drive Controller


## IMU Sensor Broadcaster


## Test Drive

### Gazebo

### Lidarbot

### Motor Connection Checks

TODO:

Fully charge the 18650 batteries, then prop the robot on a box (add image).

Build the base package (if you don't want to build everything):

```
colcon build --packages-select lidarbot_base
```

Then run the following command to run tests:

```
colcon test --ctest-args tests --packages-select lidarbot_base
```

Explain test sequence

negative value is backward, positive is forward. 

[Writing basic cpp tests in ros2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Testing/Cpp.html)

[Colcon testing docs](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Testing/CLI.html)

reference test file

Examine test results:

`colcon test-result --all --verbose`

TODO: command to drive lidarbot around

TODO: ignore camera warning and error messages

## SLAM

### Gazebo

To start mapping in a simulation environment, launch the Gazebo simulation of lidarbot on the development machine (which includes the joystick node for teleoperation):

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
  <img src=docs/images/gazebo_mapping.gif width="800">
</p>

After generating the map, in the **SlamToolboxPlugin** in RViz, type in a name for the map in the field beside the **Save Map** button, then click on it. 

<p align='center'>
    <img src=docs/images/save_map.png width="400">
</p>

The saved map can be found in the workspace directory and will be used by [Nav2 stack](https://navigation.ros.org/) for navigation. 

### Lidarbot

Run the following command on lidarbot to brings up the camera, lidar and joystick:

```
ros2 launch lidarbot_bringup lidarbot_bringup_launch.py
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
