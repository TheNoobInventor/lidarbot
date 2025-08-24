# Lidarbot

<!-- ![ROS2 CI](https://github.com/TheNoobInventor/lidarbot/actions/workflows/.github/workflows/lidarbot_ci_action.yml/badge.svg) -->

A differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle is equipped with a Raspberry Pi camera for visual feedback and an RPlidar A1 sensor used for Simultaneous Localization and Mapping (SLAM), autonomous navigation using the Nav2 stack. Additionally, an MPU6050 inertial measurement unit (IMU) is employed by the `robot_localization` package on the robot, to fuse IMU sensor data and the wheel encoders data, using an Extended Kalman Filter (EKF) node, to provide more accurate robot odometry estimates.

Hardware components are written for the Waveshare Motor Driver HAT and MPU6050 sensor to be accessed by the `ros2_control` differential drive controller and IMU sensor broadcaster respectively, via the `ros2_control` resource manager.

<p align='center'>
    <img src=docs/images/real_mapping.gif width="600">
</p>

A preprint of this work is available [here](http://dx.doi.org/10.13140/RG.2.2.15748.54408).

- [Lidarbot](#lidarbot)
  - [🗃️ Package Overview](#️-package-overview)
  - [🧰	Hardware](#hardware)
    - [Part list](#part-list)
    - [Project Wiring and Assembly](#project-wiring-and-assembly)
  - [🔌	 Installation](#-installation)
    - [Development Machine setup](#development-machine-setup)
      - [WiringPi](#wiringpi)
      - [MPU6050 library](#mpu6050-library)
      - [Sourcing ROS Installation](#sourcing-ros-installation)
      - [Gazebo Fortress](#gazebo-fortress)
      - [Display lidarbot model in RViz](#display-lidarbot-model-in-rviz)
      - [Teleoperation](#teleoperation)
      - [Twist mux](#twist-mux)
    - [Lidarbot setup](#lidarbot-setup)
      - [Motor Driver HAT](#motor-driver-hat)
      - [Raspberry Pi Camera](#raspberry-pi-camera)
      - [MPU6050 offsets](#mpu6050-offsets)
  - [Network Configuration](#network-configuration)
  - [ros2 control Framework](#ros2-control-framework)
    - [Hardware components](#hardware-components)
      - [System](#system)
      - [Sensor](#sensor)
      - [Actuator](#actuator)
    - [Resource Manager](#resource-manager)
    - [Controllers](#controllers)
      - [Differential Drive Controller](#differential-drive-controller)
      - [Joint State Broadcaster](#joint-state-broadcaster)
      - [IMU Sensor Broadcaster](#imu-sensor-broadcaster)
    - [Controller Manager](#controller-manager)
  - [Test Drive](#test-drive)
    - [Robot localization](#robot-localization)
    - [Gazebo](#gazebo)
    - [Physical](#physical)
  - [Mapping](#mapping)
    - [Gazebo](#gazebo-1)
    - [Physical](#physical-1)
  - [Navigation](#navigation)
    - [Gazebo](#gazebo-2)
    - [Physical](#physical-2)
  - [ArUco package](#aruco-package)
    - [Generate ArUco marker](#generate-aruco-marker)
    - [Webcam calibration](#webcam-calibration)
    - [Aruco trajectory visualizer node](#aruco-trajectory-visualizer-node)
  - [Acknowledgment](#acknowledgment)


## 🗃️ Package Overview
- [`lidarbot_aruco`](./lidarbot_aruco/) : Contains configuration, launch and node files to use ArUco markers with lidarbot.
- [`lidarbot_base`](./lidarbot_base/) : Contains the ROS2 control hardware component for the lidarbot with low-level code for the Waveshare Motor Driver HAT.
- [`lidarbot_bringup`](./lidarbot_bringup/) : Contains hardware component for the MPU6050 module, launch files to bring up the camera, lidar and the real lidarbot.
- [`lidarbot_description`](./lidarbot_description/) : Contains the URDF description files for lidarbot, sensors and `ros2 control`.
- [`lidarbot_gazebo`](./lidarbot_gazebo/) : Contains configuration, launch and world files needed to simulate lidarbot in Gazebo Classic.
- [`lidarbot_gz`](./lidarbot_gz/) : Contains urdf, launch and world files needed to simulate lidarbot in Gazebo Fortress.
- [`lidarbot_navigation`](./lidarbot_navigation/) : Contains launch, configuration and map files used for lidarbot navigation.
- [`lidarbot_slam`](./lidarbot_slam/) : Contains configuration files for the slam toolbox and RViz, launch file to generate maps using SLAM.
- [`lidarbot_teleop`](./lidarbot_teleop/) : Contains configuration and launch files used to enable joystick control of the lidarbot in simulation and physically.

## 🧰	Hardware
### Part list
The following components were used in this project:

| | Part |
| --| --|
|1| Raspberry Pi 4 (4 GB)|
|2| SanDisk 32 GB SD Card (minimum)|
|3| [Two wheel drive robot chassis kit](https://www.amazon.com/perseids-Chassis-Encoder-Wheels-Battery/dp/B07DNYQ3PX/ref=sr_1_9?crid=3T8FVRRMPFCIX&keywords=two+wheeled+drive+robot+chassis&qid=1674141374&sprefix=two+wheeled+drive+robot+chas%2Caps%2C397&sr=8-9)|
|4| [Waveshare Motor Driver HAT](https://www.waveshare.com/wiki/Motor_Driver_HAT)|
|5| 2 x [Motors with encoders and wire harness](https://s.click.aliexpress.com/e/_DBL19Mr)|
|6| MPU6050 board|
|7| [RPlidar A1](https://s.click.aliexpress.com/e/_DdPdRS7)|
|8| Raspberry Pi camera v1.3|
|9| [3D printed stands for RPlidar A1 and RPi 4](https://www.thingiverse.com/thing:3970110)|
|10| Mount for Raspberry Pi camera|
|11| Powerbank for RPi 4 (minimum output: 5V 3A)|
|12| Gamepad|
|13| [Mini Travel Router (optional)](https://s.click.aliexpress.com/e/_DcgfT61)|
|14| 3 Slot 18650 battery holder|
|15| 3 x 18650 batteries to power Motor Driver HAT|
|16| Female to Female Dupont jumper cables|
|17| Spare wires|
|18| Logitech C270 webcam (optional)|

Some other tools or parts used in the project are as follows:

| | Tool/Part |
| --| --|
|1| Soldering iron|
|2| 3D printer|
|3| Screwdriver set|
|4| Double-sided tape|

Additionally, some nylon stand-offs were used in between the Raspberry Pi 4 and its 3D printed stand to make it easier to plug in the USB power cable from the powerbank.

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

In a previous hardware version of lidarbot, photo interrupters were used with encoder disks with 20 slots (included in the robot chassis kit). However, this setup proved restrictive in yielding satisfactory results in navigation due to the low number of encoder ticks, which was only **20** for a **360 degree** turn of the motor. Therefore, the robot chassis kit motors were replaced with the ones below with built-in encoders --- which have encoder ticks of approximately **1084**, calculated using this [guide](https://automaticaddison.com/calculate-pulses-per-revolution-for-a-dc-motor-with-encoder/) from Automatic Addison.

These are the new motors used for lidarbot:

<p align="center">
  <img title='Motors' src=docs/images/motors.jpg width="400">
  <img title='Motor pins' src=docs/images/motor_pins.jpg width="400">
</p>

The pins on right and left motors are connected to the GPIO pins as follows:

| Right motor pins | GPIO.BOARD | GPIO.BCM|
| ----------- | ------------| ------ |
| C1 (or Encoder A) | 22 | GPIO25 |
| C2 (or Encoder B) | 16 | GPIO23 |
| VCC | 5V | 5V |
| GND | GND | GND |

| Left motor pins| GPIO.BOARD | GPIO.BCM|
| ----------- | ------------| ------ |
| C1 (or Encoder A) | 18 | GPIO24 |
| C2 (or Encoder B) | 15 | GPIO22 |
| VCC | 5V | 5V |
| GND | GND | GND |

Where, 

C1 - counts the respective motor pulses and

C2 - checks the direction of motion of the respective motor, that is, forward or reverse.

<p align="center">
  <img title='MPU6050' src=docs/images/mpu6050.jpg width="400">
  <img title='Encoders' src=docs/images/encoders.jpg width="400">
</p>

The screw terminal blocks on the Motor Driver HAT ([shown below](https://www.waveshare.com/wiki/Motor_Driver_HAT)) are connected to the motor pins, M+ and M-, and battery holder cables as follows: 

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

Solder the cables (provided) to the motors. Spare wires might be needed if the provided ones are too short to reach the Motor Driver Hat. Should the wheel(s) move in the direction opposite of what is expected, exchange the respective motor cables screwed into the terminal blocks.

Finally, the Raspberry Pi camera is connected to the ribbon slot on the Raspberry Pi 4 and the RPlidar A1 sensor is plugged into one of the RPi 4's USB ports.

<p align='center'>
  <img title='Top View' src=docs/images/top_view.jpg width="600">
</p>

<p align="center">
  <img title='Side View' src=docs/images/side_view.jpg width="600">
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
) for Raspberry Pi 4, with modifications to incorporate quaternions, was used to setup the `ros2_control` IMU sensor broadcaster in the [`lidarbot_bringup`](./lidarbot_bringup/) package.

Recall that the MPU6050 module uses the I2C communication protocol, the I2C dependencies for using this library are installed with:

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

#### Gazebo Fortress

*UPDATE: Previously, [Gazebo Classic](https://classic.gazebosim.org/) was used as the simulator for the project but Gazebo Classic has been [deprecated](https://gazebosim.org/docs/latest/gazebo_classic_migration/). The Gazebo Classic lidarbot package is still available here, [`lidarbot_gazebo`](./lidarbot_gazebo/), however, you might encounter installation issues with Classic. It is recommended to [migrate to Gazebo](https://gazebosim.org/docs/latest/gazebo_classic_migration/ ), formerly called "Ignition", and choosing the Fortress release for use with Ubuntu 22.04.*

Gazebo Fortress is used as the simulator for lidarbot. It can be installed with ROS following the steps outlined in this [guide](https://gazebosim.org/docs/fortress/ros_installation/).

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
  <img src=docs/images/joint_state_publisher_gui.gif width="800">
</p>

The different arguments for the launch file, and their default values, can be viewed by adding `--show-args` at the end of launch command:

```
ros2 launch lidarbot_description description_launch.py --show-args
```

#### Teleoperation

A wireless gamepad, like the one shown below, is used to control lidarbot both in simulation and physically. 

<p align='center'>
  <img src=docs/images/wireless_gamepad.jpg width="600">
</p>

The [joy_tester](https://index.ros.org/p/joy_tester/#humble) package is used to test and map the gamepad (joystick) keys to control lidarbot. To use it, plug in the USB dongle in the PC, then run:

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

The `joy_node` parameter, `deadzone`, specifies the amount a joystick has to be moved for it to be [considered to be away from the center](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md). This parameter is normalized between `-1` and `1`. A value of `0.25` indicates that the joytsick has to be moved `25%` of the way to the edge of an axis's range before that axis will output a non-zero value. 

The `deadzone` parameter should be tuned to suit the performance of user's game controller.

#### Twist mux

The [`twist_mux`](https://index.ros.org/p/twist_mux/github-ros-teleop-twist_mux/#humble) package is used to multiplex several velocity command sources, used to move the robot with an unstamped [geometry_msgs::Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) message, into a single one. These sources are assigned priority values to allow a velocity source to be used or disabled. In this project, the command velocity sources are from the joystick and navigation.

The `twist_mux` configuration file is in [`twist_mux.yaml`](./lidarbot_teleop/config/twist_mux.yaml), and is used in the Gazebo and lidarbot bringup launch files, [`gz_launch.py`](./lidarbot_gz/launch/gz_launch.py) and [`lidarbot_bringup_launch.py`](./lidarbot_bringup/launch/lidarbot_bringup_launch.py) respectively.

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

`rviz2` and the Gazebo related packages are skipped in the ROS dependency installation process as they are only run on the PC and not on the robot --- the `rosdep` keys for the ROS2 Humble distribution are available [here](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml).

[WiringPi i2c library](#wiringpi) and [MPU6050 RPi 4 C++ library](#mpu6050-library) are also installed before building the workspace --- a `Downloads` directory will need to be created to clone the WiringPi files.

Likewise to avoid manually sourcing the underlay and overlay, the same steps employed in the [development machine setup](#sourcing-ros-installation) are followed but replacing `dev_ws` with `robot_ws` where necessary.

Afterwards, navigate to the workspace directory then run the build command:

```
cd ~/robot_ws
colcon build --symlink-install
```

#### Motor Driver HAT

Waveshare's Motor Driver HAT was used to control the motors of lidarbot. The relevant files are found in the [`include`](./lidarbot_base/include/lidarbot_base/) and [`src`](./lidarbot_base/src/) directories of the [`lidarbot_base`](./lidarbot_base/) package, the file tree of these directories are as shown, 

```
├── CMakeLists.txt
├── include
│   └── lidarbot_base
│       ├── Debug.h
│       ├── DEV_Config.h
│       ├── lidarbot_hardware.hpp
│       ├── MotorDriver.h
│       ├── motor_encoder.h
│       ├── PCA9685.h
│       └── wheel.hpp
├── lidarbot_hardware.xml
├── package.xml
└── src
    ├── DEV_Config.c
    ├── lidarbot_hardware.cpp
    ├── motor_checks_client.cpp
    ├── motor_checks_server.cpp
    ├── MotorDriver.c
    ├── motor_encoder.c
    ├── PCA9685.c
    └── wheel.cpp
```

The following table summarizes the package files specifying those that were made [available by Waveshare](https://www.waveshare.com/wiki/Motor_Driver_HAT), the newly added ones and the functions they serve.

| File(s) | New | Modified | Function |
| ----------- | ------------| ------ | ------ |
| `Debug.h` | No | No | Output debug information with `printf`. |
| `DEV_Config.h`, `DEV_Config.c` | No | Yes | Configure underlying device interfaces and protocols. |
| `lidarbot_hardware.hpp`, `lidarbot_hardware.cpp` | Yes | No | Configure the WaveShare Motor Driver HAT system hardware component. It reads the motor encoder values to calculate the wheel positions and velocities. It also sends commands to the drive the motors of each wheel proportional to the velocity commands received from the `ros2_control` differential driver controller. |
| `motor_checks_client.cpp` | Yes | No | Request motor tests to confirm motor connections and the motor operation. |
| `motor_checks_server.cpp` | Yes | No | Run requested motor tests. |
| `MotorDriver.h`, `MotorDriver.c` | No | Yes | Program the TB6612FNG motor Integrated Circuit (IC) to drive the motors. |
| `motor_encoder.h`, `motor_encoder.c` | Yes | No | Calculate encoder ticks and set the speed each motor should run at according to the commands received from the hardware component. |
| `PCA9685.h`, `PCA9685.c` | No | No | Program the PCA9685 chip to adjust the motor speed using Pulse Width Modulation (PWM). |
| `wheel.hpp`, `wheel.cpp` | Yes | No | Configure motor wheel parameters. |

The hierarchy of motor control is shown in the image below.

<p align='center'>
  <img src=docs/images/motor_control_hierarchy.jpg width="600">
</p>

The main reasons for modifying the Waveshare files was to enable the use of the WiringPi library and also to find a workaround to determining the motor direction of rotation --- this was made relatively easier since the motor encoders used here have hall effect sensors affixed to them as shown below. 

<p align='center'>
  <img src=docs/images/hall_effect_sensors.png width="800">
</p>

Hall effect sensors in motor encoders detect changes in the magnetic field associated with the motor's rotation. These changes can be used by the motor encoder to determine the position of the motor. [As the motor rotates](https://automaticaddison.com/calculate-pulses-per-revolution-for-a-dc-motor-with-encoder/), it generates alternating electrical signals of high and low voltage, and each time the signal goes from from low high (that is, it is rising), that counts as a single ***pulse***. The image below, from this [guide](https://automaticaddison.com/calculate-pulses-per-revolution-for-a-dc-motor-with-encoder/) from Automatic Addison, visualizes this.

<p align='center'>
  <img src=docs/images/time_interval.jpg width="600">
</p>

The encoder pulse, sometimes referred to as a ***tick***, for a 360 degree turn of the motor is used as the wheel odometry data of the robot. This data will be used with the MPU6050 IMU sensor data using an Extended Kalman Filter with the [robot_localization](#robot-localization) package to provide more accurate robot odometry estimates. Recall that the encoder ticks for the motors used here is **1084**, for a 360 degree turn of the motor.

The library dependencies for using the Motor Driver HAT were met after installing the MPU6050 library.

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

Prior to using the [IMU sensor broadcaster](#imu-sensor-broadcaster), the MPU6050 module needs to be calibrated to filter out its sensor noise/offsets. This is done in the following steps:

- Place lidarbot on a flat and level surface and unplug the RPlidar. 
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
  colcon build --symlink-install --packages-select lidarbot_bringup
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

## Network Configuration

Both the development machine and lidarbot need to be connected to the same local network as a precursor to bidirectional communication between the two systems. This [guide](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/) by Robotics Backend was used in configuring the network communication. 

To ensure communication between the dev machine and lidarbot, firstly, the firewall on the development machine had to be disabled (the firewall on the Ubuntu server was disabled by default):

```
sudo ufw disable
```

The next step is to export `ROS_DOMAIN_ID` number, a number between 0 (by default) and 101, to the shell configurations of both systems:

```
echo "export ROS_DOMAIN_ID=31" >> ~/.zshrc
```

This step is necessary particularly if there are other robots and development machines on the same local network. This [YouTube video](https://www.youtube.com/watch?v=qSjNQC2AT_4) and [article](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) go into more detail about `ROS_DOMAIN_ID`.

Then source the shell configuration file:
```
source $HOME/.zshrc
```

Both systems might need to be rebooted to effect these changes.

A static IP address was assigned to lidarbot on the router for easy discoverability on the network. Furthermore, it is advisable to use a router that supports at least the WiFi 5 wireless standard to avoid excessive data lag on RViz and terminal crashes when recording a [ros2bag](https://index.ros.org/r/rosbag2/github-ros2-rosbag2/#humble) for instance; in relation to ros2bags, changing the default data storage format to [MCAP](https://foxglove.dev/blog/announcing-the-mcap-storage-plugin-for-ros2) should [improve read and write performance](https://adrian-website.mcap.pages.dev/guides/benchmarks/rosbag2-storage-plugins). 

A dedicated network travel router, following this setup by [Articulated Robotics](https://articulatedrobotics.xyz/tutorials/ready-for-ros/networking#dedicated-network) was used in this project to establish a reliable connection between the development machine and lidarbot.

Additional network troubleshooting information can be found [here](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html).

## ros2 control Framework

To control the motion of lidarbot both in simulation and physically, the `ros2_control` framework was used. It is defined as a [hardware-agnostic control framework](https://control.ros.org/master/doc/resources/resources.html#roscon-fr-2022) focusing on the modularity of control systems for robots, sharing of controllers and real-time performance. It provides hardware abstraction and low-level management utility, for instance, hardware lifecycle, communication and access control, which is useful for other frameworks like the manipulation path planning and autonomous navigation frameworks, [MoveIt2](https://moveit.picknik.ai/main/index.html) and [Nav2](https://ieeexplore.ieee.org/iel7/9340668/9340635/09341207.pdf) respectively.

The `ros2_control` framework can be thought of as a [middleman](https://automaticaddison.com/how-to-control-a-robotic-arm-using-ros-2-control-and-gazebo/) between the robot hardware and the robot software and is comprised of the following parts:

1. Hardware Components
2. Resource Manager
3. Controllers
4. Controller Manager

### Hardware components

Hardware components are drivers for physical hardware and represent its abstraction in the `ros2_control` framework. These components are usually provided by robot or hardware manufacturers, otherwise the components would either have to be sourced from the ROS community or written from scratch using this [guide](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html) made available by the `ros2_control` development team. Hardware components are written in `C++` and exported as plugins using the [`pluginlib`](https://index.ros.org/p/pluginlib/#humble)-library; these plugins are dynamically loaded at runtime by the [Resource Manager](#resource-manager).

Hardware components are configured in the robot's Unified Robot Description Format (URDF) file --- an XML file that describes the model of the robot --- using the `<ros2_control>` [tag](https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-description-in-urdf) and the recommended `xacro` (XML) macro files.

There are 3 basic types of components:

#### System

This refers to a hardware setup made up of more than 1 Degree of Freedom (DOF) which contains multiple joints and sensors. This component type offers a lot of flexibility in handling hardware setups which can range from mobile robots to humanoid robot hands and more. The Waveshare Motor Driver HAT controls the two motors (or two wheel joints) on lidarbot, hence the robot base is classifed as a system component.
  
A system component has reading and writing capabilities, these are expressed as state interfaces and command interfaces respectively. The system component configuration for the robot base with a velocity command interface and state interfaces for both the velocity and position is shown in the code snippet below, available in this [xacro file](./lidarbot_description/urdf/ros2_control.xacro).

```
<ros2_control name='RealRobot' type='system'>
  <hardware>
    <plugin>lidarbot_base/LidarbotHardware</plugin>
      <param name='left_wheel_name'>left_wheel_joint</param>
      <param name='right_wheel_name'>right_wheel_joint</param>
      <param name='enc_ticks_per_rev'>1084</param>
      <param name='loop_rate'>30.0</param>
  </hardware>
  <joint name='left_wheel_joint'>
    <command_interface name='velocity'>
      <param name='min'>-10</param>
      <param name='max'>10</param>
    </command_interface>
    <state_interface name='velocity'/>
    <state_interface name='position'/>
  </joint>
  <joint name='right_wheel_joint'>
    <command_interface name='velocity'>
      <param name='min'>-10</param>
      <param name='max'>10</param>
    </command_interface>
    <state_interface name='velocity'/>
    <state_interface name='position'/>
  </joint>
</ros2_control>
```

A similar system hardware component is configured for Gazebo Fortress in this [URDF](./lidarbot_gz/urdf/lidarbot_gz.urdf.xacro) file.

#### Sensor

A sensor provides information about changes to a physical phenomenon it is observing. A sensor component has only reading capabilities and is related to a joint or a link in the URDF file. This component type can be configured as an externally connected sensor, with its own `<ros2_control>` tags, or as an integrated sensor embedded in system component `<ros2_control>` tags. The MPU6050 module is configured as an externally connected sensor component and is associated with a fixed link named `imu_link`. This configuration is shown in the code snippet below which is available in the MPU6050 section of this [xacro file](./lidarbot_description/urdf/ros2_control.xacro).

```
<ros2_control name='MPU6050' type='sensor'>
  <hardware>
    <plugin>lidarbot_bringup/MPU6050Hardware</plugin>
      <param name='sensor_name'>mpu6050</param>
      <param name='frame_id'>imu_link</param>
  </hardware>
  <sensor name='mpu6050'>
    <state_interface name='orientation.x'/>
    <state_interface name='orientation.y'/>
    <state_interface name='orientation.z'/>
    <state_interface name='orientation.w'/>
    <state_interface name='angular_velocity.x'/>
    <state_interface name='angular_velocity.y'/>
    <state_interface name='angular_velocity.z'/>
    <state_interface name='linear_acceleration.x'/>
    <state_interface name='linear_acceleration.y'/>
    <state_interface name='linear_acceleration.z'/>  
  </sensor>
</ros2_control>
```

However, a standalone [IMU sensor plugin](https://gazebosim.org/docs/fortress/sensors/#imu-sensor) was used to simulate an [IMU sensor](./lidarbot_gz/urdf/sensors.xacro) in Gazebo Fortress.

#### Actuator

This component type only has 1 DOF. Some examples of these include motors and valves. Actuator components are related to a single joint and have both reading and writing capabilities, however, obtaining state feedback is not mandatory if is inaccessible; for instance, controlling a DC motor that has no encoders.

The image below (adapted from [Denis Štogl](https://vimeo.com/649651707) (CC-BY)) shows the system and sensor components used for the lidarbot base and the MPU6050 module respectively — custom hardware components were written for the [Waveshare Motor Driver HAT](./lidarbot_base/src/lidarbot_hardware.cpp) and the [MPU6050 module](./lidarbot_bringup/src/mpu6050_hardware_interface.cpp) using the [MPU6050 library](#mpu6050-library).

<p align="center">
  <img title='hardware components' src=docs/images/hardwarecomp.png width="400">
</p>

The coloured icons represent the state and command interfaces of the hardware components. The state interfaces for the respective cartesian axes, associated with the MPU6050 module, were merged into one for clarity.

### Resource Manager

The Resource Manager (RM) abstracts the hardware components for the `ros2_control` framework. The RM is responsible for loading the components using the `pluginlib`-library, managing their lifecycle and components’ state and command interfaces. In the control loop execution, which is managed by the Controller Manager, the RM’s `read()` and `write()` methods handle the communication with the hardware components. 

The abstraction provided by RM allows reuse of implemented hardware components. A sensor component, like the MPU6050 module, has only reading capabilities, thus it can be shared by multiple controllers simultaneously. However, components with command interfaces require exclusive access by only one controller. 

The bidirectional arrows in the figure below (adapted from [Denis Štogl](https://vimeo.com/649651707) (CC-BY)) indicate interfaces that have both read and write capabilities, but interfaces that only provide state feedback are represented with unidirectional arrows.

<p align="center">
  <img title='resource manager' src=docs/images/resman.png width="400">
</p>

### Controllers

The controllers in the `ros2_control` framework are based on control theory which compare the reference value with the measured output and, based on this error, calculate a system’s input. Controllers access the latest hardware lifecycle state when the Controller Manager’s `update()` method is called and enables them to write to the hardware command interfaces.

There are a number of controllers available for specific hardware configurations, such as ***Ackermann steering controller***, ***bicycle steering controller***, ***gripper controller*** and ***differential drive controller*** (which is used on lidarbot). The full list of controllers is available on the `ros2_controllers` [page](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html). Similar to hardware components, a [guide](https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html) is also provided on how to write a custom controller if a particular hardware configuration is missing.

#### Differential Drive Controller

The differential drive controller receives `Twist.msg` velocity command messages from the Nav2 stack which are written to the velocity command interfaces of the robot base system hardware component. This in turn drives the motors in accordance with the received velocity command interfaces. This sequence of commands is illustrated in the next image.

#### Joint State Broadcaster

Broadcasters only have reading abilities and are used to publish sensor data from hardware components to ROS topics. The joint state broadcaster reads all state interfaces and reports them on the `/joint_states` and `/dynamic_joint_state` topics. These topics are used by RViz to display the current state of the robot joints.

#### IMU Sensor Broadcaster

The IMU sensor broadcaster on lidarbot publishes `Imu.msg` messages on the topic, `imu_broadcaster/imu`, from the MPU6050 IMU hardware sensor component. The IMU sensor data is utilized by the [`robot_localization`](#robot-localization) package to fuse wheel odometry data with an Extended Kalman Filter (EKF). This is used to improve the robot’s localization for autonomous navigation with the Nav2 framework. The topic connections between Nav2, relevant packages and the lidarbot controllers are presented in the [navigation](#navigation) subsection.

The lidarbot differential drive controller and IMU sensor broadcaster are configured in the [`controllers.yaml`](./lidarbot_bringup/config/controllers.yaml) file in the bringup package; the joint state broadcaster here uses the default parameters so no configuration is needed.

In configuring the IMU sensor broadcaster, the static orientation, velocity and linear acceleration covariances, which have a row major arrangement about the x, y and z axes, need to be calculated. A [covariance matrix](https://www.cuemath.com/algebra/covariance-matrix/), sometimes referred to as a variance-covariance matrix, is a square matrix where the diagonal elements represent the variance and the off-diagonal elements represent the covariance.

Visualizing the static velocity covariance, for instance, with the matrix below, the diagonal elements represent the variance of a chosen sample data size of velocity measurements for the respective axes. While the off-diagonal elements represent the covariance between velocities of two axes over a chosen sample data size.

       ┌──────────┬──────────┬──────────┐
       │ Var(x,x) │ Cov(x,y) │ Cov(x,z) │
       ├──────────┼──────────┼──────────┤
       │ Cov(y,x) │ Var(y,y) │ Cov(y,z) │
       ├──────────┼──────────┼──────────┤
       │ Cov(z,x) │ Cov(z,y) │ Var(z,z) │
       └──────────┴──────────┴──────────┘

The matrix above is represented as a 9 element array, in the IMU Sensor Broadcaster configuration of the [`controllers.yaml`](./lidarbot_bringup/config/controllers.yaml) file for the respective covariances to be calculated.

However, only the variance array elements are calculated. It is assumed that the different axes do not vary together, therefore the off-diagonal elements/covariances are zero. The [`mpu6050_covariances.cpp`](./lidarbot_bringup/src/mpu6050_covariances.cpp) file can be modified to incorporate the covariance calculations later on.

The variance was calculated with a sample data size of **500** points, with a delay of **0.25s** to read non-consecutive values from the MPU6050 module.

Keep the module still then run this node to generate the variances:

```
ros2 run lidarbot_bringup mpu6050_covariances
```

An output similar to the following will be shown in the terminal window

```
Please keep the MPU6050 module level and still.
Reading and appending 500 sensor data points to respective arrays, it may take a while ...

Calculating variances ...

static_covariance_orientation: [5.46409e-06, 0.0, 0.0, 5.72254e-06, 0.0, 0.0, 4.22433e-07, 0.0, 0.0]
static_covariance_angular_velocity: [2.01015e-06, 0.0, 0.0, 1.9657e-06, 0.0, 0.0, 8.13776e-07, 0.0, 0.0]
static_covariance_linear_acceleration: [0.000632951, 0.0, 0.0, 0.000801987, 0.0, 0.0, 0.00117363, 0.0, 0.0]

Paste covariance arrays in the imu_broadcaster ros__parameters section in lidarbot_bringup/config/controllers.yaml.

```

These arrays are then pasted in the `imu_broadcaster ros__parameters` section of the [`controllers.yaml`](./lidarbot_bringup/config/controllers.yaml) file to complete the IMU Sensor Broadcaster configuration.

### Controller Manager

The Controler Manager (CM) is responsible for managing the lifecycle of controllers (for  instance, loading, activating, deactivating and unloading respective controllers) and the interfaces they require. Through the RM, the CM has access to the hardware component interfaces. The Controller Manager connects the controllers and hardware-abstraction sides of the `ros2_control` framework, and also serve as the entry-point for users via ROS services using the [`ros2 cli`](https://control.ros.org/master/doc/ros2_control/ros2controlcli/doc/userdoc.html).

The Controller Manager matches required and provided interfaces, grants contollers access to hardware when enabled, or reports an error if there is an access conflict. The execution of the control-loop is managed by the CM's `update()` method which reads data from the hardward components, updates the outputs of all active controllers, and writes the result to the components.

In essence, the CM manages the controllers, which calculate the commands needed to make the [robot move as desired](https://automaticaddison.com/how-to-control-a-robotic-arm-using-ros-2-control-and-gazebo/).

The following image (adapted from [Denis Štogl](https://vimeo.com/649651707) (CC-BY)) illustrates all the parts of the `ros2_control` framework modified for use on lidarbot.

<p align="center">
  <img title='ros2 control architecture' src=docs/images/ros2_control_arch.png width="800">
</p>

The `ros2_control` framework can be explored in more detail by reading the [official documentation](https://control.ros.org/master/doc/getting_started/getting_started.html), this [article](https://masum919.github.io/ros2_control_explained/) from Masum, this [tutorial](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts/) from Articulated Robotics and the [preprint](http://dx.doi.org/10.13140/RG.2.2.15748.54408) for this project.

## Test Drive

### Robot localization

Before setting up lidarbot for autonomous navigation with the Nav2 stack, its wheel encoder and IMU sensor data are fused. Slipping between the wheels and the ground is [common in differential drive robots](https://www.sciencedirect.com/science/article/pii/S1474667016428909), and can lead to substantial deviation from the actual position of a wheeled mobile robot from its desired position. Fusing different sensor data aids in correcting for this variation. An Extended Kalman Filter is employed by the [`robot_localization`](https://index.ros.org/p/robot_localization/#humble) package to fuse the wheel odometry and IMU sensor data to obtain more accurate robot odometry estimates. 

This results in improved robot navigation as the Nav2 stack makes use of odometry data to estimate the pose (position and orientation) of the robot. The wheel odometry and IMU sensor data are obtained from the differential drive controller topic, `diff_controller/odom`, and the IMU sensor broadcaster topic, `imu_broadcaster/imu` respectively. 

The EKF is configured using this [guide](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html) from Nav2; more information about the `robot_localization` package can be found in the [official documenation](https://docs.ros.org/en/noetic/api/robot_localization/html/configuring_robot_localization.html). The EKF configuration file, `ekf.yaml`, is available [here](./lidarbot_navigation/config/ekf.yaml).

By default the Gazebo and real world lidarbot bringup launch files have the `use_robot_localization` argument set to `true` but this can also be set to `false` if one chooses to do so.

```
ros2 launch lidarbot_bringup lidarbot_bringup_launch.py use_robot_localization:=false
```
### Gazebo

To drive lidarbot around in Gazebo Fortress, first ensure that the USB dongle of the gamepad is plugged in to the development machine then run this command to bringup lidarbot, sensors and also integrates `ros2_control`, `twist_mux`, `robot_localization` and gamepad/joystick control:

```
ros2 launch lidarbot_gz gz_launch.py
```

The GIF below shows lidarbot being driven around by the gamepad (with the gamepad configuration presented [here](#teleoperation)) in a [world made up of obstacles](https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-gazebo#making-an-obstacle-course). 

<p align='center'>
    <img src=docs/images/gazebo_test_drive.gif width="600">
</p>

### Physical

A [ROS service](https://foxglove.dev/blog/creating-ros2-services) was written to test the connections of the motor(s) and by extension to know if the motor is faulty. Before running the tests, ensure that the 18650 batteries are charged, then prop the robot on a box or similar to prevent it falling off an edge for instance. 

Run the [client node](./lidarbot_base/src/motor_checks_client.cpp) to request the motor checks:

```
ros2 run lidarbot_base motor_checks_client
```

Then run the [server node](./lidarbot_base/src/motor_checks_server.cpp) to check the motors:

```
ros2 run lidarbot_base motor_checks_server
```
These are the steps followed by the server node:
- The server initializes the left and right motor pulse counts to 0. 
- It then runs each motor in the forward direction at 50% speed for 2 seconds. The terminal output is shown below:
  ```
  [INFO] [1699988646.067449176] [rclcpp]: Ready to check motors
  [INFO] [1699988648.190017279] [rclcpp]: Received request to check motors...
  [INFO] [1699988648.190117076] [rclcpp]: Checking left motor...
  [INFO] [1699988652.196330587] [rclcpp]: Checking right motor...
  [INFO] [1699988656.202619229] [rclcpp]: Sending back response...
  ```
- The current pulse counts for each motor are checked to confirm that the pulse is above 0.
- If both motors have their pulse counts above 0, a success message is sent to the client: 

  ```
  [INFO] [1699991078.643028687] [rclcpp]: service not available, waiting again...
  [INFO] [1699991087.233641544] [rclcpp]: The checks were successful!
  ```
- If one or both of the motors do not have pulse counts above zero a warning message is sent from the server to the client and identifies the faulty motor(s). At the moment, however, the message sent to the client does not identify the faulty motor but instead outputs this message when there is an error:

  ```
  terminate called after throwing an instance of 'std::future_error'
    what():  std::future_error: No associated state
  [ros2run]: Aborted
  ```
  This section will be updated once the issue has been fixed.

<p align='center'>
    <img src=docs/images/motor_connection_tests.gif width="400">
</p>

If a motor moves backward instead of forward, swap the cables for the specific motor to change the direction.

After it is confirmed that both motors moved forward, lidarbot can be driven around with the gamepad after running this command:

```
ros2 launch lidarbot_bringup lidarbot_bringup_launch.py
```

This launch file brings up the physical lidarbot, raspberry pi camera, RPLIDAR A1 and also integrates `ros2_control`, `twist_mux`, `robot_localization` and gamepad/joystick control.

**Note:** There are some warning and error messages outputted to the terminal related to the camera. These are mostly related to calibrating the camera and can be ignored. 

## Mapping

The Nav2 stack uses laser scan data to build a map of an environment and to localize the robot in the map while in motion. The SLAM package, [`slam_toolbox`](https://joss.theoj.org/papers/10.21105/joss.02783.pdf), is used to generate a map of Lidarbot’s surroundings with the RPLIDAR A1 by driving the robot around using the [`teleop_twist_joy`](https://index.ros.org/r/teleop_twist_joy/) package with the [gamepad](./lidarbot_teleop/launch/joystick_launch.py).

### Gazebo

Before starting the mapping operation, ensure that the `mode` key, under `ROS Parameters` in the [`mapper_params_online_async.yaml`](./lidarbot_slam/config/mapper_params_online_async.yaml) file, is set to `mapping` and also that the `map_file_name`, `map_start_pose` and the `map_start_at_dock` keys are commented out:

``` 
# ROS Parameters
odom_frame: odom
map_frame: map
base_frame: base_footprint
scan_topic: /scan
use_map_saver: true
mode: mapping #localization

# if you'd like to immediately start continuing a map at a given pose
# or at the dock, but they are mutually exclusive, if pose is given
# will use pose
#map_file_name: /path/to/map_file
#map_start_pose: [0.0, 0.0, 0.0]
#map_start_at_dock: true 
```

To start mapping in a simulation environment, launch the Gazebo simulation of lidarbot on the development machine (which includes the joystick node for teleoperation):

```
ros2 launch lidarbot_gz gz_launch.py
```

For Gazebo Fortress run:

```
ros2 launch lidarbot_gazebo gazebo_launch.py
```

In a separate terminal, navigate to the workspace directory, `lidarbot_ws` for example, and launch `slam_toolbox` with the [`online_async_launch.py`](./lidarbot_slam/launch/online_async_launch.py) file:

```
ros2 launch lidarbot_slam online_async_launch.py \
slam_params_file:=src/lidarbot_slam/config/mapper_params_online_async.yaml \
use_sim_time:=true
```

***Note**: The online asynchronous mode in `slam_toolbox` uses live and the most recent scan data to create a map, to avoid any lags therefore, some scans can be skipped.*

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

### Physical

To begin mapping in the real world, first run the bringup command:

```
ros2 launch lidarbot_bringup lidarbot_bringup_launch.py
```

Then ensure that the [`mapper_params_online_async.yaml`](./lidarbot_slam/config/mapper_params_online_async.yaml) file is configured for mapping (refer to the previous subsection). Then open a new terminal, on the development machine, navigate to the workspace directory and launch `slam_toolbox` with the `use_sim_time` parameter set to `false`:

```
ros2 launch lidarbot_slam online_async_launch.py \
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

To localize a robot in a given map, Nav2 makes use of the Adaptive Monte Carlo Localization (AMCL) package, `nav2_amcl`. This a probabilistic localization module that outputs the pose estimates of a robot taking LIDAR scans, a map (from the Nav2 `map_server`) and transform messages as inputs.

It was noted in the [controllers subsection](#controllers) that the Nav2 stack outputs command velocity messages to the differential drive controller. These are sent on the topic `diff_controller/cmd_vel_unstamped` for lidarbot. The velocity messages are sent to the robot base system hardware component to drive the motors. The updated robot odometry data from this motion is sent to the EKF on the topic `/diff_controller/odom`, and fused with the IMU data from the `imu_broadcaster/imu` topic. The fused odometry data is input into Nav2, in addition to the laser scan data, occupancy grid map and transform messages, to output new velocity messages. This process keeps iterating as the robot moves towards the goal position.

The Nav2 topic graph is shown in the following image adapted from [Linorobot2](https://github.com/linorobot/linorobot2).

<p align='center'>
    <img src=docs/images/nav2_topics.png width="800">
</p>

### Gazebo

Lidarbot will be localized and navigated within the map previously generated with the `slam_toolbox`.

Bring up lidarbot in Gazebo Fortress:

```
ros2 launch lidarbot_gz gz_launch.py
```

Bring up lidarbot in Gazebo classic:

```
ros2 launch lidarbot_gazebo gazebo_launch.py
```

Navigate to the development workspace and open the following terminals.

To open `RViz`:

```
rviz2 -d src/lidarbot_navigation/rviz/lidarbot_nav.rviz
```

To localize lidarbot in the map with `nav2_amcl`:

```
ros2 launch lidarbot_navigation localization_launch.py \
map:=./src/lidarbot_navigation/maps/sim_map.yaml use_sim_time:=true
```

To launch navigation:

```
ros2 launch lidarbot_navigation navigation_launch.py use_sim_time:=true \
map_subscribe_transient_local:=true
```

Make sure to set the `use_sim_time` argument to `true`.

In `RViz`, set the initial pose using the `2D Pose Estimate` button in the toolbar so that lidarbot is aligned correctly both in `RViz` and Gazebo Classic. Afterwards, click on the `2D Goal Pose` and choose a place on the map for lidarbot to navigate to:

<p align='center'>
    <img src=docs/images/gazebo_navigation.gif width="800">
</p>

The blue arrows indicate unfiltered odometry, while the green ones are the filtered odometry. 

### Physical

Run a similar set of commands on the physical lidarbot for localization and navigation. Bring up lidarbot:

```
ros2 launch lidarbot_bringup lidarbot_bringup_launch.py
```

Launch `RViz`:

```
rviz2 -d src/lidarbot_navigation/rviz/lidarbot_nav.rviz
```

To localize lidarbot in the map with `nav2_amcl`:

```
ros2 launch lidarbot_navigation localization_launch.py map:=./real_map.yaml use_sim_time:=false
```

To launch navigation:

```
ros2 launch lidarbot_navigation navigation_launch.py use_sim_time:=false \
map_subscribe_transient_local:=true
```

Make sure to set the `use_sim_time` argument to `false`.

Similarly, set the initial pose using the `2D Pose Estimate` button in the `RViz` toolbar so that lidarbot is aligned correctly both in `RViz` and in its environment. Afterwards, click on the `2D Goal Pose` and choose a place on the map for lidarbot to navigate to. 

To save time, all of these commands can be called from one launch file and adding delays where necessary to ensure certain processes are launched before others to avoid errors. Additional information on setting up navigation with Nav2 is available in this [setup guide](https://docs.nav2.org/setup_guides/index.html).

An ArUco marker was placed on top of lidarbot to track its trajectory during navigation, a Logitech C270 webcam was utilized in tracking the marker.

## ArUco package

To track the trajectory of lidarbot during navigation, an ArUco marker, a binary synthetic square marker, was placed on top of it. Synthetic or fiducial markers are used in [computer vision applications](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) for pose estimation in robot navigation, augmented reality and more. 

The four corners of fiducial markers provide enough correspondence between points in the real world and their 2D image projection to obtain the camera pose estimate.

More information about ArUco markers can be found in [this](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) OpenCV tutorial.

### Generate ArUco marker

The open source computer vision library, [OpenCV](https://docs.opencv.org/4.x/d1/dfb/intro.html), contains an ArUco module based on the [ArUco library](https://www.uco.es/investiga/grupos/ava/portfolio/aruco/), a popular library for detection of square fiducial markers developed by Rafael Muñoz and Sergio Garrido.

Firstly, the `opencv-contrib-python` module needs to be installed and not `opencv-python`: 

```
pip uninstall opencv-python
pip install opencv-contrib-python
```

The computer might need to be restarted for the install to be effected.

Next navigate to the path in the `lidarbot_aruco` pacakge directory:

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
    <img src=docs/images/generated_aruco_marker.png width="400">
</p>

To close the window, press the **q** key on the keyboard on the opened window.

### Webcam calibration

The Logitech webcam C270 HD is used in this project and needs to be calibrated.

First install the [ROS usb camera driver](https://index.ros.org/r/usb_cam/#humble) package:

```
sudo apt install ros-humble-usb-cam
```

Camera calibration was done following the steps outlined this [guide](https://automaticaddison.com/how-to-perform-pose-estimation-using-an-aruco-marker/)

Execute the command below to run the usb-cam driver node:

```
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ~/dev_ws/src/lidarbot_aruco/config/params_1.yaml
```

### Aruco trajectory visualizer node

```
ros2 run lidarbot_aruco aruco_trajectory_visualizer_node
```

Launch file to bringup the usb driver and aruco trajectory visualizer node:

```
ros2 launch lidarbot_aruco trajectory_visualizer_launch.py
```

<p align='center'>
    <img src=docs/images/lidarbot_aruco_marker.png width="600">
</p>

<p align='center'>
    <img src=docs/images/lidarbot_aruco_test.gif width="800">
</p>

The following images show the start and end positions of navigation with lidarbot.

<p align='center'>
    <img src=docs/images/nav_start.png width="400">
</p>

<p align='center'>
    <img src=docs/images/nav_end.png width="400">
</p>

An alternative to using AruCo markers to visualize the robot's trajectory is to use [this](https://github.com/MOGI-ROS/mogi_trajectory_server) ROS2 trajectory visualization server by MOGI-ROS.

## Acknowledgment

- [Articulated Robotics](https://articulatedrobotics.xyz/)
- [Automatic Addison](https://automaticaddison.com/)
- [Diffbot](https://github.com/ros-mobile-robots/diffbot)
- [Linorobot2](https://github.com/linorobot/linorobot2)
- [Mini pupper](https://github.com/mangdangroboticsclub/mini_pupper_ros)
- [Pyimagesearch](https://pyimagesearch.com/)
- [Robotics Backend](https://roboticsbackend.com/)
