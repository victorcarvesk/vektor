# Vektor
[![Licence](https://img.shields.io/badge/License-MIT-green.svg)](./LICENSE)

Design of a ground robot based on ROS 2 Humble and simulated on the Ignition Gazebo Fortress.

## Overview

This repository is organized in the following directory structure:

- [bringup](./vektor_bringup/) → stores robot_state_publisher (rsp) and RViz launch files and RViz configuration files.
- [description](./vektor_description/) → stores XACRO description files, robot meshes, and configuration files for ros2_control.
- [navigation](./vektor_navigation/) → stores map files for navigation with nav2.
- [simulation](./vektor_simulation/) → stores simulation launch files (bridge, controllers, spawn and the simulation itself).
- [teleoperation](./vektor_teleoperation/) → stores a gamepad node and his launch file to teleoperate the robot with a gamepad.
- [world](./vektor_world/) → stores world files (SDF) and its components.


## Prerequisites
- [Ubuntu 22.04.3 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Ignition Gazebo Fortress](https://gazebosim.org/docs/fortress/ros_installation)
- [ros2_control](https://control.ros.org/humble/doc/getting_started/getting_started.html#binary-packages): `sudo apt install ros-humble-ign-ros2-control ros-humble-ign-ros2-control-demos`

## Installation
1. Create a workspace directory and go into it:
```bash
mkdir vektor_ws && cd vektor_ws
```
2. Clone this repo as `src`:
```bash
git clone -b humble git@github.com:victorcarvesk/vektor.git src
```
3. Build the project
```bash
colcon build
```

## Usage

1. Source the ROS 2 Humble environment:
```bash
source /opt/ros/humble/setup.bash
```
2. Source project environment:
```bash
source install/setup.bash
```
3. Launch simulation:
```bash
ros2 launch vektor_simulation simulation.launch.py
```
4.  Teleoperate:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### ros2_control
1. Add `use_ros2_control:=true` at the end of simulation launch command:
```bash
ros2 launch vektor_simulation simulation.launch.py use_ros2_control:=true
```
2. Teleoperate
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
```

### RViz
There is 2 (two) ways to launch RViz:
- Add `use_rviz:=true` at the end of simulation launch command:
```bash
ros2 launch vektor_simulation simulation.launch.py use_rviz:=true
```
- Launch RViz directly:
```bash
ros2 launch vektor_bringup rviz.launch.py 
```

### Teleoperation with a gamepad
> [!WARNING]
> This node has not yet been integrated into ros2_control controllers of this robot.
- Launch gamepad node:
```bash
ros2 launch vektor_teleoperation teleop_joy.launch.py
```
> [!IMPORTANT]
> Control the robot with the right stick of the gamepad (tested with a Xbox controller).

## Contribution
This project is based on the robot developed by [Josh Newans](https://github.com/joshnewans) in the "[Build a mobile robot](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)" playlist on the YouTube channel [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics).

## License
This project is licensed under the [MIT license](./LICENSE).
