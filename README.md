# auto_drive

An autonomous driving perception and planning system based on ROS2, including obstacle detection, tracking and path planning functions.

Data flow: sensor data -> obstacle detection -> obstacle tracking -> path planning -> control instructions (analog output)

![result](./asset/system_design.png)

## Table of contents

- [Install](#Install)
- [Use](#Use)
- [Contribution](#Contribution)

## Install

[ROS2 Basic](https://github.com/Erio-Harrison/ros2_basic)


## Use

1. Clone this repository, then:

   ```bash
   cd auto_drive
   ```

2. Build and Run: 

   ```
   colcon build
   ```

   ```bash
   ros2 launch launch/auto_drive_system.launch.py
   ```

![result](./asset/configure.png)

## Contribution

If anyone wants to add examples based on this, please directly apply for PR.