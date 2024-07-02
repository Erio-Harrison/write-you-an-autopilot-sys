![result](./asset/logo.png)

# write-you-an-autopilot-sys

Take you step by step to realize a simple version of the autonomous driving system.

## Environment

`Ubuntu 22.04.4 LTS` and `ROS2 (Robot Operating System 2)`


## Use

1. Clone this repository, then:

   ```bash
   cd write-you-an-autopilot-sys
   ```
2. We use ZeroMq for network communication:

   ```bash
   sudo apt-get update
   sudo apt-get install libzmq3-dev
   ```

3. Build and Run: 

   ```
   colcon build
   ```

   ```bash
   ros2 launch launch/auto_drive_system.launch.py
   ```

   You may see something like this:

![result](./asset/configure.png)

## License Statement

Note: The content of this tutorial is protected by the Apache License, and the author reserves all copyrights.