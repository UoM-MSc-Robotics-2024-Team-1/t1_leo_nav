# t1_leo_nav

The `t1_leo_nav` repository is a comprehensive suite designed for ROS2, tailored for navigation and simulation of the Leo Rover. This suite is divided into three main packages: `t1_rover`, `t1_sim`, and `t1_nav`, each serving a distinct role in the system.

## Packages Overview

### t1_rover

The `t1_rover` package deals with all the files for the Leo Rover. It includes:

- Robot localization node.
- Imu Filter node
- Launch files to visualize the rover in `rviz2`.

This package encapsulates all necessary components to view the rover standalone in `rviz2` and `gazebo`.

### robot_description

This package is from: `https://github.com/LeoRover/leo_common-ros2`

It includes the urdf files for the robot

### t1_sim

`t1_sim` is geared towards simulation. It includes:

- Launch files for Gazebo, providing a realistic simulation environment.
- Bridging of topics to facilitate communication between ROS2 and Gazebo.

This package contains everything necessary for a complete simulation experience.

### t1_nav

The `t1_nav` package is focused on autonomous navigation and mapping, it includes:

- Integration with `nav2` and Behavior Trees for advanced navigation strategies.
- Uses SLAM Toolbox for mapping

- There are scripts that are currently underdevelopment but still function and can be looked at for self learning
  - `explore.py` for autonomous navigation and mapping.
  - `custom_map_saver_node.py` to save maps to your system automatically.
  - `nav_point.py` to publish teleop twist commands for rover movement.
  - `simple_nav.py` for waypoint navigation using nav2 (uses `robot_navigate.py`).
  - `robot_navigate.py` containing various functions for navigation using nav2.
  - ignore other scripts

The main launch file is `t1_nav`, though `t1_rover` can also be launched independently if you dont want navigation.

## Installation

Before running the packages, ensure the following dependencies are installed:

1. **RPLIDAR Package** - make sure this is in the same workspace

```bash
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
```

2. **Gazebo Ignition** - Follow the official Gazebo installation guide.

4. **Rviz2** - Comes pre-installed with ROS2 distributions.
```bash
sudo apt update
sudo apt install ros-<distro>-rviz2
```

5. **Robot_localization**
```bash
sudo apt install ros-<distro>-robot-localization
```

7. **Nav2**
```bash
sudo apt install ros-<distro>-navigation2
```

9. **SLAM**
```bash
sudo apt install ros-<distro>-slam-toolbox
```

Ensure that you have the above packages and tools installed in your ROS2 environment.

# Important
To setup the leo rover with ROS2, please be sure to follow this tutorial: `https://github.com/UoMMScRobotics/UOMDocumentationForLeoRover/blob/main/Task06_Installing_ROS2/LeoOSROS2.md`

# Launching the Simulation

To launch the simulation environment with t1_sim, use:

```bash
ros2 launch t1_rover rover.launch.py
```

# Running Navigation

To run the navigation tasks with t1_nav, use:
```bash
ros2 launch t1_nav nav_demo.launch.py
```

# Launching without Simulation
The launch files will use sim by default, to launch without simulation:

```bash
ros2 launch t1_rover rover.launch.py use_sim_time:=false
ros2 launch t1_nav nav_demo.launch.py use_sim_time:=false
```

# Contributing

Contributions to t1_leo_nav are welcome! :)

# License

This project is licensed under the MIT License - see below for details.
MIT License

```

MIT License

Copyright (c) 2023 [Brandon Skinner]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

```

