# t1_leo_nav

The `t1_leo_nav` repository is a comprehensive suite designed for ROS2, tailored for navigation and simulation of the Leo Rover. This suite is divided into three main packages: `t1_rover`, `t1_sim`, and `t1_nav`, each serving a distinct role in the system.

# Other Packages Included
- `robot_description`
- `rplidar_ros`

# Workspace Layout
Your workspace directory should contain the following packages:
- `leo_description`
- `rplidar_ros`
- `t1_rover`
- `t1_sim`
- `t1_nav`
- `t1_ui`


## Packages Overview

### t1_rover

The `t1_rover` package deals with all the nodes for the Leo Rover. It includes:

- Robot localization node.
- Imu Filter node.
- Launch node to visualize the rover in `rviz2`.

This package encapsulates all necessary components to view the rover standalone in `rviz2` and `gazebo`.
You can also drive the rover around using `teleop_twist_keyboard` or other packages.

### robot_description

Includes the urdf files for the leo robot, this is launched from the `t1_rover` launch file.

### rplidar_ros

Includes different rplidars to launch, our specifc model we are using is the `a2m12`. This is launched from the t1_rover launch file

### t1_sim

`t1_sim` is geared towards simulation. It includes:

- Launch node for Gazebo, providing a realistic simulation environment.
- Bridging of topics to facilitate communication between ROS2 and Gazebo.

This package contains everything necessary for a complete simulation experience.

### t1_nav

The `t1_nav` package is focused on autonomous navigation and mapping, it includes:

- Integration with `nav2` and Behavior Trees for advanced navigation strategies.
- Uses `SLAM Toolbox` for mapping

- There are scripts are used for navigation purposes, you will need to run them from your own terminal
  - `explore.py` for autonomous navigation and mapping using breadth first search.
  - `custom_map_saver_node.py` to save maps to your system automatically.
  - `nav_point.py` to publish teleop twist commands for rover movement.
  - `nav2_nav_test.py` autonomously navigate to patrolling points using nav2, visualize the patrol points using marker (uses `robot_navigate.py`).
  - `robot_navigate.py` containing various functions for navigation using nav2.

The main launch file is `t1_nav`, though `t1_rover` can also be launched independently if you dont want navigation.

# T1 Rover User Interface (t1_ui)

The T1 Rover User Interface (t1_ui) is a graphical application designed to facilitate navigation and control of the T1 Rover system. This user-friendly interface enables operators to interact with the rover, providing essential functionalities for navigation and system control. Key features include:

- **State Machine Control**: Initiate the rover's state machine for autonomous operation.
- **Position Setting and Navigation**: Define target positions and command the rover to navigate to specified coordinates.
- **Emergency Stop**: Halt rover motion instantaneously with the emergency stop functionality.
- **Start Position Configuration**: Configure the rover's start position for accurate navigation.
- **Return to Start Position**: Command the rover to return to its designated starting location.

The application also incorporates robust connectivity checks to ensure seamless communication between the rover and the control system. These checks verify connectivity with both the rover hardware and the NUC (Mini PC), providing real-time status updates to the user.

**This is automatically run with t1_nav for ease of use.**

## Installation

Before running the packages, ensure the following dependencies are installed:

1. **RPLIDAR Package** - make sure this is in the same workspace

```bash
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
```

2. **Gazebo Ignition** - Follow the official Gazebo installation guide.

3. **URDF Files Package** - make sure the `leo_description` package is in the same workspace

```bash
git clone -b ros2 https://github.com/LeoRover/leo_common-ros2.git
```

5. **Rviz2** - Comes pre-installed with ROS2 distributions.
```bash
sudo apt update
sudo apt install ros-<distro>-rviz2
```

5. **Robot_localization**
```bash
sudo apt install ros-<distro>-robot-localization
```

6. **Nav2**
```bash
sudo apt install ros-<distro>-navigation2
```

7. **SLAM**
```bash
sudo apt install ros-<distro>-slam-toolbox
```

Ensure that you have the above packages and tools installed in your ROS2 environment.

# Important
To setup the leo rover with ROS2, please be sure to follow this tutorial: `https://github.com/UoMMScRobotics/UOMDocumentationForLeoRover/blob/main/Task06_Installing_ROS2/LeoOSROS2.md`

# Launching the Leo Rover with Simulation

To launch the simulation environment with t1_sim, use:

```bash
ros2 launch t1_rover rover.launch.py
```

# Launching the Leo Rover with Simulation and Running Autonomous Navigation 

To run the navigation tasks with t1_nav, use:
```bash
ros2 launch t1_nav nav_demo.launch.py
```
*this will also launch `t1_rover` package automatically, you do not need to launch it seperately*

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

