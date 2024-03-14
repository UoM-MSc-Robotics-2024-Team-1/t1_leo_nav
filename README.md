# t1_leo_nav

The `t1_leo_nav` repository is a comprehensive suite designed for ROS2, tailored for navigation and simulation of the T1 rover. This suite is divided into three main packages: `t1_rover`, `t1_sim`, and `t1_nav`, each serving a distinct role in the ecosystem.

## Packages Overview

### t1_rover

The `t1_rover` package acts as the description package for the T1 rover. It includes:

- URDF files for the rover's physical description.
- Robot localization nodes.
- Launch files to visualize the rover in `rviz2`.

This package encapsulates all necessary components to view the rover standalone in `rviz2`.

### t1_sim

`t1_sim` is geared towards simulation. It includes:

- Launch files for Gazebo, providing a realistic simulation environment.
- Bridging of topics to facilitate communication between ROS2 and Gazebo.

This package contains everything necessary for a complete simulation experience.

### t1_nav

The `t1_nav` package is focused on navigation and includes:

- Integration with `nav2` and Behavior Trees for advanced navigation strategies.
- `explore.py` for autonomous navigation and mapping.
- `custom_map_saver_node.py` to save maps to your system.
- `nav_point.py` to publish teleop twist commands for rover movement.
- `simple_nav.py` for waypoint navigation.
- `robot_navigate.py` containing various functions for navigation.

The main launch file is `t1_nav`, though `t1_rover` can also be launched independently.

## Installation

Before running the packages, ensure the following dependencies are installed:

1. **RPLIDAR Package**

```bash
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
```

2. **Gazebo Ignition** - Follow the official Gazebo installation guide.
3. **Rviz2** - Comes pre-installed with ROS2 distributions.
4. **Robot_localization**
5. **Nav2**
6. **SLAM**

Ensure that you have the above packages and tools installed in your ROS2 environment.

# Usage
Configuring the URDF

Before launching, ensure you update the macros.urdf file within the t1_rover package to include your absolute directory path for the mesh files. - This bug is due to be fixed shortly

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

Contributions to t1_leo_nav are welcome!

License

This project is licensed under the MIT License - see the LICENSE file for details.

vbnet
