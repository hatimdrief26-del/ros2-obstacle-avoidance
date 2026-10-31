# 🤖 ROS2 Obstacle Avoidance Robot

Academic robotics project featuring a simulated mobile robot that detects and avoids obstacles.

## Overview

This project uses **ROS 2**, **Gazebo Classic**, **RViz 2**, and **Python** to simulate obstacle detection and avoidance. A LiDAR scan topic provides distance measurements, and a ROS 2 node publishes motion commands. A second node monitors robot status.

## Features

- LiDAR-based obstacle detection (`/scan`)
- Motion commands published to `/cmd_vel`
- Configurable safety and emergency-stop distances
- Robot model in URDF/Xacro and Gazebo world in SDF
- RViz visualization configuration
- Nominal and perturbed simulation launch scenarios
- Custom ROS 2 service and action definitions

## Repository structure

```text
src/
├── obstacle_avoider_pkg/
│   ├── obstacle_avoider_pkg/   # Python ROS 2 nodes
│   ├── launch/                # Simulation/scenario launch files
│   ├── config/                # RViz configuration
│   ├── urdf/                  # Robot description
│   ├── worlds/                # Gazebo simulation world
│   ├── package.xml
│   └── setup.py
└── obstacle_avoider_interfaces/
    ├── srv/                   # Custom service
    ├── action/                # Custom action
    ├── CMakeLists.txt
    └── package.xml
```

## Setup and launch

> Requires a compatible **ROS 2 installation on Ubuntu** with Gazebo Classic, `gazebo_ros`, RViz 2, Xacro and `colcon`. The ROS 2 distribution and runtime dependencies have not been verified for this repository.

From the repository root in an appropriately configured ROS 2 terminal:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch obstacle_avoider_pkg simulation_complete.launch.py
```

Alternative scenarios (run one at a time):

```bash
ros2 launch obstacle_avoider_pkg scenario_nominal.launch.py
ros2 launch obstacle_avoider_pkg scenario_perturbe.launch.py
```

If these commands fail, verify the ROS 2 distribution, installed packages and Gazebo Classic compatibility. These instructions are based on the provided launch files and have not been tested here.

## Screenshots

Add your own screenshots of Gazebo and RViz in an `images/` folder, and reference them here.

## Author

**Hatim Drief** — Embedded Systems & Artificial Intelligence engineering student, ENSA Fès, Morocco.
