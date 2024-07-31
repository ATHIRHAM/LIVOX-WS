# LIVOX-WS
# Project Setup and Execution

This guide will help you set up and run this ROS 2 project.
Dont forget to first clone the respiratory and then Follow these instructions to ensure everything is configured correctly.

## Prerequisites

Ensure you have ROS 2 Humble installed on your system. You also need the necessary ROS 2 packages and your project workspace set up.
Ensure to clone this respiratory and unzip livox_ros_driver2.
## Setup Instructions

### 1. Source the ROS 2 Environment

Before performing any actions, you need to configure your environment for ROS 2. Open a terminal and run the following command to source the ROS 2 Humble setup script:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Build the livox_ros_driver2 Package

```bash
cd LIVOX_WS/src/livox_ros_driver2/
./build.sh humble
```
### 3.Source the Workspace Setup Script:
```bash
cd LIVOX_WS
source install/setup.bash
```
### 4.Launch ROS 2 Nodes
Open two additional terminals for launching the ROS 2 nodes.

Terminal 1: Launch the livox_ros_driver2 node with RViz:

```bash
source /opt/ros/humble/setup.bash
source /path/to/LIVOX_WS/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
Terminal 2: Launch the fast_lio node for mapping:

```bash
source /opt/ros/humble/setup.bash
source /path/to/LIVOX_WS/install/setup.bash
ros2 launch fast_lio mapping_mid360.launch.py
```
Replace /path/to/LIVOX_WS with the actual path to your workspace directory.
