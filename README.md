# LIVOX-WS
# Project Setup and Execution

This guide will help you set up and run this ROS 2 project. 
Follow these instructions to ensure everything is configured correctly.
Don' foregt to configure the right IP address for the MId-360 (for more information,check https://srl.esa.io/posts/509)
For more information or explanations, check my research paper.

## Prerequisites

Ensure you have ROS 2 Humble installed on your system. You also need the necessary ROS 2 packages and your project workspace set up.
Ensure to clone this repository and unzip livox_ros_driver2.
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
cd LIVOX_WS
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
Terminal 2: Launch the fast_lio node for mapping:

```bash
source /opt/ros/humble/setup.bash
cd LIVOX_WS
source install/setup.bash
ros2 launch fast_lio mapping_mid360.launch.py
```

## Other information 
The map_saver (the important file in it is map_saver_node.py), and the lidar_to_pcd (the important file is obstacle_detection.py, the other files were created before using fast LIO)  folders were devloped by me.
FAST-LIO-ROS2 and livox-ros_driver2 were obtained from an open-access GitHub repository (the site URLs are provided in my research paper).
The mapping_mid360.launch.py (in FAST-LIO-ROS2 folder) was modified to include all the necessary nodes(look at the file directly to understand).
For more explanaion of the code, look at the comments provided in the code and at my research paper as well.
