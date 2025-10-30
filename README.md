# ROS 2 Humble Autonomous Navigation Project

This repository contains a complete ROS 2 Humble project demonstrating the full pipeline for mobile robot simulation and autonomous navigation. The project includes a custom robot model, environment mapping via SLAM, and two advanced navigation modes: pre-defined waypoint following and dynamic perception-based "search-and-navigate" using Aruco markers.

## 🤖 Project Overview

This project showcases a mobile robot's ability to operate intelligently in a simulated Gazebo environment. The core functionalities include:

* **Robot Modeling:** A custom robot model is defined in URDF, with all coordinate frames (transforms) correctly configured and broadcast using TF2. This includes a base, wheels, and a movable camera link.
* **Environment Mapping (SLAM):** The robot can be manually driven to explore an unknown environment, using the SLAM Toolbox to generate and save a persistent map for later use.
* **Autonomous Navigation (Nav2):**
    1.  **Waypoint Following:** The robot can autonomously navigate a precise, pre-defined sequence of goal poses using a custom Python script that interfaces with the Nav2 stack.
    2.  **Dynamic Perception:** The robot can use its camera to actively scan the environment for Aruco markers. Upon detecting a marker, it dynamically commands the Nav2 stack to navigate to it.

## 📂 Repository Structure

This workspace (`task1_ws`) is organized into two primary packages:

* `src/camp`: This is the main package. It contains all custom **launch files**, **Python scripts** (for navigation and TF), robot/world **models**, **maps**, and custom RViz configurations.
* `src/nav2_pkg`: This package provides the core **Nav2 parameter files** (like `nav2_params.yaml`) and other supporting configurations that are referenced by the launch files in the `camp` package.

## 🛠️ Environment & Prerequisites

Before running this project, ensure you have the following installed:

* **ROS 2 Humble** (on Ubuntu 22.04)
* **Gazebo** (Ignition Gazebo, as used by Humble)
* **ROS 2 Nav2 Stack:**
    ```bash
    sudo apt install ros-humble-nav2-bringup
    ```
* **ROS 2 SLAM Toolbox:**
    ```bash
    sudo apt install ros-humble-slam-toolbox
    ```
* **Perception Packages:**
    ```bash
    # For handling camera data from Gazebo
    sudo apt install ros-humble-ros-gz-image 
    # For Aruco detection
    pip install opencv-contrib-python
    ```

## 🚀 Build Instructions

1.  **Clone the Repository:**
    ```bash
    git clone [https://your-github-repo-url.com/task1_ws.git](https://your-github-repo-url.com/task1_ws.git)
    cd task1_ws
    ```

2.  **Install Dependencies:**
    ```bash
    rosdep install -i --from-path src -y
    ```

3.  **Build the Workspace:**
    ```bash
    colcon build
    ```

4.  **Source the Workspace:**
    > **Note:** You must run this command in every new terminal you open.
    ```bash
    source install/setup.bash
    ```

---

## 🏃‍♂️ How to Run

All commands are run from your workspace root (`task1_ws/`) after sourcing it.

### 1. Visualizing the Robot Model & Transforms

This launch file starts the Gazebo simulation and RViz to inspect the robot model and its TF tree, including the custom dynamic transform.

1.  In one terminal, launch the simulation and RViz:
    ```bash
    ros2 launch camp mylaunch.py
    ```
2.  In a **new terminal**, source the workspace and run the TF broadcaster script:
    ```bash
    ros2 run camp dynamic_carrot_broadcaster.py
    ```
    You should now see the robot, the camera link, and the new TF frame in RViz.

### 2. Mapping the Environment (SLAM)

This command launches the SLAM system, allowing you to create a map of the environment.

1.  Launch the SLAM node (this will also start Gazebo/RViz):
    ```bash
    ros2 launch camp slam_launch.py
    ```
2.  Drive the robot around the environment (e.g., using `ros2 run teleop_twist_keyboard teleop_twist_keyboard`) to build a complete map.
3.  Once the map is complete, save it:
    ```bash
    # This saves the map to your camp/map folder
    ros2 run nav2_map_server map_saver_cli -f src/camp/map/map
    ```

### 3. Running Autonomous Navigation

After saving a map, you can run either of the autonomous navigation modes.

#### Mode A: Waypoint Navigation

This mode makes the robot follow a pre-defined path specified in the `waypoint_navigator.py` script.

1.  Stop the SLAM process (`Ctrl+C`) and launch the full navigation stack with your saved map:
    ```bash
    # Launch Nav2 using your created map
    ros2 launch camp nav2_bringup_launch.py
    ```
2.  In a **new terminal**, source the workspace and run the waypoint navigation script:
    ```bash
    ros2 run camp waypoint_navigator.py
    ```
    The robot will now autonomously navigate to the sequence of goal poses.

#### Mode B: Dynamic Aruco Marker Navigation

This mode uses the robot's camera to find and navigate to Aruco markers in the world.

1.  First, ensure you have a map of the Aruco world (if you haven't, run the SLAM process from Step 2 in that world and save the map as `arucoMap`).
2.  Launch the simulation and navigation stack for the Aruco world:
    ```bash
    # This file should launch Gazebo (with marker.world) and Nav2 (with arucoMap)
    ros2 launch camp arucolaunch.py
    ```
3.  In a **new terminal**, source the workspace and run the Aruco detection and navigation script:
    ```bash
    ros2 run camp aruco_navigation.py
    ```
    The robot will now actively scan for Aruco markers and navigate to them upon detection.

---

## .gitignore

To keep your repository clean, create a `.gitignore` file in the root of `task1_ws` and add the following lines to ignore build, install, and log files:

```
/build/
/install/
/log/
```
