# ROS 2 Humble Autonomous Navigation Project

This repository contains a complete ROS 2 Humble project demonstrating the full pipeline for mobile robot simulation and autonomous navigation. The project includes a custom robot model, environment mapping via SLAM, and two advanced navigation modes: pre-defined waypoint following and dynamic perception-based "search-and-navigate" using Aruco markers.

## 🤖 Project Overview

This project showcases a mobile robot's ability to operate intelligently in a simulated Gazebo environment. The core functionalities are built using ROS 2, Gazebo, and the Nav2 stack.

### Core Functionalities

* **1. Modeling & Visualization:**
    * A custom Husky robot model is defined in `model.urdf`.
    * A standalone launch file, `visualize.launch.py`, allows for inspecting the URDF, transforms (TF), and joint states in RViz without running a full simulation.
    * A Python script, `dynamic_carrot_broadcaster.py`, verifies the `base_link` to `camera_link` transform and demonstrates broadcasting a new dynamic TF frame (`carrot`).

* **2. Environment Mapping (SLAM):**
    * The project uses `slam_toolbox` for 2D LiDAR-based mapping.
    * The `slam_launch.py` file acts as a wrapper, loading the specific tuning parameters from `camp/config/mapper_params_online_async.yaml`.
    * This allows the robot to be driven through an unknown world to generate and save a map, which is required for navigation.

* **3. Autonomous Navigation (Nav2):**
    * **Mode A: Waypoint Navigation:** The robot can autonomously navigate a precise, pre-defined sequence of goal poses. The `mylaunch.py` file starts the simulation and the full Nav2 stack (loading `map.yaml`). The `waypoint_navigator.py` script then commands the robot, setting its initial pose to match the Gazebo spawn point `(-7, 5)` and intelligently proceeding through a list of waypoints.
    * **Mode B: Dynamic Perception (Aruco Navigation):** The robot uses its camera to find Aruco markers in the world. The `arucolaunch.py` file starts the simulation (`marker.world`) and spawns the marker. The `nav2_bringup_launch_aruco.py` file loads the corresponding `arucoMap.yaml` and Nav2 stack. Finally, the `aruco_navigation.py` script (the perception brain) identifies markers and sends navigation goals to them.

## 📂 Repository Structure

This workspace (`task1_ws`) is organized into two primary packages:

* `src/camp`: This is the **main package**. It contains all custom **launch files**, **Python scripts** (for navigation and TF), robot/world **models**, **maps**, and custom RViz configurations.
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
* **Joint State Publisher GUI:**
    ```bash
    sudo apt install ros-humble-joint-state-publisher-gui
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

## 🏃‍♂️ How to Run: Step-by-Step Guide

All commands are run from your workspace root (`task1_ws/`) after sourcing it.

### Step 1 (Optional): Visualize Robot Model & TFs

This step is for verifying your URDF and transforms without a full simulation.

1.  **In Terminal 1:** Launch the visualization nodes:
    ```bash
    ros2 launch camp visualize.launch.py
    ```
2.  **In Terminal 2:** Run the dynamic TF broadcaster:
    ```bash
    ros2 run camp dynamic_carrot_broadcaster.py
    ```
* **Result:** RViz will open. You will see the robot model, a `joint_state_publisher_gui` window to move the camera joint, and a new TF frame named `carrot` broadcasted 1m in front of the robot.

### Step 2: Create a Map (SLAM)

Here, we will map the `task2.world` environment.

1.  **In Terminal 1:** Launch the Gazebo simulation. `arucolaunch.py` is perfect for this, as it launches Gazebo and the robot without starting the conflicting navigation nodes (AMCL/Map Server).
    ```bash
    # We'll re-use arucolaunch but tell it to load task2.world
    ros2 launch camp arucolaunch.py world:=task2.world
    ```
2.  **In Terminal 2:** Launch the SLAM node:
    ```bash
    ros2 launch camp slam_launch.py
    ```
3.  **Action:** Drive the robot around the map (e.g., using `ros2 run teleop_twist_keyboard teleop_twist_keyboard`) until the map in RViz is complete.
4.  **In Terminal 3:** Save your new map:
    ```bash
    # Saves the map to the 'camp/map' directory
    ros2 run nav2_map_server map_saver_cli -f src/camp/map/map
    ```

### Step 3: Run Autonomous Navigation (Mode A: Waypoint)

Now we will use the map you just made to navigate a pre-defined path.

1.  **In Terminal 1:** Launch the full simulation and Nav2 stack. `mylaunch.py` is built for this. It starts Gazebo, loads `map.yaml`, and starts AMCL and the map server.
    ```bash
    ros2 launch camp mylaunch.py
    ```
2.  **In Terminal 2:** Run the waypoint navigation script:
    ```bash
    ros2 run camp waypoint_navigator.py
    ```
* **Result:** The robot will initialize its pose at `(-7, 5)` (matching the spawn point in `mylaunch.py`) and begin navigating to the three goal poses defined in the script. The script is smart enough to skip to the next waypoint if it gets within 0.5m of its current goal.

### Step 4: Run Autonomous Navigation (Mode B: Aruco)

This mode requires its own map (`arucoMap.yaml`) and a different world.

**4a. First, Map the Aruco World:**
(If you haven't already, repeat **Step 2** for the Aruco world)

1.  **Term 1 (Sim):** `ros2 launch camp arucolaunch.py` (it defaults to `marker.world`)
2.  **Term 2 (SLAM):** `ros2 launch camp slam_launch.py`
3.  **Term 3 (Save):** `ros2 run nav2_map_server map_saver_cli -f src/camp/map/arucoMap`

**4b. Run Aruco Navigation:**

1.  **In Terminal 1:** Launch the Gazebo simulation.
    ```bash
    ros2 launch camp arucolaunch.py
    ```
2.  **In Terminal 2:** Launch the Nav2 stack, this time using the `arucoMap.yaml`.
    ```bash
    ros2 launch camp nav2_bringup_launch_aruco.py
    ```
3.  **In Terminal 3:** Run the Aruco perception and navigation script:
    ```bash
    ros2 run camp aruco_navigation.py
    ```
* **Result:** The robot will now actively look for Aruco markers. When it finds one, it will automatically send a navigation goal to its location.

---

## 💡 Project Notes & Improvements

* **Absolute Paths:** The files `nav2_bringup_launch.py` and `nav2_bringup_launch_aruco.py` use an **absolute path** (`/home/priverse/...`) to the map file. This is not portable. For a truly robust project, you should modify these files to use `get_package_share_directory('camp')` to find the map file, just as your other launch files do.
* **Launch File Redundancy:** You have multiple files launching parts of the Nav2 stack (`mylaunch.py`, `arucolaunch.py`, `nav2_bringup_launch.py`). A more advanced setup would involve a single, modular `bringup.launch.py` file that accepts arguments (like `world_name`, `map_name`, `run_slam`, etc.) to avoid code duplication.
* **TF Prefix:** The `dynamic_carrot_broadcaster.py` script looks for `husky_robot_model__base_link`. The `__` double underscore implies a namespace. This is correct, but be aware that your TF frames are namespaced, which is important for multi-robot systems.

## .gitignore

To keep your repository clean, create a `.gitignore` file in the root of `task1_ws` and add the following lines to ignore build, install, and log files:

```
/build/
/install/
/log/
```
