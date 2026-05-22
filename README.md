# ROS 2 Autonomous Navigation and Perception Project

This repository contains a complete ROS 2 workspace dedicated to autonomous navigation and perception tasks.

## Repository Structure

```text
Ros2-Autonomous-Navigtion-and-Perception-Project/
├── software/
│   └── ros2_ws/                  # The core ROS 2 colcon workspace
│       └── src/                  # ROS 2 packages and nodes
```

## Getting Started

To build and run the ROS 2 workspace:

```bash
cd software/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Contributing
Please see the [CONTRIBUTING.md](CONTRIBUTING.md) guidelines for adding new nodes or modifying existing ROS 2 packages.

## License
Released under the [MIT License](LICENSE).
