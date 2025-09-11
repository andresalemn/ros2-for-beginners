# ROS2 Workspace for "ROS 2 for Beginners"

This ROS2 workspace was created for the Udemy course "ROS 2 for Beginners". The course was originally for ROS Foxy and Humble (2023) and has been updated for ROS Jazzy (2025). This workspace is based on the Humble version.

## Packages

This workspace contains the following packages:

*   **my_cpp_pkg**: C++ package with examples of nodes, topics, services, and parameters.
*   **my_py_pkg**: Python package with examples of nodes, topics, services, and parameters.
*   **my_robot_bringup**: A package for launching the robot.
*   **my_robot_interfaces**: A package for custom messages and services.
*   **turtlesim_catch_them_all**: A package for a turtlesim game.

## Course Information

*   **Course Name**: ROS 2 for Beginners (ROS Jazzy - 2025)
*   **Original Course Name**: ROS 2 for Beginners (ROS Foxy, Humble - 2023)
*   **ROS2 Version**: Humble

## Setup

To use this workspace, you need to have ROS2 Humble installed. Then, you can build the workspace with:

```bash
colcon build
```

And source the workspace with:

```bash
source install/setup.bash
```
