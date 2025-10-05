# ROS2 For Beginners - Udemy Course Code Repository

## Project Overview

This repository serves as a personal archive and reference for the code developed while following the "ROS2 For Beginners" course on Udemy. The primary purpose is to preserve my learning journey, exercises, and projects from the course in an easily accessible format. It is not intended as an official or alternative distribution of the course materials, but rather as a personal learning record.

## Course Information

This repository is based on the fantastic "ROS2 For Beginners" course created by **Edouard Renard**.

*   **Course Link:** [https://www.udemy.com/course/ros2-for-beginners/](https://www.udemy.com/course/ros2-for-beginners/?srsltid=AfmBOorR-VRcJygGzLzkz5uOUg3hoa-fGZUAmZYGW0hrmZNa_7fkBz4j&couponCode=PMNVD2525)

I highly recommend this course to anyone looking to get started with ROS2. Edouard's explanations are clear, concise, and incredibly practical.

## About ROS2

The Robot Operating System 2 (ROS2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

## Repository Structure

Here's a breakdown of the main directories and their contents:

*   **`ed-files/`**:
    This directory contains the original code resources and example implementations provided by the instructor, Edouard Renard, throughout the course. This serves as a direct reference to the course material.
    *   `my_cpp_pkg`: A ROS2 package demonstrating C++ node development, including publishers, subscribers, services, and client implementations.
    *   `my_py_pkg`: A ROS2 package demonstrating Python node development, mirroring many of the C++ examples with Python.
    *   `my_robot_bringup`: Contains ROS2 launch files and configuration files (`.yaml`) for orchestrating multiple nodes and setting up complex applications, particularly related to the `turtlesim` simulation.
    *   `my_robot_interfaces`: Defines custom ROS2 message (`.msg`) and service (`.srv`) types used across different packages for inter-node communication.
    *   `turtlesim_catch_them_all`: A Python-based ROS2 package implementing the "Catch Them All" game within the `turtlesim` environment, showcasing intermediate ROS2 concepts.
    *   `turtlesim_project_cpp`: A C++ implementation of the "Catch Them All" `turtlesim` project, providing a comparison to the Python version.

*   **`final-project/`**:
    This folder stores the instructions (in PDF format) for the final project module of the course. It outlines the requirements and goals for applying the learned ROS2 concepts to a more complex scenario.

*   **`ros2_ws/`**:
    This is my personal ROS2 workspace, containing my implementations and modifications of the course exercises and projects. It's structured as a standard ROS2 workspace with a `src/` directory.
    *   **`ros2_ws/src/`**: Inside this are my versions of the ROS2 packages:
        *   `my_cpp_pkg`
        *   `my_py_pkg`
        *   `my_robot_bringup`
        *   `my_robot_interfaces`
        *   `turtlesim_catch_them_all`
        (These mirror the `ed-files` packages but contain my specific code, comments, and any personal experiments.)

*   **`General_ROS2_Workflow_Guide.md`**:
    A personal markdown file containing notes, commands, and a step-by-step guide I compiled to help internalize the standard ROS2 development workflow, from creating packages to building and running nodes.

*   **`README.md`**:
    This file provides a comprehensive overview of the repository, its purpose, the course details, and instructions for setting up and using the code.

---

## Getting Started

To use or run the code in this repository, you'll need a working ROS2 environment (e.g., Foxy, Galactic, Humble, Iron, or rolling).

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/andresalemn/ros2-for-beginners.git
    cd ros2-for-beginners
    ```
2.  **Install dependencies (if any specific to the course):**
    ```bash
    # This might vary depending on the course content and your ROS2 distro
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  **Build the workspace:**
    ```bash
    colcon build
    ```
4.  **Source the setup files:**
    ```bash
    source install/setup.bash
    ```
    (Or `source install/setup.zsh` for Zsh users)
5.  **Run a ROS2 node (example):**
    ```bash
    ros2 run my_package_name my_node_executable
    ```
    Replace `my_package_name` and `my_node_executable` with actual package and executable names from the course.

## Instructor's Resources & Thanks

I extend my sincerest gratitude to **Edouard Renard** for creating such an excellent and comprehensive course on ROS2. His teaching style and the quality of the materials are truly outstanding.

Edouard also runs **RoboticsBackend.com**, a valuable resource for ROS developers:
*   **Website:** [https://roboticsbackend.com/](https://roboticsbackend.com/)

For those looking to dive deeper into ROS2, I highly recommend checking out his book:
*   **ROS 2 from Scratch: Get started with ROS 2 and create robotics applications with Python and C++:** [Link to buy the book](https://www.amazon.com.mx/dp/B0DJCFC29Q?tag=mxtagdefault-20&ge)
## License

This content is for personal educational use. Please refer to the Udemy course for original licensing of course materials. My code implementations are provided "as is" with no specific license intended for commercial use unless otherwise stated within individual files.

## Contact

If you have any questions or feedback regarding this repository, feel free to reach out.

*   **GitHub:** [andresalemn](https://github.com/andresalemn)
*   **LinkedIn:** [andresalemn](https://www.linkedin.com/in/andresalemn)
