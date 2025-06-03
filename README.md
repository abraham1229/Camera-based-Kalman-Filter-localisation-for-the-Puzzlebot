# TE3003B: Integration of Robotics and Intelligent Systems 🤖🌐

## Project Description 📌 

This repository contains all the necessary materials for the "TE3003B: Integration of Robotics and Intelligent Systems" course 📚.  
The course will guide students through a series of exercises that lead to a final challenge in autonomous systems, with a focus on localization, navigation, and intelligent path-planning using Kalman Filters for 2D map-based localization.

## Dependencies & Requirements 📦 
- **Ubuntu 22.04** with **ROS2 Humble** 🐧
- Basic knowledge of robotics (recommended) 🤖
- Access to a **Puzzlebot Jetson/Lidar Edition** 🚗


## Installation & Usage 🚀

Build your ROS 2 workspace
```
colcon build
```
Source the setup file
```
source install/setup.bash
```

To run a single node
```
ros2 run <desired_package> <desired_node.py>
```
To launch multiple nodes using a launch file
```
ros2 launch <desired_package> <desired_launch.py>
```

## Demo Video & Presentation 🎥🧠

This section includes a demonstration of the robot's behavior and a brief explanation of the system’s internal logic.
  
- [View the slides](https://www.canva.com/design/DAGpJeb9LPc/OB18H8y1_iGPbxbvVS0cjQ/edit?utm_content=DAGpJeb9LPc&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)

- [Watch the system in action](https://youtu.be/RbW9n8S3kpg)

## Credits & License 👥 
This project was created as part of a challenge by **Manchester Robotics**.  

Developed by:  
- **Abraham Ortiz** - [GitHub](https://github.com/abrahamortiz) 
- **Alan Flores** - [GitHub](https://github.com/AIF31)
