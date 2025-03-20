# ROS2 Robot Application

This repository contains ROS2 packages for robot control and manipulation, including:

- **grasp_training**: Packages for training grasping algorithms
- **domain_randomization**: Tools for domain randomization in simulation
- **kinova_scene_demo**: Demo scenes for Kinova robot arms
- **ros2_kortex**: Kinova Kortex ROS2 drivers and utilities

## Setup

Clone this repository into your ROS2 workspace:

```bash
git clone https://github.com/zxjAiLun/robot-app.git
cd robot-app
colcon build
```

## Usage

Launch the robot demo:

```bash
ros2 launch grasp_training grasp_demo.launch.py
``` 