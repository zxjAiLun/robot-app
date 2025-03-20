# ROS2 Robot Application

This repository contains ROS2 packages for controlling a Kinova Gen3 robot arm with a RealSense camera.

## Prerequisites

- ROS2 Jazzy
- Gazebo
- Kinova ROS2 packages
- RealSense ROS2 packages

## Installation

1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/yourusername/robot-app.git
```

3. Install dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-jazzy-realsense2-description ros-jazzy-robotiq-description
```

4. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

5. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launch the Simulation

To launch the simulation with the robot and table:

```bash
ros2 launch grasp_training grasp_demo.launch.py
```

This will:
1. Start Gazebo with the table world
2. Load the Kinova Gen3 robot model
3. Start the robot controllers
4. Launch the grasp training node

### Test Grasp

To test the grasp functionality:

```bash
ros2 launch grasp_training test_grasp.launch.py
```

## Package Structure

- `grasp_training`: Main package for robot control and grasp training
- `table_drawer`: Package containing the table and drawer models
- `ros2_kortex`: Kinova robot description and control package

## Dependencies

- `realsense2_description`: RealSense camera models
- `robotiq_description`: Robotiq gripper models
- `ros2_control`: Robot control framework
- `gazebo_ros2_control`: Gazebo integration for ros2_control

## Troubleshooting

1. If you see errors about missing mesh files:
   - Make sure you have installed `realsense2_description` and `robotiq_description`
   - Check that the mesh files are in the correct locations

2. If the robot doesn't move:
   - Check that the controllers are loaded and active
   - Verify the robot's URDF is loaded correctly

3. If Gazebo doesn't start:
   - Make sure Gazebo is installed
   - Check that the world file is accessible

## License

This project is licensed under the MIT License - see the LICENSE file for details. 