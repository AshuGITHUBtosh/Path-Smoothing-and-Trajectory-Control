# Robotics Assignment: Path Smoothing and Trajectory Control in 2D Space

### Setup and Execution Instructions

#### Prerequisites
- ROS 2 Humble (Ubuntu 22.04 recommended)
- `colcon` build system
- Python 3.10+
- Dependencies: `numpy`

### Clone the package into your ROS 2 workspace
- cd ~/rosx_ws/src
- git clone https://github.com/AshuGITHUBtosh/Path-Smoothing-and-Trajectory-Control.git
### Build the package and source the workspace
- cd ~/ros2_ws
- colcon build --symlink-install
- source install/setup.bash
### rviz2
- Set Fixed Frame → odom
- Path (/smoothed_path)
- Path (/actual_path)
- Odometry (/odom)
- TF

### Launch simulator + path generator + controller
ros2 launch robot_nav_assignment nav_demo.launch.py
