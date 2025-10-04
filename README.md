# Robotics Assignment: Path Smoothing and Trajectory Control in 2D Space

### Setup and Execution Instructions

#### Prerequisites
- ROS 2 Humble (Ubuntu 22.04 recommended)
- `colcon` build system
- Python 3.10+
- Dependencies: `numpy`

```bash```
sudo apt update
sudo apt install python3-colcon-common-extensions python3-pip -y
pip3 install numpy

# Clone or copy this package into your ROS 2 workspace
cd ~/ros2_ws/src
git clone <this-repo-url> robot_nav_assignment

# Build the package
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
# rviz2
- Set Fixed Frame → odom
- Path (/smoothed_path)
- Path (/actual_path)
- Odometry (/odom)

# Launch simulator + path generator + controller
ros2 launch robot_nav_assignment nav_demo.launch.py
