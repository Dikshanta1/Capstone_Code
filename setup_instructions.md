# Setup & Build Instructions
## Autonomous Frontier Exploration Robot

### 1. Directory Tree Structure
Here is the final structure of the workspace containing all configuration and code files:

```text
/home/dikshant/ros2_humble
├── esp32_firmware
│   └── esp32_firmware.ino         # ESP32 firmware with motor control and encoders
└── robot_ws
    └── src
        └── robot_base             # The unified ROS 2 package
            ├── package.xml        # Package dependencies
            ├── setup.py           # Python setuptools install configuration
            ├── setup.cfg
            ├── config
            │   ├── explore_lite.yaml # Frontier exploration parameters
            │   └── nav2_params.yaml  # Navigation stack parameters
            ├── launch
            │   ├── autonomous_exploration.launch.py # Master launch file
            │   └── robot_slam.launch.py             # Basic SLAM launch file
            ├── robot_base
            │   ├── __init__.py
            │   └── base_controller.py  # Hardware UART bridge & Odometry
            ├── rviz
            │   └── rviz_config.rviz    # Visualization settings
            └── urdf
                └── robot.urdf.xacro    # Robot description (dimensions, camera mount)
```

### 2. Install Required ROS 2 Dependencies

Run the following commands on your Jetson Xavier NX terminal to install the necessary packages for ROS 2 Humble:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-realsense2-camera \
  ros-humble-depthimage-to-laserscan \
  ros-humble-rtabmap-ros \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-explore-lite \
  ros-humble-rviz2 \
  python3-serial
```

### 3. Configure UART Permissions (Dialout Group)

To allow the `base_controller.py` node to read/write to the `/dev/ttyTHS0` UART port without requiring `sudo`:

```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
```
*Note: You must log out and log back in (or reboot the Jetson) for the group changes to take effect.*

### 4. Build the Workspace

Navigate to your workspace and build the package:

```bash
cd ~/ros2_humble/robot_ws
colcon build --symlink-install
```

### 5. Launch the Autonomous Robot!

Source the workspace and run the master launch file:

```bash
source install/setup.bash
ros2 launch robot_base autonomous_exploration.launch.py
```

The robot will automatically open RViz, load the camera, build a 3D map, find frontiers (unknown areas), and autonomously drive towards them to completely map your environment!
