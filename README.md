# Autonomous Frontier Exploration Robot (4WD)
## Capstone Robotics Project

An autonomous mobile robot platform built on **ROS 2 Humble**, featuring:
- **RTAB-Map 3D Visual SLAM** via Intel RealSense D435i
- **Nav2 (Navigation2)** stack for path planning and obstacle avoidance
- **Explore Lite** for autonomous frontier-based exploration and room mapping
- **ESP32 Dual-Core Microcontroller** for low-level differential drive motor control and encoder feedback over high-speed hardware UART

---

## 📋 System Requirements & Prerequisites

Each team member working on this project must have:
- **Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS Distribution:** ROS 2 Humble Hawksbill (Desktop Install)
- **Git & Build Tools:** `git`, `python3-pip`, `colcon`

---

## 🚀 Quickstart for Team Members (First-Time Setup)

### 1. Clone the Repository
Open a terminal on your machine and clone the project:
```bash
git clone https://github.com/Dikshanta1/Capstone_Code.git
cd Capstone_Code
```

### 2. Install Required Dependencies
Run the following command to install all ROS 2 packages, build tools, and Python serial libraries:
```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-serial \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-realsense2-camera \
  ros-humble-depthimage-to-laserscan \
  ros-humble-rtabmap-ros \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-explore-lite \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard
```

### 3. Build the ROS 2 Workspace
```bash
cd robot_ws
colcon build --symlink-install
```
*(Note: `--symlink-install` allows you to edit Python files and launch scripts without rebuilding every time!)*

### 4. Source the Setup File
Every time you open a new terminal to run robot commands, source your workspace:
```bash
source install/setup.bash
```
> **Tip:** You can automatically source this by adding it to your `~/.bashrc`:
> ```bash
> echo "source ~/Capstone_Code/robot_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 🖥️ Running the Robot (Choose Your Mode)

### Mode 1: Full Autonomous Exploration (On Real Robot / Jetson)
*Requires RealSense D435i and ESP32 connected to Jetson.*
This launches RealSense, Depth-to-LaserScan, RTAB-Map SLAM, Nav2 Navigation, Frontier Exploration, Base Controller, and RViz:
```bash
cd ~/Capstone_Code/robot_ws
source install/setup.bash
ros2 launch robot_base autonomous_exploration.launch.py
```

### Mode 2: SLAM & Manual Mapping with Keyboard
If you want to manually drive the robot to map an environment before running full autonomy:
```bash
# Terminal 1: Launch robot base, camera, and SLAM
cd ~/Capstone_Code/robot_ws
source install/setup.bash
ros2 launch robot_base robot_slam.launch.py

# Terminal 2: Teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Mode 3: Testing on a Personal Laptop (No Robot Hardware)
If you are developing algorithms, tuning configs, or checking the robot URDF model on a laptop without physical sensors:
1. **Visualize Robot URDF & TF frames in RViz:**
   ```bash
   ros2 launch robot_state_publisher robot_state_publisher.launch.py \
     robot_description:=$(ros2 run xacro xacro ~/Capstone_Code/robot_ws/src/robot_base/urdf/robot.urdf.xacro)
   ```
2. **Note on Base Controller without Hardware:**
   `base_controller.py` connects to `/dev/ttyTHS0` by default. On a laptop without an ESP32 connected, it will retry connection every 2 seconds. This is expected. If you plug in an ESP32 via USB on your laptop, change `serial_port` parameter to `/dev/ttyUSB0`.

---

## ⚡ ESP32 Firmware & Hardware Setup

The firmware for the low-level motor controller is located in [`esp32_firmware/esp32_firmware.ino`](file:///home/dikshant/ros2_humble/esp32_firmware/esp32_firmware.ino).

### Flashing the ESP32:
1. Open the Arduino IDE on your machine.
2. Go to **Preferences** and ensure ESP32 board support is installed:
   - Board URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Select Board: **ESP32 Dev Module**.
4. Open `esp32_firmware/esp32_firmware.ino`.
5. Connect ESP32 via USB and click **Upload**.

### Hardware Wiring (ESP32 to Jetson Xavier NX 40-Pin Header):
| ESP32 Pin | Jetson 40-Pin Header | Function |
| :--- | :--- | :--- |
| **Pin 14 (TXD)** | **Pin 10 (UART1_RXD)** | Serial TX -> RX (`/dev/ttyTHS0`) |
| **Pin 13 (RXD)** | **Pin 8 (UART1_TXD)** | Serial RX <- TX (`/dev/ttyTHS0`) |
| **GND** | **Pin 6 / 9 / 14 (GND)** | **MANDATORY Common Ground** |

### Jetson UART Permission (One-Time Setup):
To allow the Jetson user to access `/dev/ttyTHS0` without `sudo`:
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
sudo cp ~/Capstone_Code/setup_scripts/99-robot-uart.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
*(Log out and log back in or reboot for changes to apply).*

---

## 📂 Project Directory Structure

```text
Capstone_Code/
├── README.md                          # Project documentation and team guide
├── setup_instructions.md              # Hardware and environment instructions
├── .gitignore                         # Prevents build/install artifacts from being pushed
├── esp32_firmware/
│   └── esp32_firmware.ino             # ESP32 firmware (PWM motors, quadrature encoders, watchdog)
├── setup_scripts/
│   ├── 99-robot-uart.rules            # Udev rules for hardware UART permissions
│   └── requirements.txt               # Python dependencies
└── robot_ws/                          # ROS 2 Colcon Workspace
    └── src/
        └── robot_base/                # Unified ROS 2 package
            ├── package.xml            # ROS 2 package dependencies
            ├── setup.py               # Python entry points and install targets
            ├── setup.cfg
            ├── config/
            │   ├── explore_lite.yaml  # Frontier exploration parameters
            │   └── nav2_params.yaml   # Nav2 planner, costmap, and controller parameters
            ├── launch/
            │   ├── autonomous_exploration.launch.py  # Master launch file
            │   └── robot_slam.launch.py              # SLAM + Camera + Base launch
            ├── robot_base/
            │   ├── __init__.py
            │   └── base_controller.py # UART hardware bridge & Odometry publisher
            ├── rviz/
            │   └── rviz_config.rviz   # Pre-configured RViz display setup
            └── urdf/
                └── robot.urdf.xacro   # Complete robot physical dimensions & sensor frames
```

---

## 🤝 Team Rules: DOs and DON'Ts

| ✅ DO | ❌ DON'T |
| :--- | :--- |
| **DO** pull latest changes before starting work: `git pull origin main` | **DON'T** commit `build/`, `install/`, or `log/` folders. |
| **DO** create feature branches for new work: `git checkout -b feature/your-feature-name` | **DON'T** edit files inside `robot_ws/install/` (they will be overwritten on build; always edit inside `robot_ws/src/robot_base/`). |
| **DO** rebuild workspace after pulling new code: `colcon build --symlink-install` | **DON'T** push directly to `main` without testing that the package compiles. |
| **DO** test your Python code with `colcon build` to catch syntax errors early. | **DON'T** commit personal secrets, Wi-Fi passwords, or API keys. |

---

## 🔧 Troubleshooting Guide

### 1. `Failed to connect to /dev/ttyTHS0: [Errno 13] Permission denied`
- **Fix:** Add your user to the `dialout` and `tty` groups, then reboot:
  ```bash
  sudo usermod -a -G dialout $USER
  sudo usermod -a -G tty $USER
  sudo reboot
  ```

### 2. `colcon: command not found`
- **Fix:** Install colcon extensions:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```

### 3. RealSense camera not detected or frame drops
- Make sure the RealSense D435 is plugged into a **USB 3.0** port (blue port). USB 2.0 ports cannot carry depth stream bandwidth.
- Verify device is detected:
  ```bash
  lsusb | grep Intel
  ```

### 4. RViz shows `Fixed Frame [map] does not exist`
- This is normal for the first 3-5 seconds while RTAB-Map initializes. Once the camera detects features and builds the initial point cloud, the `map -> odom` transform will activate automatically.
