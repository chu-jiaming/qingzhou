# Qingzhou — Autonomous Navigation Robot

[![ROS](https://img.shields.io/badge/ROS-Melodic-22314E?logo=ros)](https://www.ros.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-18.04-E95420?logo=ubuntu)](https://releases.ubuntu.com/18.04/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Nano-76B900?logo=nvidia)](https://developer.nvidia.com/embedded/jetson-nano)

> **Team Lilac 12** — Autonomous navigation system for the Qingzhou robot competition platform, running on NVIDIA Jetson Nano with ROS Melodic.

## Overview

Qingzhou is an indoor autonomous navigation robot. The system integrates SLAM-based localization, global & local path planning, vision-based lane/traffic-sign detection, and cloud communication into a complete autonomous driving pipeline.

## Architecture

```
Sensor Layer                 Planning Layer               Control Layer
┌─────────────┐           ┌──────────────────┐         ┌────────────────┐
│  YDLIDAR X4 │──scan──▶  │    move_base     │         │                │
├─────────────┤           │  ┌────────────┐  │         │  qingzhou      │
│  IMU        │──imu───▶  │  │global_planner│──plan─▶│  _bringup      │──▶ STM32 (MCU)
├─────────────┤           │  │(A*/Dijkstra) │  │         │  (odometry +   │
│  Camera     │──frame─▶  │  └────────────┘  │         │   actuation)   │
│  (vision)   │           │  ┌────────────┐  │         │                │
│             │           │  │TEB local    │──cmd──▶│                │
│             │           │  │_planner     │  │         └────────────────┘
│             │           │  └────────────┘  │
│             │           │  ┌────────────┐  │         ┌────────────────┐
│             │           │  │ costmap_2d  │  │         │  qingzhou      │
│             │           │  │ + layers    │  │         │  _cloud        │──▶ Remote Server
│             │           │  └────────────┘  │         │  (TCP/UDP)     │
│             │           └──────────────────┘         └────────────────┘
└─────────────┘
   vision_vel_pkg
   (lane detection,
    traffic sign OCR)

   robot_pose_ekf + imu_filter_madgwick
        │
        ▼
   fused odometry (/odom_ekf)
```

## Packages

### Custom Packages (Team Lilac 12)

| Package | Description |
|---------|-------------|
| `qingzhou_nav` | Top-level navigation coordinator — launches all subsystems, manages waypoints, coordinates move_base |
| `qingzhou_bringup` | Low-level MCU communication via serial — reads encoders/IMU, publishes odometry, sends velocity commands |
| `qingzhou_cloud` | TCP/UDP cloud communication module for remote monitoring and command |
| `qingzhou_sim` | Gazebo simulation environment — URDF model, control plugins, worlds, RViz config |
| `vision_vel_pkg` | Vision pipeline — Canny/Sobel lane detection, traffic sign OCR (Tesseract + OcrLiteOnnx), TF broadcasting |

### Standard ROS Navigation Stack

| Package | Role |
|---------|------|
| `move_base` | Navigation orchestration — combines global & local planning with recovery behaviors |
| `global_planner` | Global path planning (A* / Dijkstra) |
| `base_local_planner` | Local trajectory planning (Trajectory Rollout) |
| `costmap_2d` | 2D occupancy grid costmap with plugin-based layers (static, obstacle, inflation, voxel) |
| `navfn` | Fast interpolated navigation function planner |
| `nav_core` | Navigation plugin interfaces |
| `clear_costmap_recovery` | Recovery behavior: clears costmaps |
| `rotate_recovery` | Recovery behavior: rotates in place |
| `voxel_grid` | 3D voxel grid for obstacle representation |
| `map_server` | Static map loading and saving |
| `navigation_msgs` | Custom navigation message/action definitions |

### Third-Party Drivers

| Package | Description |
|---------|-------------|
| `ydlidar` | YDLIDAR X4 LiDAR ROS driver |
| `teleop_twist_keyboard` | Keyboard teleoperation utility |

### Sensor Fusion

| Package | Description |
|---------|-------------|
| `robot_pose_ekf` | Extended Kalman Filter fusing wheel odometry + IMU into `/odom_ekf` |
| `imu_calibrate` | IMU calibration tools |

## Hardware

- **Main Computer**: NVIDIA Jetson Nano
- **MCU**: STM32 (serial communication via `/dev/to_stm32`)
- **LiDAR**: YDLIDAR X4
- **Camera**: USB camera (480×360)
- **IMU**: Built-in 6-axis IMU

## Prerequisites

- Ubuntu 18.04
- ROS Melodic
- Python 3.6+
- OpenCV, PyTesseract

```bash
# Install ROS dependencies
sudo apt-get install ros-melodic-navigation ros-melodic-teb-local-planner \
  ros-melodic-imu-filter-madgwick ros-melodic-gmapping ros-melodic-amcl

# Install Python dependencies
pip install opencv-python numpy pytesseract
```

## Building

```bash
mkdir -p ~/qingzhou_ws/src
cd ~/qingzhou_ws/src
# Copy or clone this repository into src/
cd ~/qingzhou_ws
catkin_make
source devel/setup.bash
```

## Usage

### Simulation

```bash
roslaunch qingzhou_gazebo qingzhou_sim.launch
```

### Real Robot

```bash
# Bring up sensors, odometry, and EKF
roslaunch qingzhou_nav qingzhou_bringup.launch

# Launch navigation stack (map, AMCL, move_base)
roslaunch qingzhou_nav qingzhou_move_base.launch

# Launch vision pipeline
roslaunch qingzhou_nav network.launch
```

### Keyboard Teleoperation

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Maps

Pre-built maps are located in `qingzhou_nav/maps/`. To create a new map:

```bash
roslaunch qingzhou_nav gmapping.launch
# Drive the robot around to explore, then:
roslaunch qingzhou_nav map_save.launch
```

## Repository Structure

```
qingzhou/
├── README.md
├── CMakeLists.txt                  # Catkin workspace top-level
├── qingzhou_nav/                   # Navigation orchestration
│   ├── launch/                     # Main launch files
│   ├── config/                     # Navigation parameters
│   ├── maps/                       # Pre-built map files
│   └── src/                        # Navigation node + socket server
├── qingzhou_odom/
│   ├── qingzhou_bringup/           # MCU serial driver
│   ├── robot_pose_ekf/             # EKF sensor fusion
│   └── imu_calibrate/              # IMU calibration
├── qingzhou_sim/
│   ├── qingzhou_description/       # URDF model + meshes
│   ├── qingzhou_gazebo/            # Simulation worlds
│   ├── qingzhou_control/           # Gazebo controllers
│   └── qingzhou_rviz/              # RViz configuration
├── qingzhou_cloud/                 # Cloud communication
├── vision_vel_pkg/                 # Vision processing
│   └── scripts/
│       ├── vision_pipeline.py      # Main vision pipeline
│       ├── lane_filtering.py       # Moving-average-filter lane detection
│       ├── lane_detection_canny.py # Canny edge-detection lane detection
│       ├── lane_detection_sobel.py # Sobel operator lane detection
│       ├── send_tf_node.py         # Coordinate TF broadcaster
│       └── ocr-lite-onnx/          # Lightweight OCR engine
├── move_base/                      # Navigation coordinator
├── global_planner/                 # Global path planner
├── base_local_planner/             # Local trajectory planner
├── costmap_2d/                     # 2D costmap
├── navfn/                          # Navfn global planner
├── nav_core/                       # Navigation plugin interfaces
├── clear_costmap_recovery/         # Clear costmap recovery
├── rotate_recovery/                # Rotate recovery
├── voxel_grid/                     # 3D voxel grid
├── map_server/                     # Map server
├── navigation_msgs/                # Navigation messages
├── ydlidar/                        # YDLIDAR driver
└── teleop_twist_keyboard/          # Keyboard teleop
```

## Team

**Team Lilac 12**

- Email: chujiaming143@gmail.com

## License

This project is for educational and competition purposes. See individual packages for license details.
