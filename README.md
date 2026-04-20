# Azrael mobile manipulator
Collection of ROS2 packages for the Azrael mobile manipulator developed in STIIMA-CNR

[<img src="docs/azrael.jpeg" alt="stiima" width="300">](https://www.stiima.cnr.it/)

---

## Network Setup

RPI4  : ubuntu@192.168.1.10
NUC   : pauli@192.168.1.128

## Quick Start

### 1. Base Motor Control (Raspberry Pi 4)

On your PC:

```bash
ssh ubuntu@192.168.1.10
sudo azrael_base_driver/build/azrael_mobile_driver
```
This starts the low-level base motor driver.

### 1. Core Bringup

```bash
ssh pauli@192.168.1.128
```
#### 2.1 Base driver, odometry, lidar

```bash
ros2 launch azrael_app base_bringup.launch.yml
```

#### 2.2 UR manipulator + gripper bringup

```bash
ros2 launch azrael_app ur_bringup.launch.py
```
Example:
```bash
ros2 launch azrael_app ur_bringup.launch.py fake_ur:=false gripper:=robotiq-2f-140
```
available grippers:
- `robotiq-2f-140`
- `robotiq-2f-85`

### 3 Navigation & Localization (Remote PC)
```bash
ros2 launch azrael_app nav.launch.yml
```
This should start the navigation stack (map server, planner, controller, AMCL, etc.).

### 4 Visualization (Remote RViz)
```bash
ros2 launch azrael_app remote_rviz.launch.py
```

## Manual Teleoperation

### Joystick
```bash
ros2 launch azrael_app joy.launch.yml
```

### Keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/azrael/cmd_vel
```
