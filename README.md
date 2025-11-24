# Azrael mobile manipulator
Collection of ROS2 packages for the Azrael mobile manipulator developed in STIIMA-CNR

[<img src="docs/azrael.jpeg" alt="stiima" width="300">](https://www.stiima.cnr.it/)

---

## Network Setup

RPI4  : ubuntu@192.168.1.10(raspberry)  
NUC   : pauli@192.168.1.128(pauli)  

On the RPI4 run the motor driver in azrael_base_driver/build/ as "sudo azrael_mobile_driver"  
On the Nuc run "ros2 launch azrael_driver_udp azrael_driver_bringup.py" to bringup the driver,odometry,lidar,and robot description.  
The "azrael_base_nav.launch.py" provides a basic navigation environment.


```
 ros2 launch azrael_app ur_bringup.launch.py fake_ur:=false headless_mode:=true robot_ip:=192.168.254.100 launch_rviz:=false
```

---

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
#### 2.1 Base driver, odometry, lidar, robot description

#### 2.2 UR manipulator + gripper bringup

### 3 Navigation & Localization (Remote PC)
```bash
ros2 launch azrael_app azrael_nav.launch.py
```
This should start the navigation stack (map server, planner, controller, AMCL, etc.).

#### 3.1 Localization (AMCL lifecycle)
Configure and activate amcl:
```bash
ros2 lifecycle set /amcl configure && ros2 lifecycle set /amcl activate
```
### 4 Visualization (Remote RViz)
```bash
ros2 launch azrael_app remote_rviz.launch.py
```

## Manual Teleoperation
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/azrael/cmd_vel
```
