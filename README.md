# Robotteknologi 5. Semester



## 📋 Project Overview

This system integrates:
- **Robot Control**: UR3e manipulation via MoveIt2 motion planning
- **Vision System**: Intel RealSense camera-based corrosion detection using thresholding for point cloud generation
- **Teleoperation UI**: PyQt6-based user interface with corrosion adjustments and basic UR control
- **Parameterisation**: Transformation of pointcloud to spline
- **Path Planning**: Automated path generation over corrosion area
- **Tool Orientation**: Tool orientation over path of corrosion area

## 🛠️ System Requirements

### Operating System
- Ubuntu 24.04

### ROS2 Distribution
- **ROS2 Jazzy**

### Hardware Requirements
- UR3e Robot Arm
- Intel RealSense D435 camera

## 📁 Workspace Structure

```
Vattenfall_ws/src/
├── corrosion_detection/        # Corrosion detection pipeline (ML-based)
├── debug/                       # Debugging utilities
├── launch_module/               # Centralized launch files
├── parameterization/            # Robot parameterization service
├── path_planning/               # Inspection path generation
├── realsense_publisher/         # RealSense camera publisher
├── realsense_wrapper/           # RealSense driver wrapper
├── robodk_and_communication/    # RoboDK simulation interface
├── tool_orientation/            # Tool orientation control
├── ur3e_workstation/            # URDF, launch files, MoveIt config
└── user_interface/              # PyQt6 teleoperation UI
```


## 🧩 Installation – ROS 2 Jazzy
This project targets **Ubuntu 24.04** and uses the
[ROS 2 Jazzy Ubuntu Development Setup](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html)
as its installation baseline.
### Set locale (ensure UTF-8)
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
### Enable Ubuntu universe repo
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
### Add ROS2 apt sources
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
### Install development tools (optional)
```bash
sudo apt update && sudo apt install ros-dev-tools
```
### Update package cache
```bash
sudo apt update
```
### Upgrade system (recommended)
```bash
sudo apt upgrade
```
### Install ROS 2 Jazzy (desktop)
```bash
sudo apt install ros-jazzy-desktop
```
### Source ROS 2 setup script
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```


## 🤖 Installation – UR Driver
Installation instructions for the Universal Robots ROS 2 driver are provided in
the official [UR Robot Driver installation documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html).
And will be gone through step by step:
### Physical UR setup
The UR robot needs to be connected over ethernet connection to handle, which can be done directly to a PC or through a 'good' network switch. For this project a network switch was used to with the connections
- UR Robot
- PC
- Router

The use of a network switch with a router instead of direct PC connection was for the router to help with providing IP's for the UR robot and PC, while having the possibility to run parts of the pipeline on seperate PC's. A direct connection between the PC and UR robot would have the need of making a static ip.

### UR driver
The used ROS2 distro for this installation is ROS2 Jazzy and the installation of the UR driver for it is done using
```bash
sudo apt-get install ros-${ROS_DISTRO}-ur
```

To work with the UR driver it is firstly needed action is to ensure the driver has the UR calibration for it to understand the and correctly communicate with the UR robot. This is done by
```bash
ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
source /opt/ros/jazzy/setup.bash

```

## 🔗 Complete Pipeline Installation
For this the Github repo is cloned and build for use
```bash
git clone https://github.com/jens1906/Robotteknologi-5.-semester.git
cd ~/Vattenfall_ws
colcon build 
source install/setup.bash
```

## ▶️ Run the Pipeline

### UR driver 
#### Physical setup
Start by preparring UR robot by using the teach pendent and setup and go and make a program
```
  Instalation - Network - Put in PC IP
  Program - URCaps - External control
```
#### Terminal 1: Setup UR to PC Connection
Go to terminal and open a terminal
```bash
  cd ~/Vattenfall_ws
  source install/setup.bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<ip> use_mock_hardware:=false # set to true for testing without physical UR3e Robot
```
Go to teach pendant and run the previously made progrma on UR teach pendent for connection to PC
If this does not work, the connection might be blocked by Firewall, allow ports access through Firewall or disable it by
```bash 
  sudo ufw disable
```

### Pipeline
#### Terminal 2: Setup Cartisian path follower and MoveIt
Setup the UR integration.
```bash
  cd ~/Vattenfall_ws
  source install/setup.bash
  cd ~/Vattenfall_ws/ur3e_workstation
  ./launch_ur_moveit.sh
```

#### Terminal 3: Run the pipeline
Run Camera, Corrosion, Parameterisation, Pathplanning and Tool integration
```bash
  cd ~/Vattenfall_ws
  source install/setup.bash
  ros2 launch launch_module run_all.launch.py
```

#### Terminal 4: Run User Interface
Run Userinterface
```bash
  cd ~/Vattenfall_ws
  source install/setup.bash
  ros2 run user_interface user_interface_node
```

## 👥 Contributors
This project was developed by a group of students from Aalborg University, as part of the 5th semester in the Robot technologi bachalor.  
- André Vester Magnusson
- Daniel Holst Dreier
- Jens Søby Hansen
- Mads Majlund Thomsen
- Mayvand Basir Hotaki
- Thor Ivarsen Østergaard

## 🔗 References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Intel RealSense ROS2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
