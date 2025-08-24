# T-Robotics AMR ROS 2 Software Stack

This repository contains the official ROS 2 software stack for the Autonomous Mobile Robot (AMR) platform developed by T-Robotics.

## Overview
This repository contains ROS 2 packages for implementing autonomous navigation and control on the AMR platform developed by T-Robotics.  
This work was conducted as part of the T-Robotics Industry-University Cooperation Project.

## Dependencies
- ROS 2 Humble: Please follow the official installation instructions.
- Required ROS 2 Packages:
```bash
sudo apt-get update && sudo apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-rclcpp-action \
    ros-humble-rclcpp-lifecycle \
    ros-humble-rclcpp-components \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-common \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-plugins
    ros-humble-xacro \
    ros-humble-diagnostic-updater \
    ros-humble-sick-safetyscanners2 \
    ros-humble-robot-localization
```
- Lely CANopen:
```bash
sudo add-apt-repository ppa:lely/ppa
sudo apt-get update
sudo apt-get install liblely-coapp-dev liblely-co-tools python3-dcf-tools
sudo apt-get install pkg-config
sudo apt-get install can-utils
```
- Other System Libraries:
```bash
sudo apt install -y \
    python3-pytest \
    python3-colcon-common-extensions \
    libboost-dev \
    nlohmann-json3-dev \
    libserial-dev \
    xterm
```

## Installation
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RCILab/RCI_mobile_navigation_tr.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage
```bash
ros2 launch tr_real bringup_real.launch.py
```

## Examples
1. **Simulation** – Gazebo simulation with the AMR
   ```bash
   ros2 launch tr_sim simulation.launch.py
   ```
2. **Mapping**
   ```bash
   ros2 launch tr_nav2_bringup mapping.launch.py
   ros2 launch tr_nav2_bringup rviz_launch.py
   ros2 run nav2_map_server map_saver_cli -f ~/tr_ws/src/tr_real/maps/<map_name>
   ```
3. **Navigation**
   ```bash
   ros2 launch tr_real tr_navigation2.launch.py map_name:='<map_name>'
   ```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact
Maintainer: Daum Park (doumpork@khu.ac.kr)  
Lab: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)
