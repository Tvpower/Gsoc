# Warehouse RFID Simulation Demo

A ROS 2 and Gazebo Harmonic simulation showcasing a mobile robot with RFID sensing capabilities in a warehouse environment.

## Overview

This project demonstrates a warehouse robot equipped with an RFID sensor that can detect tagged objects in a simulated warehouse environment. The simulation includes:

- A four-wheeled mobile robot with differential drive
- Custom RFID sensor plugin (DummySensor)
- Warehouse environment with shelves, pallets, boxes, and other equipment
- ROS 2 integration for robot control and sensor data

## Prerequisites

- Ubuntu 24.04 LTS (Noble Numbat)
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic (Garden)

## Installation

### 1. Install ROS 2 Jazzy

Follow the official ROS 2 Jazzy installation instructions:
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. Install Gazebo Harmonic (Garden)

```bash
# Add Gazebo apt repository
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

### 3. Install ROS 2 - Gazebo Integration Packages

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces
```

### 4. Clone and Build this Repository

```bash
# Source ROS 2 
source /opt/ros/jazzy/setup.bash

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/Tvpower/Gsoc

# Build the workspace
cd ~/ros2_ws
colcon build
```

## Project Structure

```
my_rfid_demo/
├── CMakeLists.txt
├── include/
│   └── my_rfid_demo/
│       └── dummy_sensor.hpp
├── launch/
│   └── warehouse_demo.launch.py
├── package.xml
├── src/
│   └── plugins/
│       └── dummy_sensor.cpp
├── urdf/
│   └── warehouse_robot.urdf.xacro
└── worlds/
    └── warehouse.sdf
```

## Running the Simulation

```bash
# Setup your workspace
/bin/bash /your_path/usr/rfid_ws/setup_workspace.sh


# Launch the simulation
 /bin/bash /your_path/usr/rfid_ws/launch.sh

```

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch the simulation
ros2 launch my_rfid_demo warehouse_demo.launch.py
```

## Demo Features

1. **Mobile Robot**: A four-wheeled robot with differential drive system
2. **RFID Sensor**: Custom DummySensor plugin that publishes RFID tag detections
3. **Warehouse Environment**: Shelves, pallets, boxes and a forklift
4. **ROS 2 Integration**: Topic bridging between Gazebo and ROS 2

## Controlling the Robot

The robot can be controlled using the ROS 2 `cmd_vel` topic:

```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

#Can also use
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

## Monitoring RFID Sensor Output

View the RFID sensor output with:

```bash
ros2 topic echo /dummy_sensor
```
When the robot approaches objects with RFID tags (such as pallet1 and box2), the sensor will detect them and publish their IDs.

## Troubleshooting

### Plugin Not Loading

If the DummySensor plugin fails to load, check:

1. Ensure the plugin library was correctly built:
```bash
ls -l ~/ros2_ws/install/my_rfid_demo/lib/libdummy_sensor.so
```

2. Verify the plugin references in the URDF and SDF files match the library name

### Simulation Not Starting

If the simulation fails to start:

1. Check that all dependencies are installed:
```bash
rosdep install --from-paths ~/ros2_ws/src --ignore-src -r -y
```

2. Ensure the world file paths are correct in the launch file

3. Check the Gazebo logs for specific errors:
```bash
gz log
```

## Modifying the Simulation

### Adding More RFID Tags

Edit the `dummy_sensor.cpp` file to add more tags to detect:

```cpp
// Within the Update function:
if (current_time % 5000000000 < 2000000000) {
    tags.emplace_back("pallet1:RFID:12345");
} else if (current_time % 7000000000 < 3000000000) {
    tags.emplace_back("box2:RFID:67890");
} 
// Add more conditions here for additional tags
```

### Customizing the Robot

The robot model is defined in `urdf/warehouse_robot.urdf.xacro`. You can modify this file to change the robot's appearance and properties.


## Author

Tvpower