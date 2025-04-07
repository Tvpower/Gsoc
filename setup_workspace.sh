#!/bin/bash

#source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

#build the workspace with colcon
colcon build --symlink-install

#source the workspace setup
source ~/rfid_ws/install/setup.bash

echo "Workspace has been built and sourced!"
