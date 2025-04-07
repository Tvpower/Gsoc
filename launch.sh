#!/bin/bash
#switch into main directory
cd ~/rfid_ws

#build with colcon
colcon build --symlink-install

#source the new build
source install/setup.bash

#launch world
ros2 launch my_rfid_demo warehouse_demo.launch.py