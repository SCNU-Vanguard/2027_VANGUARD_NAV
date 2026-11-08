#!/bin/bash
cd /home/l/ros_ws || exit 1
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
