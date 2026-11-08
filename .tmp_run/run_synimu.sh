#!/bin/bash
cd /home/l/ros_ws || exit 1
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42
/usr/bin/python3 .tmp_run/replay_synimu.py 2>&1 | grep -vE "^\[INFO\]|^\[WARN\]"
