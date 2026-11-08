#!/bin/bash
# 回放测试的启动包装：统一环境，避免长命令行
cd /home/l/ros_ws || exit 1
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42
/usr/bin/python3 .tmp_run/replay_test.py "${1:-raw}" 2>&1 | grep -vE "^\[INFO\]|^\[WARN\]"
