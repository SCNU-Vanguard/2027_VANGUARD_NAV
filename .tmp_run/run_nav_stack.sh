#!/bin/bash
# 导航栈启动包装（由 systemd-run --user 托管，脱离工具调用生命周期）
cd /home/l/ros_ws || exit 1
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
exec ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True use_zlio:=True
