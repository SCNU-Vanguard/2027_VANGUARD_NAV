#!/bin/bash
# 重启导航栈（Gazebo 是独立进程，不受影响），让 imu_guard 生效
cd /home/l/ros_ws || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

OLD=$(pgrep -f "ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py" | head -1)
echo "旧 launch pid: ${OLD:-无}"
if [ -n "$OLD" ]; then
  kill -INT "$OLD" 2>/dev/null
  for _ in $(seq 1 20); do
    kill -0 "$OLD" 2>/dev/null || break
    sleep 0.5
  done
  kill -0 "$OLD" 2>/dev/null && kill -TERM "$OLD" 2>/dev/null
  echo "旧 launch 已停止"
fi
sleep 3

setsid nohup ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True use_zlio:=True \
  > .tmp_run/nav_launch.log 2>&1 &
echo "新 launch 已启动，pid $!"
sleep 8
grep -cE "process started" .tmp_run/nav_launch.log
