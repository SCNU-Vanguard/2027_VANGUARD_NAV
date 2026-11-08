#!/bin/bash
# 重启导航栈：关键是让 python3 解析到 /usr/bin/python3（ROS 的 rclpy/numpy 在这里），
# 否则 launch 里的 python 节点会因 ModuleNotFoundError: numpy 全部退出。
cd /home/l/ros_ws || exit 1

# 先停掉所有既有 launch 与残留节点
for p in $(pgrep -f "rm_navigation_simulatio[n]"); do kill -INT "$p" 2>/dev/null; done
sleep 4
for p in $(pgrep -f "rm_navigation_simulatio[n]"); do kill -TERM "$p" 2>/dev/null; done
pkill -x zlio_node 2>/dev/null
pkill -x backend_node 2>/dev/null
sleep 2

export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

which python3
python3 -c "import numpy; print('numpy ok')"

setsid nohup ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True use_zlio:=True \
  < /dev/null > .tmp_run/nav_launch.log 2>&1 &
sleep 22
echo "--- 关键进程 ---"
pgrep -x zlio_node | head -2
pgrep -x backend_node | head -2
pgrep -f "imu_guar[d]" | head -2
echo "--- 死亡记录 ---"
grep -E "process has died" .tmp_run/nav_launch.log | cut -c1-100
