#!/usr/bin/env bash
# 一键停掉仿真 + 导航侧所有进程。
#
# 为什么需要这个脚本：中断过的 launch（Ctrl+C、超时、崩溃）会留下孤儿进程。
# 残留的 component_container_isolated 会占住 nav2_container 这个名字，
# 导致下一次 launch 的 LoadComposableNodes 加载失败——表现为 nav2、loam_interface、
# sensor_scan_generation、map_server 全都不启动，RViz 里看不到机器人。
# 残留的 ign_sim_pointcloud_tool 会让 livox/lidar 出现多个发布者，
# point_lio 随即刷 "lidar loop back, clear buffer"，里程计彻底失效。
#
# 用法：
#   ./stop_all.sh          # 停导航 + 仿真
#   ./stop_all.sh nav      # 只停导航侧，保留 Gazebo

set -u

stop_nav() {
  echo "[stop_all] 停止导航侧进程..."
  pkill -f component_container_isolated 2>/dev/null
  pkill -f pointlio_mapping 2>/dev/null
  pkill -f backend_node 2>/dev/null
  pkill -f map_odom_fallback 2>/dev/null
  pkill -f fake_lio_publisher 2>/dev/null
  pkill -f terrainAnalysis 2>/dev/null
  pkill -f ign_sim_pointcloud_tool 2>/dev/null
  pkill -f pb_teleop_twist_joy 2>/dev/null
  pkill -f joy_node 2>/dev/null
  pkill -f rviz2 2>/dev/null
  pkill -f "ros2 launch pb2025_nav_bringup" 2>/dev/null
}

stop_sim() {
  echo "[stop_all] 停止 Gazebo 与桥接..."
  pkill -f "ign gazebo" 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  pkill -f "ros2 launch rmu_gazebo_simulator" 2>/dev/null
}

case "${1:-all}" in
  nav) stop_nav ;;
  all) stop_nav; sleep 1; stop_sim ;;
  *) echo "用法: $0 [all|nav]"; exit 1 ;;
esac

sleep 2
left=$(ps -eo comm 2>/dev/null | grep -cE "component_cont|pointlio|backend_node|ign_sim_pointcl|rviz2|terrainAnal")
echo "[stop_all] 剩余相关进程: $left"
if [ "$left" -ne 0 ]; then
  echo "[stop_all] 仍有残留，请检查：ps -eo pid,etime,comm | grep -E 'component_cont|pointlio|backend|ign_sim'"
  exit 1
fi
echo "[stop_all] 已清理干净"
