#!/bin/bash
# 导航复现测试：录包 + 发真实导航目标 + 记录状态变化
cd /home/l/ros_ws || exit 1
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
source /opt/ros/humble/setup.bash
source install/setup.bash

NS=/red_standard_robot1
OUT=bags/zlio_fix_$(date +%H%M%S)
mkdir -p bags .tmp_run

echo "=== 录制 -> $OUT ==="
ros2 bag record -o "$OUT" \
  "$NS/livox/imu" "$NS/velodyne_points" \
  "$NS/lidar_odometry" "$NS/registered_scan" "$NS/chassis_odometry_gt" \
  "$NS/backend_node/localization_state" "$NS/backend_node/localization_pose" \
  "$NS/cmd_vel" /clock \
  > .tmp_run/record_fix.log 2>&1 &
REC=$!
sleep 4

echo "=== 目标 1: (4.0, 0.0) ==="
timeout 20 ros2 action send_goal /red_standard_robot1/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > .tmp_run/nav1.log 2>&1
sleep 2

echo "=== 目标 2: (0.5, 2.5) ==="
timeout 18 ros2 action send_goal /red_standard_robot1/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 2.5, z: 0.0}, orientation: {w: 1.0}}}}" \
  > .tmp_run/nav2.log 2>&1

sleep 3
kill -INT $REC 2>/dev/null
wait $REC 2>/dev/null
echo "=== 完成 $OUT ==="
du -sh "$OUT" 2>/dev/null
tail -4 .tmp_run/nav1.log
tail -4 .tmp_run/nav2.log
