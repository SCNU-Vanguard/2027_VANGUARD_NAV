#!/bin/bash
# 记录 + 发真实导航目标，复现"点导航点后崩"
cd /home/l/ros_ws || exit 1
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null

NS=/red_standard_robot1
OUT=bags/zlio_nav_$(date +%H%M%S)
mkdir -p bags .tmp_run

echo "=== 录制 -> $OUT ==="
ros2 bag record -o "$OUT" \
  "$NS/livox/imu" "$NS/velodyne_points" \
  "$NS/lidar_odometry" "$NS/registered_scan" "$NS/chassis_odometry_gt" \
  "$NS/backend_node/localization_state" "$NS/backend_node/localization_pose" \
  "$NS/cmd_vel" /tf /clock \
  > .tmp_run/record2.log 2>&1 &
REC=$!

sleep 4
echo "=== 发送导航目标 (4.0, 0.0) ==="
timeout 25 ros2 action send_goal /red_standard_robot1/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > .tmp_run/nav.log 2>&1 &
NAV=$!

sleep 22
kill -INT $NAV 2>/dev/null
kill -INT $REC 2>/dev/null
wait $REC 2>/dev/null
echo "=== 完成 $OUT ==="
du -sh "$OUT" 2>/dev/null
echo "=== 导航结果 ==="
tail -12 .tmp_run/nav.log
