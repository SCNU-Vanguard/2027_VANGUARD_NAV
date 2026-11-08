#!/bin/bash
# 一边录包一边用 cmd_vel 驱车，复现"运动后定位崩"

cd /home/l/ros_ws || exit 1
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null

NS=/red_standard_robot1
OUT=bags/zlio_run_$(date +%H%M%S)
mkdir -p bags .tmp_run

echo "=== 录制 -> $OUT ==="
ros2 bag record -o "$OUT" \
  "$NS/livox/imu" "$NS/livox/lidar" "$NS/velodyne_points" \
  "$NS/lidar_odometry" "$NS/registered_scan" "$NS/chassis_odometry_gt" \
  "$NS/backend_node/localization_state" "$NS/backend_node/localization_pose" \
  "$NS/frontend/frame" /tf /tf_static /clock \
  > .tmp_run/record.log 2>&1 &
REC=$!

sleep 4
echo "=== 开始驱车 ==="

drive() {  # $1=时长 $2=线速度 $3=角速度
  timeout "$1" ros2 topic pub -r 20 "$NS/cmd_vel" geometry_msgs/msg/Twist \
    "{linear: {x: $2}, angular: {z: $3}}" > /dev/null 2>&1
}

echo "[1] 原地右转 1.5 rad/s 3s";  drive 3 0.0 1.5
sleep 1
echo "[2] 直行 0.6 m/s 4s";        drive 4 0.6 0.0
sleep 1
echo "[3] 原地左转 -1.5 rad/s 3s"; drive 3 0.0 -1.5
sleep 1
echo "[4] 直行 0.8 m/s 3s";        drive 3 0.8 0.0
sleep 1
echo "[5] 边走边转 2s";            drive 2 0.5 1.0
drive 1 0.0 0.0

sleep 2
kill -INT $REC 2>/dev/null
wait $REC 2>/dev/null
echo "=== 完成 $OUT ==="
du -sh "$OUT"
