#!/bin/bash
# 采集 20 秒定位数据，供离线复现（ZLIO 前端 / 后端都能在包上重跑）
#
# 用法：
#   1) 正常启动仿真（use_zloc:=True use_zlio:=True），等它锁定
#   2) 另开一个终端跑本脚本：bash tools/capture_zlio_bag.sh
#   3) 看到 "开始录制" 后，在 RViz 里点一个导航点，让车动起来
#   4) 20 秒后自动停止，包在 bags/zlio_<时间>/ 下
#
# 采的话题刻意很少，够用且不至于太大（20 s 约 300~400 MB）：
#   /livox/imu            原始 IMU（含重力，m/s²）——验证加速度单位/符号/频率
#   /livox/lidar          gz 原始点云（无 time 字段）——验证"瞬时快照"这条判断
#   /velodyne_points      转换后的点云（含伪造 time 字段）——验证逐点时间跨度
#   /lidar_odometry       前端输出的里程计
#   /registered_scan      桥接输出（odom 系点云）
#   /chassis_odometry_gt  真值位姿（MecanumDrive2 直接读 WorldPose）
#   /tf /tf_static        完整坐标链
#   /clock                仿真时钟

set -u
NS=/red_standard_robot1
DURATION=${1:-20}
OUT="bags/zlio_$(date +%H%M%S)"

cd "$(dirname "$0")/.." || exit 1
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
mkdir -p bags

echo "录制 ${DURATION}s -> ${OUT}"
echo "现在去 RViz 点一个导航点。"
sleep 3

timeout "$DURATION" ros2 bag record -o "$OUT" \
  --topics \
  "$NS/livox/imu" \
  "$NS/livox/lidar" \
  "$NS/velodyne_points" \
  "$NS/lidar_odometry" \
  "$NS/registered_scan" \
  "$NS/chassis_odometry_gt" \
  /tf /tf_static /clock

echo
echo "完成：$OUT"
du -sh "$OUT" 2>/dev/null
echo "把整个目录发给我（或告诉我路径）即可。"
