#!/bin/bash
# 单次调用内完成：起 Gazebo → 起导航 → 录包 + 发导航目标 → 汇总
cd /home/l/ros_ws || exit 1
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

NS=/red_standard_robot1
rm -rf bags/zlio_final; mkdir -p bags

echo "=== 1. 启动 Gazebo ==="
setsid nohup .tmp_run/run_gz.sh > .tmp_run/gz2.log 2>&1 < /dev/null &
for i in $(seq 1 30); do
  sleep 2
  n=$(ros2 topic list 2>/dev/null | grep -c "livox/lidar")
  [ "$n" -ge 1 ] && break
done
echo "lidar 话题数=$n"

echo "=== 2. 启动导航栈 ==="
setsid nohup .tmp_run/run_nav_stack.sh > .tmp_run/nav2.log 2>&1 < /dev/null &
for i in $(seq 1 25); do
  sleep 2
  grep -q "loaded validated map" .tmp_run/nav2.log 2>/dev/null && break
done
sleep 12
echo "imu_guard 模式："
grep -o "IMU 模式 [^：]*" .tmp_run/nav2.log 2>/dev/null | tail -1
grep -c "process has died" .tmp_run/nav2.log 2>/dev/null

echo "=== 3. 录包 + 导航 ==="
ros2 bag record -o bags/zlio_final \
  "$NS/livox/imu" "$NS/velodyne_points" "$NS/lidar_odometry" \
  "$NS/chassis_odometry_gt" "$NS/backend_node/localization_state" /clock \
  > .tmp_run/record3.log 2>&1 &
REC=$!
sleep 5
timeout 18 ros2 action send_goal "$NS/navigate_to_pose" nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}" \
  > .tmp_run/nav3.log 2>&1
sleep 2
kill -INT $REC 2>/dev/null
sleep 4
du -sh bags/zlio_final 2>/dev/null

echo "=== 4. 后端状态汇总 ==="
grep -E "state ->" .tmp_run/nav2.log 2>/dev/null | sed -E 's/.*state -> //' | cut -c1-80 | tail -12
echo "--- conflict 次数 ---"
grep -c "correction conflict" .tmp_run/nav2.log 2>/dev/null
echo "--- 桥接真值对比（最后 8 条）---"
for f in $(ls -t ~/.ros/log/python3_*.log 2>/dev/null | head -6); do
  if grep -q "前端 vs 真值" "$f"; then grep "前端 vs 真值" "$f" | tail -8; break; fi
done
