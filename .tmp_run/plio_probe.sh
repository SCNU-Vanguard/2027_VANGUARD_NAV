#!/bin/bash
cd /home/l/ros_ws || exit 1
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42
timeout 8 ros2 run point_lio pointlio_mapping --ros-args -r __ns:=/plio -r __node:=point_lio \
  --params-file src/pb2025_sentry_nav/point_lio/config/mid360.yaml \
  -p lid_topic:=/plio/livox/lidar -p imu_topic:=/plio/livox/imu -p publish_tf:=false 2>&1 | tail -20
