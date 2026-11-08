#!/usr/bin/env python3
"""把同一段包喂给 point_lio，测量它在同一个坏 IMU 下漂多少（对照组）。"""
import glob
import math
import subprocess
import threading
import time

import numpy as np
import rosbag2_py
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, PointCloud2

NS = '/plio'
BAG = sorted(glob.glob('/home/l/ros_ws/bags/zlio_fix_*'))[-1]
CFG = '/home/l/ros_ws/src/pb2025_sentry_nav/point_lio/config/mid360.yaml'
SRC = '/red_standard_robot1'
print('bag:', BAG)

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msgs, gt = [], []
while reader.has_next():
    tp, d, t = reader.read_next()
    if tp == SRC + '/velodyne_points':
        msgs.append((t, 'scan', d))
    elif tp == SRC + '/livox/imu':
        msgs.append((t, 'imu', d))
    elif tp == SRC + '/chassis_odometry_gt':
        gt.append(deserialize_message(d, get_message(types[tp])))
msgs.sort(key=lambda x: x[0])


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


TG = np.array([st(m) for m in gt])
PG = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in gt])
print(f'{len(msgs)} 条消息，gt {len(gt)}')

rclpy.init()
node = Node('plio_replay')
pub_imu = node.create_publisher(Imu, NS + '/livox/imu', 200)
pub_scan = node.create_publisher(PointCloud2, NS + '/livox/lidar', 100)
od = []
lock = threading.Lock()


def on_odom(msg):
    with lock:
        od.append((st(msg), np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])))


node.create_subscription(Odometry, NS + '/aft_mapped_to_init', on_odom, 200)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

proc = subprocess.Popen([
    'ros2', 'run', 'point_lio', 'pointlio_mapping', '--ros-args',
    '-r', f'__ns:={NS}', '-r', '__node:=point_lio',
    '--params-file', CFG,
    '-p', 'use_sim_time:=false',
    '-p', f'lid_topic:={NS}/livox/lidar',
    '-p', f'imu_topic:={NS}/livox/imu',
    '-p', 'publish_tf:=false',
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(4.0)

t0 = msgs[0][0]
start = time.time()
for t, kind, raw in msgs:
    lag = (t - t0) * 1e-9 - (time.time() - start)
    if lag > 0:
        time.sleep(min(lag, 0.05))
    if kind == 'imu':
        pub_imu.publish(deserialize_message(raw, Imu))
    else:
        pub_scan.publish(deserialize_message(raw, PointCloud2))
time.sleep(2.0)
proc.terminate()
try:
    proc.wait(timeout=5)
except subprocess.TimeoutExpired:
    proc.kill()

with lock:
    o = sorted(od, key=lambda x: x[0])
print(f'\npoint_lio 输出 {len(o)} 帧')
if len(o) < 10:
    print('!! point_lio 没有输出，检查参数')
else:
    print(f"{'t':>6}{'odom_d':>9}{'gt_d':>8}{'odom_z':>9}")
    mx = 0.0
    r0_od = o[0][1]
    j0 = int(np.argmin(np.abs(TG - o[0][0])))
    for k in range(0, len(o), max(1, len(o) // 14)):
        ts, p = o[k]
        j = int(np.argmin(np.abs(TG - ts)))
        d_od = np.linalg.norm(p - r0_od)
        d_gt = np.linalg.norm(PG[j] - PG[j0])
        mx = max(mx, abs(d_od - d_gt))
        print(f'{ts - o[0][0]:6.1f}{d_od*100:9.1f}{d_gt*100:8.1f}{p[2]:9.2f}')
    print(f'\n=== point_lio: 相对位移最大偏差 {mx*100:.1f} cm ===')
rclpy.try_shutdown()
