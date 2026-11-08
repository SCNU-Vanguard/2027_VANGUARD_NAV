#!/usr/bin/env python3
"""离线验证：把「物理自洽的合成 IMU」喂给 ZLIO，看平移是否还发散。

合成 IMU 用真值（chassis_odometry_gt）重算：
  ω  = gt 的角速度
  a  = R_imu^T · (dv_world/dt + [0,0,9.81])      （IMU 测的是比力，含重力反作用）
  q  = R_chassis · Rx(-15°)                       （雷达/IMU 安装横滚）
这样得到的是"真机 IMU 应该给出的信号"（含 gz 缺失的抗混叠），用它可以把
「ZLIO 本身的精度」与「仿真 IMU 的缺陷」分离开。
"""
import glob
import math
import os
import subprocess
import sys
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

BAG = sorted(glob.glob('/home/l/ros_ws/bags/zlio_fix_*'))[-1]
NS = '/replay'
CFG = '/home/l/ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/zlio_sim.yaml'
SRC = '/red_standard_robot1'
RATE = 200.0

print('bag:', BAG)
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
scan_raw = []
gt = []
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == SRC + '/velodyne_points':
        scan_raw.append(data)
    elif topic == SRC + '/chassis_odometry_gt':
        gt.append(deserialize_message(data, get_message(types[topic])))


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


TG = np.array([st(m) for m in gt])
PG = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in gt])
QG = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                m.pose.pose.orientation.z, m.pose.pose.orientation.w] for m in gt])
VG = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in gt])
WG = np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z] for m in gt])
print(f'gt {len(gt)} @ {1/np.median(np.diff(TG)):.0f} Hz, scans {len(scan_raw)}')


def q2R(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - (y * y + z * z) * s, (x * y - w * z) * s, (x * z + w * y) * s],
        [(x * y + w * z) * s, 1 - (x * x + z * z) * s, (y * z - w * x) * s],
        [(x * z - w * y) * s, (y * z + w * x) * s, 1 - (x * x + y * y) * s],
    ])


R_MOUNT = np.array([
    [1, 0, 0],
    [0, math.cos(math.radians(-15)), -math.sin(math.radians(-15))],
    [0, math.sin(math.radians(-15)), math.cos(math.radians(-15))],
])

# 世界系速度与加速度
Vw = np.array([q2R(QG[i]) @ VG[i] for i in range(len(gt))])
Aw = np.gradient(Vw, axis=0) / np.gradient(TG)[:, None]

# 组装合成 IMU
imu_msgs = []
for i in range(0, len(gt), max(1, int(round(1 / np.median(np.diff(TG)) / RATE)))):
    R = q2R(QG[i]) @ R_MOUNT
    f = R.T @ (Aw[i] + np.array([0.0, 0.0, 9.81]))
    t = TG[i]
    imu_msgs.append((t, R, WG[i], f))
print(f'合成 IMU {len(imu_msgs)} 帧')

# a 的统计，确认没有伪迹
FA = np.array([m[3] for m in imu_msgs])
FN = np.linalg.norm(FA, axis=1)
print(f'合成 |a|: 中位 {np.median(FN):.2f}  min {FN.min():.2f}  max {FN.max():.2f}')
print(f'合成相邻帧跳变: 中位 {np.median(np.linalg.norm(np.diff(FA, axis=0), axis=1)):.3f} '
      f'max {np.linalg.norm(np.diff(FA, axis=0), axis=1).max():.1f}')

rclpy.init()
node = Node('syn_replay')
pub_imu = node.create_publisher(Imu, NS + '/imu_syn', 100)
pub_scan = node.create_publisher(PointCloud2, NS + '/scan', 100)
collected = []
lock = threading.Lock()


def on_odom(msg):
    with lock:
        collected.append((st(msg),
                          np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]),
                          np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])))


node.create_subscription(Odometry, NS + '/lidar_odometry', on_odom, 200)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

zlio = subprocess.Popen([
    'ros2', 'run', 'frontend', 'zlio_node', '--ros-args',
    '-r', f'__ns:={NS}', '-p', 'use_sim_time:=false',
    '-p', f'config_file:={CFG}',
    '-p', 'input_type:=pointcloud2',
    '-p', f'lidar_topic:={NS}/scan',
    '-p', f'imu_topic:={NS}/imu_syn',
    '-p', 'point_time_field:=time',
    '-p', 'point_time_scale:=0.01',
    '-p', 'point_time_absolute:=false',
    '-p', 'imu_acceleration_scale:=1.0',
    '-p', 'publish_tf:=false',
    '-p', 'world_frame:=odom',
    '-p', 'body_frame:=front_mid360',
    '-p', 'output_resolution:=0.20',
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3.0)

# 按时间交错回放
scans = [(st(deserialize_message(d, PointCloud2)), d) for d in scan_raw]
events = [(m[0], 'imu', m) for m in imu_msgs] + [(t, 'scan', d) for t, d in scans]
events.sort(key=lambda e: e[0])
t0 = events[0][0]
start = time.time()
for t, kind, payload in events:
    lag = (t - t0) - (time.time() - start)
    if lag > 0:
        time.sleep(min(lag, 0.02))
    if kind == 'imu':
        _, R, w, f = payload
        m = Imu()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int((t - int(t)) * 1e9)
        m.header.frame_id = 'front_mid360'
        # 由 R 反推四元数（这里只用于填充，ZLIO 用 gyro 积分）
        tr = np.trace(R)
        wq = math.sqrt(max(0.0, 1 + tr)) / 2
        m.orientation.w = wq
        m.orientation.x = (R[2, 1] - R[1, 2]) / (4 * wq) if wq > 1e-6 else 0.0
        m.orientation.y = (R[0, 2] - R[2, 0]) / (4 * wq) if wq > 1e-6 else 0.0
        m.orientation.z = (R[1, 0] - R[0, 1]) / (4 * wq) if wq > 1e-6 else 0.0
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = f
        pub_imu.publish(m)
    else:
        pub_scan.publish(deserialize_message(payload, PointCloud2))
time.sleep(2.0)
zlio.terminate()
try:
    zlio.wait(timeout=5)
except subprocess.TimeoutExpired:
    zlio.kill()

with lock:
    od = sorted(collected, key=lambda x: x[0])
print(f'\n收到 ZLIO 里程计 {len(od)} 帧')
if len(od) < 10:
    sys.exit(1)
print(f"{'t':>6}{'odom_d':>9}{'gt_d':>8}{'odom_z':>9}")
max_err = 0.0
for k in range(0, len(od), max(1, len(od) // 14)):
    ts, p, q = od[k]
    j = int(np.argmin(np.abs(TG - ts)))
    d_od = np.linalg.norm(p - od[0][1])
    d_gt = np.linalg.norm(PG[j] - PG[int(np.argmin(np.abs(TG - od[0][0])))])
    max_err = max(max_err, abs(d_od - d_gt))
    print(f'{ts - od[0][0]:6.1f}{d_od*100:9.1f}{d_gt*100:8.1f}{p[2]:9.2f}')
print(f'\n=== 合成 IMU: 相对位移最大偏差 {max_err*100:.1f} cm ===')
rclpy.try_shutdown()
