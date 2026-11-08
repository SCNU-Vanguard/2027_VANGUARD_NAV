#!/usr/bin/env python3
"""离线回放：把包里的 IMU + 点云喂给 ZLIO，对比输出里程计与真值。

用法：replay_test.py [raw|guarded]
  raw      —— 原始 IMU，复现发散
  guarded  —— 经 imu_guard 限幅后的 IMU
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
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, PointCloud2

MODE = sys.argv[1] if len(sys.argv) > 1 else 'raw'
NS = '/replay'
BAG = sorted(glob.glob('/home/l/ros_ws/bags/zlio_nav_*'))[-1]
CFG = '/home/l/ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/zlio_sim.yaml'
SRC = '/red_standard_robot1'

print(f'bag={BAG}  mode={MODE}')

# ---------------- 读包 ----------------
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', ''),
)
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
keep = {SRC + '/livox/imu', SRC + '/velodyne_points', SRC + '/chassis_odometry_gt'}
msgs = []
gt = []
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic not in keep:
        continue
    if topic.endswith('chassis_odometry_gt'):
        gt.append(deserialize_message(data, get_message(types[topic])))
    else:
        msgs.append((t, topic, data))
msgs.sort(key=lambda x: x[0])
t0 = msgs[0][0]
span = (msgs[-1][0] - t0) * 1e-9
print(f'回放 {len(msgs)} 条消息，跨度 {span:.1f}s')


def stamp_of(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


GT_T = np.array([stamp_of(m) for m in gt])
GT_P = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in gt])
GT_Q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                  m.pose.pose.orientation.z, m.pose.pose.orientation.w] for m in gt])


# ---------------- 工具 ----------------
def quat_angle(q):
    w = max(-1.0, min(1.0, abs(q[3])))
    return 2.0 * math.acos(w)


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


# ---------------- ROS ----------------
rclpy.init()
node = Node('replay_harness')
pub_imu = node.create_publisher(Imu, NS + '/imu_raw', 100)
pub_scan = node.create_publisher(PointCloud2, NS + '/scan', 100)

collected = []
lock = threading.Lock()


def on_odom(msg: Odometry):
    with lock:
        collected.append((stamp_of(msg),
                          np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]),
                          np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])))


node.create_subscription(Odometry, NS + '/lidar_odometry', on_odom, 200)
node.create_subscription(Odometry, NS + '/frontend/odometry', on_odom, 200)

spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin.start()

# ---------------- 子进程 ----------------
procs = []
imu_topic = NS + '/imu_raw'
if MODE == 'guarded':
    guard = subprocess.Popen([
        '/usr/bin/python3', '/home/l/ros_ws/install/zloc_bridge/lib/zloc_bridge/imu_guard.py',
        '--ros-args', '-r', f'__ns:={NS}',
        '-p', f'input_topic:={NS}/imu_raw',
        '-p', f'output_topic:={NS}/imu_guarded',
        '-p', 'jump_threshold:=20.0',
        '-p', 'max_acceleration:=60.0',
        '-p', 'log_period:=100.0',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs.append(guard)
    imu_topic = NS + '/imu_guarded'

zlio = subprocess.Popen([
    'ros2', 'run', 'frontend', 'zlio_node', '--ros-args',
    '-r', f'__ns:={NS}', '-p', 'use_sim_time:=false',
    '-p', f'config_file:={CFG}',
    '-p', 'input_type:=pointcloud2',
    '-p', f'lidar_topic:={NS}/scan',
    '-p', f'imu_topic:={imu_topic}',
    '-p', 'point_time_field:=time',
    '-p', 'point_time_scale:=0.01',
    '-p', 'point_time_absolute:=false',
    '-p', 'imu_acceleration_scale:=1.0',
    '-p', 'publish_tf:=false',
    '-p', 'world_frame:=odom',
    '-p', 'body_frame:=front_mid360',
    '-p', 'output_resolution:=0.20',
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
procs.append(zlio)
time.sleep(3.0)

# ---------------- 回放 ----------------
print('开始回放...')
start_wall = time.time()
for t, topic, raw in msgs:
    dt_target = (t - t0) * 1e-9
    lag = dt_target - (time.time() - start_wall)
    if lag > 0:
        time.sleep(min(lag, 0.05))
    if topic.endswith('livox/imu'):
        pub_imu.publish(deserialize_message(raw, Imu))
    else:
        pub_scan.publish(deserialize_message(raw, PointCloud2))
time.sleep(2.0)

for p in procs:
    p.terminate()
for p in procs:
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill()

# ---------------- 评估 ----------------
with lock:
    od = sorted(collected, key=lambda x: x[0])
print(f'\n收到 ZLIO 里程计 {len(od)} 帧')
if len(od) < 10:
    print('!! 里程计太少，ZLIO 可能没初始化')
    sys.exit(1)

print(f"\n{'t':>6} {'odom|d|':>9} {'gt|d|':>8} {'odom_z':>8} {'转角差':>8}")
ref = None
max_err = 0.0
for k, (ts, p, q) in enumerate(od):
    j = int(np.argmin(np.abs(GT_T - ts)))
    if ref is None:
        ref = (ts, p, q, j)
        continue
    if ts - ref[0] < 1.0:
        continue
    d_od = np.linalg.norm(p - ref[1])
    d_gt = np.linalg.norm(GT_P[j] - GT_P[ref[3]])
    dq = quat_mul(q_conj(ref[2]), q)
    # 相对旋转角（在各自坐标系内是标量不变量）
    ang_diff = abs(quat_angle(dq) - quat_angle(quat_mul(q_conj(GT_Q[ref[3]]), GT_Q[j])))
    err = abs(d_od - d_gt)
    max_err = max(max_err, err)
    if k % 4 == 0:
        print(f'{ts - od[0][0]:6.1f} {d_od * 100:9.1f} {d_gt * 100:8.1f} {p[2]:8.2f} {ang_diff:8.2f}')

print(f'\n=== {MODE}: 相对位移最大偏差 {max_err * 100:.1f} cm ===')

# 导出轨迹，便于逐帧看是不是一次性跳变
with open(f'/home/l/ros_ws/.tmp_run/traj_{MODE}.csv', 'w') as f:
    f.write('t,ox,oy,oz,gx,gy,gz\n')
    for ts, p, q in od:
        j = int(np.argmin(np.abs(GT_T - ts)))
        f.write(f'{ts - od[0][0]:.3f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},'
                f'{GT_P[j][0]:.4f},{GT_P[j][1]:.4f},{GT_P[j][2]:.4f}\n')
print('轨迹已写入 .tmp_run/traj_%s.csv' % MODE)

rclpy.try_shutdown()
