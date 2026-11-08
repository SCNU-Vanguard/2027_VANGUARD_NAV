#!/usr/bin/env python3
"""对比 IMU 加速度 与 真值速度（MecanumDrive2 的 LinearVelocity），判断尖峰真伪。"""
import glob

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sorted(glob.glob('/home/l/ros_ws/bags/zlio_nav_*'))[-1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
ns = '/red_standard_robot1'
want = {ns + '/livox/imu', ns + '/chassis_odometry_gt'}
buf = {k: [] for k in want}
while r.has_next():
    tp, d, _ = r.read_next()
    if tp in want:
        buf[tp].append(deserialize_message(d, get_message(types[tp])))


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


imu = buf[ns + '/livox/imu']
gt = buf[ns + '/chassis_odometry_gt']
T = np.array([st(m) for m in imu])
T -= T[0]
A = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])

Gt = np.array([st(m) for m in gt])
Gt -= Gt[0]
V = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in gt])
W = np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z] for m in gt])
N = np.linalg.norm(A, axis=1)

print(f'IMU {len(T)}   GT {len(Gt)}')
print(f'GT 线速度: 中位 {np.median(np.linalg.norm(V, axis=1)):.3f}  max {np.linalg.norm(V, axis=1).max():.3f} m/s')
print(f'GT 角速度: max {np.linalg.norm(W, axis=1).max():.3f} rad/s')

print(f"\n{'t':>6}{'|a|max':>9}{'|v|gt':>8}{'vz_gt':>8}   尖峰时刻附近的 GT 速度")
big = np.where(N > 50)[0]
for i in big[:15]:
    t = T[i]
    j = int(np.argmin(np.abs(Gt - t)))
    print(f'{t:6.2f}{N[i]:9.1f}{np.linalg.norm(V[j]):8.3f}{V[j,2]:8.3f}')

# 若尖峰是真的，GT 速度应出现同量级跳变：检查尖峰前后一帧的速度差
print('\n尖峰前后 GT 速度变化（真实动量大时应有明显跳变）:')
dv_list = []
for i in big:
    j = int(np.argmin(np.abs(Gt - T[i])))
    j0 = max(0, j - 1)
    dv_list.append(np.linalg.norm(V[j] - V[j0]))
dv = np.array(dv_list)
print(f'  Δ|v| 中位 {np.median(dv):.4f} m/s   max {dv.max():.4f} m/s')
print(f'  若 |a|=606 m/s² 持续 5 ms，Δv 应为 {606*0.005:.2f} m/s')

# IMU 双积分得到的速度 vs GT 速度（水平）
print('\nIMU 双积分净漂移速率 = 前面已证；这里只看 GT 是否也有对应抖动')
