#!/usr/bin/env python3
"""诊断：IMU 加速度模长在运动中是否偏离 9.8（重力补偿失效的判据）。"""
import glob

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sorted(glob.glob('/home/l/ros_ws/bags/zlio_fix_*'))[-1]
print('bag:', bag)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
types = {x.name: x.type for x in r.get_all_topics_and_types()}
k = '/red_standard_robot1/livox/imu'
A = []
while r.has_next():
    tp, d, _ = r.read_next()
    if tp == k:
        A.append(deserialize_message(d, get_message(types[tp])))

a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in A])
T = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in A])
T -= T[0]
N = np.linalg.norm(a, axis=1)

print('窗口(s)  中位|a|  剔除>50后中位  z中位    样本')
for q in np.arange(0, T[-1], 2.0):
    m = (T >= q) & (T < q + 2)
    if m.sum() < 5:
        continue
    ok = N[m] < 50
    med = np.median(N[m])
    med2 = np.median(N[m][ok]) if ok.sum() else float('nan')
    zmed = np.median(a[m, 2][ok]) if ok.sum() else float('nan')
    print(f'{q:6.1f}  {med:8.2f}  {med2:13.2f}  {zmed:7.2f}  {m.sum():5d}')
