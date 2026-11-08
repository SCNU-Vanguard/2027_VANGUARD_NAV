#!/usr/bin/env python3
"""从包里提取 IMU / ZLIO 里程计 / 真值，检查加速度与平移发散。"""
import glob
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob('/home/l/ros_ws/bags/zlio_nav_*'))[-1]
print("bag:", bag)

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', ''),
)
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msgs = {k: [] for k in types}


def to_ns(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


while reader.has_next():
    topic, data, t = reader.read_next()
    if topic not in types:
        continue
    m = deserialize_message(data, get_message(types[topic]))
    msgs[topic].append((t, m))

ns = '/red_standard_robot1'
imu = msgs[ns + '/livox/imu']
odom = msgs[ns + '/lidar_odometry']
gt = msgs[ns + '/chassis_odometry_gt']
print(f"IMU {len(imu)}  ODOM {len(odom)}  GT {len(gt)}")

# ---------- IMU 基本性质 ----------
acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])
gyr = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])
t_imu = np.array([to_ns(m.header.stamp) for _, m in imu])
dt = np.diff(t_imu)
print(f"\nIMU 频率 {1/np.median(dt):.1f} Hz, dt 中位 {np.median(dt)*1000:.1f} ms, 最大 {dt.max()*1000:.1f} ms")
print(f"加速度模长: 中位 {np.median(np.linalg.norm(acc,axis=1)):.3f}  min {np.linalg.norm(acc,axis=1).min():.3f}  max {np.linalg.norm(acc,axis=1).max():.3f}")
print(f"  分量均值 x={acc[:,0].mean():.3f} y={acc[:,1].mean():.3f} z={acc[:,2].mean():.3f}")
print(f"  分量范围 x[{acc[:,0].min():.2f},{acc[:,0].max():.2f}] y[{acc[:,1].min():.2f},{acc[:,1].max():.2f}] z[{acc[:,2].min():.2f},{acc[:,2].max():.2f}]")
print(f"角速度模长 max {np.linalg.norm(gyr,axis=1).max():.3f} rad/s")

# ---------- 轨迹对比 ----------
def pose_xy(m):
    p = m.pose.pose.position
    return np.array([p.x, p.y, p.z])


t0 = min(t for _, m in gt)
T_gt = np.array([to_ns(m.header.stamp) - t0 for _, m in gt])
P_gt = np.array([pose_xy(m) for _, m in gt])
T_od = np.array([to_ns(m.header.stamp) - t0 for _, m in odom])
P_od = np.array([pose_xy(m) for _, m in odom])

print(f"\n时间跨度 GT {T_gt[0]:.1f}..{T_gt[-1]:.1f}s   ODOM {T_od[0]:.1f}..{T_od[-1]:.1f}s")
print(f"GT   轨迹 XY 范围 x[{P_gt[:,0].min():.2f},{P_gt[:,0].max():.2f}] y[{P_gt[:,1].min():.2f},{P_gt[:,1].max():.2f}] z[{P_gt[:,2].min():.2f},{P_gt[:,2].max():.2f}]")
print(f"ODOM 轨迹 XY 范围 x[{P_od[:,0].min():.2f},{P_od[:,0].max():.2f}] y[{P_od[:,1].min():.2f},{P_od[:,1].max():.2f}] z[{P_od[:,2].min():.2f},{P_od[:,2].max():.2f}]")

print("\n=== 逐秒：ZLIO 相对参考点的位移 vs 真值 ===")
print(f"{'t':>5} {'odom|d|':>9} {'gt|d|':>8} {'odom_z':>8} {'odom_x':>8} {'odom_y':>8}")
for tq in np.arange(T_od[0] + 1, T_od[-1], 1.0):
    i = np.argmin(np.abs(T_od - tq))
    j = np.argmin(np.abs(T_gt - tq))
    d_od = np.linalg.norm(P_od[i] - P_od[0])
    d_gt = np.linalg.norm(P_gt[j] - P_gt[0])
    print(f"{tq:5.1f} {d_od*100:9.1f} {d_gt*100:8.1f} {P_od[i,2]:8.2f} {P_od[i,0]:8.2f} {P_od[i,1]:8.2f}")
