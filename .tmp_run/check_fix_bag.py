#!/usr/bin/env python3
"""分析修复验证包：ZLIO 里程计 vs 真值，逐帧看漂移/跳变。"""
import glob
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob('/home/l/ros_ws/bags/zlio_fix_*'))[-1]
print('bag:', bag)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
ns = '/red_standard_robot1'
want = {ns + '/lidar_odometry', ns + '/chassis_odometry_gt', ns + '/livox/imu'}
buf = {k: [] for k in want}
while r.has_next():
    tp, d, _ = r.read_next()
    if tp in want:
        buf[tp].append(deserialize_message(d, get_message(types[tp])))


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


od = buf[ns + '/lidar_odometry']
gt = buf[ns + '/chassis_odometry_gt']
imu = buf[ns + '/livox/imu']
print(f'odom {len(od)}  gt {len(gt)}  imu {len(imu)}')

to = np.array([st(m) for m in od])
po = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in od])
tg = np.array([st(m) for m in gt])
pg = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in gt])

print(f'\nodom 范围 x[{po[:,0].min():.2f},{po[:,0].max():.2f}] y[{po[:,1].min():.2f},{po[:,1].max():.2f}] z[{po[:,2].min():.2f},{po[:,2].max():.2f}]')
print(f'gt   范围 x[{pg[:,0].min():.2f},{pg[:,0].max():.2f}] y[{pg[:,1].min():.2f},{pg[:,1].max():.2f}]')

# 逐帧位移增量，找跳变
d = np.linalg.norm(np.diff(po, axis=0), axis=1)
dt = np.diff(to)
print(f'\n相邻帧位移增量: 中位 {np.median(d)*100:.1f} cm  max {d.max()*1001:.0f} cm')
big = np.where(d > 0.5)[0]
print(f'单帧位移 >0.5 m 的次数: {len(big)}')
for i in big[:10]:
    print(f'  t={to[i]-to[0]:6.2f}s  Δ={d[i]*100:7.1f} cm  dt={dt[i]*1000:5.1f} ms  '
          f'pos=({po[i,0]:6.2f},{po[i,1]:6.2f},{po[i,2]:6.2f}) -> ({po[i+1,0]:6.2f},{po[i+1,1]:6.2f},{po[i+1,2]:6.2f})')

# 与真值的相对运动对照
print(f"\n{'t':>6}{'odom_d':>9}{'gt_d':>8}{'odom_z':>9}{'gaps':>7}")
i0 = 0
for q in np.arange(0, to[-1] - to[0], 2.0):
    i = int(np.argmin(np.abs((to - to[0]) - q)))
    j = int(np.argmin(np.abs((tg - tg[0]) - q)))
    gaps = np.sum(((to - to[0]) > q - 2.0) & ((to - to[0]) <= q))
    print(f'{q:6.1f}{np.linalg.norm(po[i]-po[0])*100:9.1f}{np.linalg.norm(pg[j]-pg[0])*100:8.1f}'
          f'{po[i,2]:9.2f}{gaps:7d}')
