#!/usr/bin/env python3
"""一次性诊断节点：判断 `registered_scan` 与先验图之间的真实相对姿态。

背景：后端初始化时报
    `startup_hint position-prior verification failed ... support_ok=false, corr_ratio≈0.12`
说明只有少数点落进 0.6 m 门限。用理想扫描试过：单纯平移/偏航 90° 都还有 22% 以上，
实测 11.7% 且 33 个扇区都有命中，是「近处地面吻合、远处全丢」的典型倾斜特征。

本节点抓一帧 `registered_scan`，按后端同样的方式转回传感器系，然后在启动提示位置附近
扫描 (roll, yaw) 组合，直接报出哪个姿态假设的支持率最高。

用法（仿真与导航都跑起来之后另开一个终端）：
    ros2 run zloc_bridge zloc_pose_check.py --ros-args \
        -p bundle:=<地图包目录> -p hint_z:=0.512
输出最后一行 `VERDICT` 即为结论。
"""

import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inspect_map_bundle import load_pcd, load_keyframes, quat_to_rot, voxelize  # noqa: E402


def rpy_to_rot(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_to_rot_np(x, y, z, w):
    return quat_to_rot(np.array([x, y, z, w]))


class PoseCheck(Node):
    def __init__(self):
        super().__init__('zloc_pose_check')
        self.declare_parameter('bundle', '')
        self.declare_parameter('cloud_topic', '/red_standard_robot1/registered_scan')
        self.declare_parameter('odom_topic', '/red_standard_robot1/lidar_odometry')
        self.declare_parameter('hint_x', 0.0)
        self.declare_parameter('hint_y', 0.0)
        self.declare_parameter('hint_z', 0.512)
        self.declare_parameter('hint_yaw_deg', 0.0)
        self.declare_parameter('gate', 0.6)
        self.declare_parameter('max_points', 4000)
        self.declare_parameter('yaw_step_deg', 5.0)
        self.declare_parameter('roll_list', [-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0])

        self.gate = self.get_parameter('gate').value
        self.max_points = self.get_parameter('max_points').value
        self.hint = np.array([self.get_parameter('hint_x').value,
                              self.get_parameter('hint_y').value,
                              self.get_parameter('hint_z').value])
        self.hint_yaw = self.get_parameter('hint_yaw_deg').value
        self.tree = None
        self.cloud = None
        self.odom = None
        self.done = False

        bundle = self.get_parameter('bundle').value
        if not bundle:
            raise RuntimeError('必须提供 -p bundle:=<地图包目录>')
        self.load_map(bundle)

        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, 10)
        self.get_logger().info('已加载先验图，等待一帧 registered_scan + lidar_odometry ...')

    def load_map(self, bundle):
        pts = load_pcd(os.path.join(bundle, 'map.pcd')).astype(np.float64)
        try:
            kf_pose, kf_quat = load_keyframes(bundle)
            near = np.linalg.norm(kf_pose[:, :2] - self.hint[:2], axis=1) < 15.0
            if near.sum() > 0:
                sub = np.vstack([
                    load_pcd(os.path.join(bundle, 'clouds', f'{i:06d}.pcd')) @ quat_to_rot(kf_quat[i]).T
                    + kf_pose[i] for i in np.where(near)[0][:16]])
                pts = np.vstack([pts, sub])
                self.get_logger().info(f'目标点云 = 整图 + {int(near.sum())} 个邻近关键帧')
        except FileNotFoundError:
            pass
        pts = voxelize(pts, 0.15)
        from scipy.spatial import cKDTree
        self.tree = cKDTree(pts.astype(np.float64))
        self.map_size = pts.shape[0]
        self.get_logger().info(f'目标点云 {self.map_size} 点')

    def on_odom(self, msg):
        if self.odom is None:
            p = msg.pose.pose
            self.odom = np.array([p.position.x, p.position.y, p.position.z,
                                  p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w])

    def on_cloud(self, msg):
        if self.cloud is None:
            self.cloud = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float64)
            self.frames = msg.header.frame_id
            self.try_solve()

    def try_solve(self):
        if self.done or self.cloud is None or self.odom is None or self.tree is None:
            return
        self.done = True
        T_odom_sensor = np.eye(4)
        T_odom_sensor[:3, :3] = quat_to_rot_np(*self.odom[3:])
        T_odom_sensor[:3, 3] = self.odom[:3]
        # 与后端一致：cloud_local = T_odom_sensor^-1 * cloud_odom
        local = (self.cloud - self.odom[:3]) @ T_odom_sensor[:3, :3]
        if local.shape[0] > self.max_points:
            local = local[np.random.default_rng(0).choice(local.shape[0], self.max_points, False)]
        self.get_logger().info(
            f'收到点云 {self.cloud.shape[0]} 点（frame={self.frames}），下采样到 {local.shape[0]} 用于搜索')

        rows = []
        rolls = list(self.get_parameter('roll_list').value)
        step = self.get_parameter('yaw_step_deg').value
        for roll in rolls:
            for yaw in np.arange(-180.0, 180.0, step):
                R = rpy_to_rot(roll, 0.0, self.hint_yaw + yaw)
                pts = local @ R.T + self.hint
                dist, _ = self.tree.query(pts, k=1, distance_upper_bound=2.0)
                ok = dist < self.gate
                rows.append((float(ok.mean()), float(np.sqrt(np.mean(dist[ok] ** 2))) if ok.any() else 9.9,
                             roll, yaw))

        rows.sort(key=lambda r: -r[0])
        print('\n 支持率    rmse    roll      yaw(相对提示)')
        for ratio, rmse, roll, yaw in rows[:8]:
            print(f'  {ratio:6.3f}  {rmse:6.3f}  {roll:+7.1f}  {yaw:+7.1f}')

        base = next((r for r in rows if abs(r[2]) < 1e-6 and abs(r[3]) < 1e-6), None)
        best = rows[0]
        flat = max((r for r in rows if abs(r[2]) < 1e-6), key=lambda r: r[0])
        print(f'\nVERDICT 提示位姿(roll=0,yaw=0) ratio={base[0] if base else float("nan"):.3f}；'
              f'仅调 yaw 最佳 {flat[0]:.3f} (yaw={flat[3]:+.0f}°)；'
              f'含 roll 最佳 {best[0]:.3f} (roll={best[2]:+.0f}°, yaw={best[3]:+.0f}°)')
        if base and best[0] > 2.0 * base[0] and abs(best[2]) >= 5.0:
            print('VERDICT-HINT 需要非零 roll 才能吻合 → 雷达安装横滚未在里程计中被补偿')
        elif flat[0] > 2.0 * (base[0] if base else 1.0):
            print('VERDICT-HINT 需要非零 yaw 才能吻合 → 提示航向或里程计初始朝向偏了')
        elif base and base[0] > 0.5:
            print('VERDICT-HINT 提示位姿本身吻合良好 → 失败来自后端的子图/门槛，而不是几何')


def main():
    rclpy.init()
    node = PoseCheck()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
