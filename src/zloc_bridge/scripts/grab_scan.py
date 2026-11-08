#!/usr/bin/env python3
"""抓取仿真里一帧真实雷达点云与真值位姿，用于离线核查先验图与仿真世界的几何一致性。"""
import sys
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry


class Grab(Node):
    def __init__(self):
        super().__init__('grab_scan')
        self.cloud = None
        self.pose = None
        self.frame = ''
        self.create_subscription(
            PointCloud2, '/red_standard_robot1/livox/lidar', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/red_standard_robot1/chassis_odometry_gt', self.on_odom, 10)
        self.get_logger().info('等待点云与真值位姿 ...')

    def on_cloud(self, msg):
        if self.cloud is not None:
            return
        try:
            pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float32)
            self.cloud = pts
            self.frame = msg.header.frame_id
            self.get_logger().info(f'收到点云 {pts.shape} frame={self.frame} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        except Exception:
            self.get_logger().error(traceback.format_exc())

    def on_odom(self, msg):
        if self.pose is None:
            p = msg.pose.pose
            self.pose = np.array([p.position.x, p.position.y, p.position.z,
                                  p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w], dtype=np.float64)
            self.get_logger().info(
                f'真值 pos=({p.position.x:.3f},{p.position.y:.3f},{p.position.z:.3f}) '
                f'frame={msg.header.frame_id} child={msg.child_frame_id}')

    def done(self):
        return self.cloud is not None and self.pose is not None


def main():
    out = sys.argv[1]
    rclpy.init()
    node = Grab()
    try:
        while rclpy.ok() and not node.done():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except Exception:
                traceback.print_exc()
                break
    finally:
        ok = node.cloud is not None and node.pose is not None
        if ok:
            np.savez(out, cloud=node.cloud, pose=node.pose, frame=np.array([node.frame]))
            print(f'saved {out}: cloud={node.cloud.shape} pose={node.pose}')
        else:
            print(f'未抓到完整数据 cloud={None if node.cloud is None else node.cloud.shape} pose={node.pose}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
