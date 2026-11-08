#!/usr/bin/env python3
"""自车点云剔除：把 registered_scan 里"车身自己那一圈点"挖掉。

为什么需要它
------------
pb2025_sentry_robot 在车身四周加了一圈 10 cm 高的黑色竖板（guard_panel：4 块侧板 + 4 块 45°
角板，外沿 0.29 m、角板 0.31 m）。雷达装在 0.237 m 高、整车朝下 -15°，按实际位姿逐条射线求交，
8 块板全部落在视场里，最近命中 0.179 m、最远 0.317 m。

这些点永远跟着车走，于是：terrain_analysis 把它们判成障碍发进 `terrain_map` →
local_costmap 的 intensity_voxel_layer 在车周 0.2~0.4 m 打一圈 lethal →
配上 inflation_radius 0.7，机器人一上来就"被自己围死"，控制器规划不出可行路径。

处理办法
--------
`registered_scan` 已经是 odom 系，`lidar_odometry` 给出车体在同一个系里的位置，因此直接按
**到车体中心的水平距离** 切一刀：距离 < self_radius 的点丢掉。

为什么是 0.34 m
---------------
自车围板外沿（角板）0.311 m，取 0.34 留 3 cm 余量；而 0.34 m 以内的真实障碍本来就已经
在车体（nav2 robot_radius 0.285）里面了，所以这一刀不会误伤。只看 XY、不看 Z ——
这个尺度上除了自车没有别的东西。

谁不需要它
----------
- point_lio：`preprocess.blind: 0.5` 已经滤掉 0.5 m 内的点；
- ZLIO 前端：`zlio_sim.yaml` 的 `sensor.minimum_range: 0.34` 等效；
- 剩下吃 `registered_scan` 的 terrain_analysis / terrain_analysis_ext 走这个节点。
  （small_gicp_relocalization 也用 registered_scan，若开启重定位，把它的输入一并 remap 过来。）
"""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudSelfFilter(Node):
    """按水平距离剔除自车点云的透明节点。"""

    def __init__(self):
        super().__init__("cloud_self_filter")

        self.declare_parameter("input_topic", "registered_scan")
        self.declare_parameter("odom_topic", "lidar_odometry")
        self.declare_parameter("output_topic", "registered_scan_filtered")
        self.declare_parameter("self_radius", 0.34)
        self.declare_parameter("odom_timeout", 1.0)
        self.declare_parameter("report_period", 10.0)

        self._radius = float(self.get_parameter("self_radius").value)
        self._radius_sq = self._radius * self._radius
        self._odom_timeout = float(self.get_parameter("odom_timeout").value)
        report_period = float(self.get_parameter("report_period").value)

        self._origin = None       # (x, y)，odom 系
        self._origin_time = None
        self._frames = 0
        self._dropped = 0
        self._stale_warned = False

        # 订阅用 BEST_EFFORT（能同时匹配 reliable / best_effort 两种上游），
        # 发布用 RELIABLE（对齐 terrain_analysis 的默认订阅）。
        cloud_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        out_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, out_qos
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 20
        )
        self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value, self._on_cloud, cloud_qos
        )

        if report_period > 0.0:
            self.create_timer(report_period, self._report)

        self.get_logger().info(
            f"cloud_self_filter: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}, self_radius={self._radius:.2f} m"
        )

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._origin = (p.x, p.y)
        self._origin_time = self.get_clock().now()

    def _on_cloud(self, msg):
        if msg.width * msg.height == 0:
            return

        if self._needs_filter():
            self._pub.publish(msg)
            return

        step = msg.point_step
        n = msg.width * msg.height
        off = {f.name: f.offset for f in msg.fields}
        if "x" not in off or "y" not in off:
            self._pub.publish(msg)
            return

        # 只用 strides 视图取 x/y，不复制整块点云
        xs = np.ndarray((n,), dtype=np.float32, buffer=msg.data, offset=off["x"], strides=(step,))
        ys = np.ndarray((n,), dtype=np.float32, buffer=msg.data, offset=off["y"], strides=(step,))
        dx = xs - self._origin[0]
        dy = ys - self._origin[1]
        keep = (dx * dx + dy * dy) >= self._radius_sq
        kept = int(np.count_nonzero(keep))

        self._frames += 1
        self._dropped += n - kept
        if kept == n:
            self._pub.publish(msg)
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * step).reshape(n, step)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = kept
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.row_step = step * kept
        out.is_dense = msg.is_dense
        out.data = raw[keep].tobytes()
        self._pub.publish(out)

    # ------------------------------------------------------------------ 工具
    def _needs_filter(self):
        """没有可用位姿时直通：宁可多给点云，也不要让 terrain_analysis 断流。"""
        if self._origin is None:
            if not self._stale_warned:
                self.get_logger().warn("还没收到 lidar_odometry，点云直通（不做自车剔除）")
                self._stale_warned = True
            return True
        age = (self.get_clock().now() - self._origin_time).nanoseconds * 1e-9
        if age > self._odom_timeout:
            if not self._stale_warned:
                self.get_logger().warn(
                    f"lidar_odometry 已 {age:.2f}s 未更新，点云直通（不做自车剔除）"
                )
                self._stale_warned = True
            return True
        self._stale_warned = False
        return False

    def _report(self):
        if self._frames == 0:
            return
        self.get_logger().info(
            f"自车剔除：{self._frames} 帧，平均每帧丢 "
            f"{self._dropped / self._frames:.0f} 点（r < {self._radius:.2f} m）"
        )
        self._frames = 0
        self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = CloudSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
