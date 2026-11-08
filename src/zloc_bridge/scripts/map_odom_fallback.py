#!/usr/bin/env python3
"""在 Z-Localization 后端不发布 map→odom 时接管该变换。

后端只在 `should_publish_tf` 为真时广播 map→odom（INITIALIZING / SUSPECT / LOST 期间不广播）。
Nav2 一旦拿不到 map→odom 就会规划失败并刷 `Timed out waiting for transform`。

判定依据只用后端自己的状态话题 `~/localization_state`，因为后端在同一个回调里
先 sendTransform 再把 `tf_published=true` 写给状态话题，两者严格同步。用 /tf 的新鲜度
去推断会和自己广播的变换互相强化，导致 20 Hz 抖动，所以这里不做那种判断。

行为：
- 状态显示 `tf_published=true` 且状态新鲜 → 完全不发，只缓存后端最近一次可信变换；
- 其余情况（后端冻结 / 后端进程不在）→ 按固定频率重复广播最后一次可信变换；
  一次都没收到过就广播单位变换，保证 map 帧存在，Nav2 不至于整栈停摆。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def roll_pitch_deg(q):
    """从四元数取 (roll, pitch)，单位度。"""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
    return roll, pitch


def parse_state(text):
    """把 `status=NORMAL;pose_trusted=true;...` 解析成 dict。"""
    fields = {}
    for item in text.split(';'):
        if '=' in item:
            key, _, value = item.partition('=')
            fields[key.strip()] = value.strip()
    return fields


class MapOdomFallback(Node):
    """后端停发 map→odom 时接管广播。"""

    def __init__(self):
        super().__init__('map_odom_fallback')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('state_topic', '/backend_node/localization_state')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('state_timeout', 2.0)
        # 接管/交还的去抖时长：避免状态标志闪动时两个节点同时广播 map→odom。
        self.declare_parameter('takeover_delay', 0.6)
        self.declare_parameter('yield_delay', 0.3)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        state_topic = self.get_parameter('state_topic').value
        rate = self.get_parameter('publish_rate').value
        self.state_timeout = self.get_parameter('state_timeout').value
        self.takeover_delay = self.get_parameter('takeover_delay').value
        self.yield_delay = self.get_parameter('yield_delay').value

        self.last_backend_tf = None      # 后端最近一次可信 map→odom
        self.backend_publishing = False  # 后端当前是否在广播
        self.last_state_time = None
        self.status = 'UNKNOWN'
        self.owning = False
        self.warned_missing_state = False
        self.warned_tilt = False
        self.backend_ok_state = None
        self.backend_ok_changed = 0.0

        state_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        tf_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST)

        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.on_tf, tf_qos)
        self.state_sub = self.create_subscription(String, state_topic, self.on_state, state_qos)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

        self.get_logger().info(
            f'map_odom_fallback 启动：{self.map_frame}→{self.odom_frame}，'
            f'状态来源 {state_topic}，状态超时 {self.state_timeout:.1f}s'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg: String):
        fields = parse_state(msg.data)
        self.last_state_time = self.now_sec()
        self.status = fields.get('status', 'UNKNOWN')
        # 只有后端确实广播了 map→odom，这个字段才是 true。
        self.backend_publishing = fields.get('tf_published') == 'true'
        self.warned_missing_state = False

    def on_tf(self, msg: TFMessage):
        # 自己接管期间发出去的变换不能再被当成后端结果缓存回来；
        # 其余时刻 /tf 上的 map→odom 只可能来自后端。
        if self.owning:
            return
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.map_frame
                and transform.child_frame_id == self.odom_frame
            ):
                self.last_backend_tf = transform
                self.check_tilt(transform.transform.rotation)

    def check_tilt(self, q):
        """map→odom 出现明显横滚/俯仰时提示：多半是雷达安装角没被里程计补偿。"""
        roll, pitch = roll_pitch_deg(q)
        if abs(roll) < 5.0 and abs(pitch) < 5.0:
            self.warned_tilt = False
            return
        if self.warned_tilt:
            return
        self.warned_tilt = True
        self.get_logger().warn(
            f'map→odom 含明显倾斜 roll={roll:+.1f}° pitch={pitch:+.1f}°：'
            '定位是用倾斜的姿态去贴合先验图的，Nav2 的 2D 代价地图会跟着歪。'
            '先跑 zloc_pose_check.py 判断是不是雷达安装横滚没被里程计补偿。'
        )

    def backend_alive(self):
        if self.last_state_time is None:
            return False
        return (self.now_sec() - self.last_state_time) < self.state_timeout

    def on_timer(self):
        backend_ok = self.backend_alive() and self.backend_publishing
        now = self.now_sec()

        # 去抖：后端状态里的 tf_published 会随发布周期闪动，直接跟着切换会让
        # 本节点和 backend 同时广播 map→odom（同一个子节点的两个父变换会打架）。
        if self.backend_ok_state is None:
            self.backend_ok_state = backend_ok
            self.backend_ok_changed = now
        elif backend_ok != self.backend_ok_state:
            self.backend_ok_state = backend_ok
            self.backend_ok_changed = now
        stable = now - self.backend_ok_changed

        if backend_ok:
            if self.owning and stable >= self.yield_delay:
                self.owning = False
                self.get_logger().info(
                    f'后端恢复广播 map→odom（status={self.status}），本节点停止接管'
                )
            return

        if not self.owning:
            if stable < self.takeover_delay:
                # 抖动窗口内什么都不做，既不发也不切换。
                return
            self.owning = True
            source = 'last-known' if self.last_backend_tf is not None else 'identity'
            reason = 'status=%s' % self.status if self.backend_alive() else '状态话题超时'
            self.get_logger().warn(
                f'后端连续 {stable:.1f}s 未发布 map→odom（{reason}），以 {source} 兜底广播'
            )

        if self.last_state_time is None and not self.warned_missing_state:
            self.warned_missing_state = True
            self.get_logger().warn(
                '尚未收到任何定位状态，若后端未启动属正常；若后端已启动请检查 state_topic 配置'
            )

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        if self.last_backend_tf is not None:
            out.transform = self.last_backend_tf.transform
        else:
            out.transform.rotation.w = 1.0

        self.tf_pub.publish(TFMessage(transforms=[out]))


def main():
    rclpy.init()
    node = MapOdomFallback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
