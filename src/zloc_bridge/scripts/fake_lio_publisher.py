#!/usr/bin/env python3
"""离线用假数据驱动 Z-Localization 后端，验证接入链路。

不需要 Gazebo：直接把先验地图裁出一块当作「当前帧点云」，配合单位里程计，
按 loam_interface 的实际接口发布：

  - /<ns>/lidar_odometry   nav_msgs/Odometry      (frame=odom, child=front_mid360)
  - /<ns>/registered_scan  sensor_msgs/PointCloud2 (frame=odom)

如果后端能收敛，说明配置、描述子、time 同步、TF 发布这几环都是通的。
注意：这是验证工具，不参与实际运行。
"""



import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def load_pcd(path):
    raw = open(path, 'rb').read()
    marker = b'DATA binary\n'
    idx = raw.find(marker) + len(marker)
    header = raw[:idx].decode(errors='ignore')
    count = int([l.split()[1] for l in header.splitlines() if l.startswith('POINTS')][0])
    return np.frombuffer(raw[idx:idx + count * 12], dtype=np.float32).reshape(-1, 3)


def visible_scan(
    points, origin, az_res=0.8, el_res=1.0, max_range=45.0, min_range=0.4,
    el_min=-7.0, el_max=52.0,
):
    """按角度分箱只保留最近点，近似一次真实扫描的遮挡效果。

    直接拿先验地图的一大块当「当前帧」会让点到面匹配的支持率虚低，
    因为真实雷达看不到墙后的点。这里用角度分箱取最近点，等价于一次
    稀疏的射线投射，并按 Mid-360 的垂直视场（-7°~+52°）裁剪。
    """
    delta = points - origin[None, :]
    rng = np.linalg.norm(delta, axis=1)
    mask = (rng > min_range) & (rng < max_range)
    delta, rng = delta[mask], rng[mask]

    az = np.degrees(np.arctan2(delta[:, 1], delta[:, 0]))
    el = np.degrees(np.arcsin(np.clip(delta[:, 2] / np.maximum(rng, 1e-6), -1.0, 1.0)))
    fov = (el >= el_min) & (el <= el_max)
    delta, rng, az, el = delta[fov], rng[fov], az[fov], el[fov]
    mask_idx = np.flatnonzero(mask)[fov]

    ai = np.floor((az + 180.0) / az_res).astype(np.int64)
    ei = np.floor((el - el_min) / el_res).astype(np.int64)
    keys = ai * 10000 + ei

    order = np.lexsort((rng, keys))
    keys_sorted = keys[order]
    first = np.ones(keys_sorted.shape[0], dtype=bool)
    first[1:] = keys_sorted[1:] != keys_sorted[:-1]
    return points[mask_idx[order][first]]


def to_pointcloud2(points, frame_id, stamp):
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * points.shape[0]
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


class FakeLioPublisher(Node):
    def __init__(self):
        super().__init__('fake_lio_publisher')

        self.declare_parameter('namespace', 'red_standard_robot1')
        self.declare_parameter('pcd', '')
        self.declare_parameter('radius', 25.0)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('sensor_frame', 'front_mid360')
        # 模拟的传感器在 map 系中的位置。仿真出生点 (-10.10, 0) 对应 map (0.30, -0.05)
        # （见 zloc_backend.yaml 的 startup_hint 说明），所以 map→odom 应为单位变换。
        self.declare_parameter('sensor_x', 0.30)
        self.declare_parameter('sensor_y', -0.05)
        self.declare_parameter('sensor_z', 0.512)

        ns = self.get_parameter('namespace').value.strip('/')
        pcd = self.get_parameter('pcd').value
        radius = self.get_parameter('radius').value
        rate = self.get_parameter('rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        origin = np.array([
            self.get_parameter('sensor_x').value,
            self.get_parameter('sensor_y').value,
            self.get_parameter('sensor_z').value,
        ])

        points = load_pcd(pcd)
        # 近似一次真实扫描：按角度分箱取最近点，自带遮挡。
        scan = visible_scan(points, origin, max_range=radius)
        # 真实雷达发的是「传感器坐标系」的点云，里程计位姿从这里起算。
        # 必须先减去原点，否则点云本身就带着 map 坐标，配准结果恒为 0，
        # 这个离线验证就测不出绝对位置的偏差了。
        scan = scan - origin[None, :]
        # 体素下采样控制消息体积，同时保持几何结构。
        voxel = 0.1
        keys = np.floor(scan / voxel).astype(np.int64)
        _, keep = np.unique(keys, axis=0, return_index=True)
        self.cloud = scan[keep]
        self.get_logger().info(
            f'载入 {points.shape[0]} 点，模拟扫描得到 {self.cloud.shape[0]} 点'
            f'（半径 {radius} m）'
        )

        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST)
        self.odom_pub = self.create_publisher(Odometry, f'/{ns}/lidar_odometry', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, f'/{ns}/registered_scan', qos)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(f'开始向 /{ns} 发布假 LIO 数据，{rate:.1f} Hz')

    def publish(self):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.sensor_frame
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        # registered_scan 在 odom 系，与里程计同 stamp。
        self.cloud_pub.publish(to_pointcloud2(self.cloud, self.odom_frame, stamp))


def main():
    rclpy.init()
    node = FakeLioPublisher()
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
