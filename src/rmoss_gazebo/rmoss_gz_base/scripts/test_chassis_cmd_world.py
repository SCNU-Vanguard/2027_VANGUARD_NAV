#!/usr/bin/env python3
"""
世界坐标系下的机器人底盘控制脚本
- WASD 控制机器人在世界坐标系下的绝对方向移动（与机器人当前朝向无关）
- [ ] 控制转向
- 松开按键后机器人立即停止
"""

import sys
import termios
import tty
import select
import math

import rclpy
from rclpy.node import Node
from rmoss_interfaces.msg import ChassisCmd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


msg = """
世界坐标系底盘控制
---------------------------
移动控制（世界坐标系绝对方向，与机器人朝向无关）：
        w    (向前，世界坐标系 +X 方向)
   a    s    d
  (向左  向后  向右)
        
转向控制：
   [  - 向左旋转
   ]  - 向右旋转

停止：
   space - 立即停止

模式切换：
   z - 速度控制模式（底盘坐标系）
   x - 跟随云台模式

CTRL-C 退出
---------------------------
"""


class WorldChassisController(Node):
    def __init__(self):
        super().__init__('world_chassis_controller')
        
        # 声明参数
        self.declare_parameter('v', 1.0)  # 线速度
        self.declare_parameter('w', 1.0)  # 角速度
        self.v = self.get_parameter('v').value
        self.w = self.get_parameter('w').value
        
        # 当前机器人位姿
        self.current_yaw = 0.0
        self.has_odom = False
        
        # 目标速度（世界坐标系）
        self.world_vx = 0.0  # 世界坐标系 X 方向速度
        self.world_vy = 0.0  # 世界坐标系 Y 方向速度
        self.angular_w = 0.0  # 角速度
        
        # 控制模式
        self.chassis_type = 1  # 1: VELOCITY, 2: FOLLOW_GIMBAL
        
        # 创建发布者
        self.chassis_pub = self.create_publisher(ChassisCmd, 'chassis_cmd', 10)
        
        # 订阅里程计数据获取当前朝向
        self.odom_sub = self.create_subscription(
            Odometry,
            'robot_base/odom',
            self.odom_callback,
            10
        )
        
        # 创建定时器，定期发布速度命令（即使按键已松开也要发送停止命令）
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20Hz
        
        self.get_logger().info(f"节点参数 v:{self.v}, w:{self.w}")
        
    def quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角 (roll, pitch, yaw)"""
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def odom_callback(self, msg: Odometry):
        """从里程计数据中提取机器人当前朝向（yaw角）"""
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )
        self.has_odom = True
        
    def world_to_chassis_velocity(self, world_vx, world_vy, yaw):
        """
        将世界坐标系下的速度转换为底盘坐标系下的速度
        
        世界坐标系 -> 底盘坐标系的旋转矩阵（绕Z轴旋转 -yaw）：
        [vx_chassis]   [cos(yaw)   sin(yaw)] [vx_world]
        [vy_chassis] = [-sin(yaw)  cos(yaw)] [vy_world]
        """
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        vx_chassis = world_vx * cos_yaw + world_vy * sin_yaw
        vy_chassis = -world_vx * sin_yaw + world_vy * cos_yaw
        
        return vx_chassis, vy_chassis
        
    def publish_cmd(self):
        """发布底盘控制命令"""
        # 将世界坐标系速度转换为底盘坐标系速度
        if self.has_odom:
            chassis_vx, chassis_vy = self.world_to_chassis_velocity(
                self.world_vx, self.world_vy, self.current_yaw
            )
        else:
            # 如果没有收到里程计数据，直接使用世界坐标系速度（会相对于底盘）
            chassis_vx = self.world_vx
            chassis_vy = self.world_vy
            
        # 创建并发布 ChassisCmd 消息
        cmd = ChassisCmd()
        cmd.type = self.chassis_type
        cmd.twist.linear.x = chassis_vx
        cmd.twist.linear.y = chassis_vy
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = self.angular_w
        
        self.chassis_pub.publish(cmd)
        
    def set_world_velocity(self, vx, vy):
        """设置世界坐标系下的目标速度"""
        self.world_vx = vx
        self.world_vy = vy
        
    def set_angular_velocity(self, w):
        """设置角速度"""
        self.angular_w = w
        
    def stop(self):
        """停止所有运动"""
        self.world_vx = 0.0
        self.world_vy = 0.0
        self.angular_w = 0.0


def get_key_with_timeout(settings, timeout=0.1):
    """
    非阻塞方式获取键盘输入，带超时
    返回 (key, has_input)
    """
    tty.setraw(sys.stdin.fileno())
    try:
        # 使用 select 检查是否有输入，超时时间为 timeout 秒
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            key = sys.stdin.read(1)
            return key, True
        else:
            return None, False
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = WorldChassisController()
    
    print(msg)
    print("等待里程计数据...")
    
    # 等待收到里程计数据
    spin_count = 0
    while not node.has_odom and spin_count < 100:  # 最多等待5秒
        rclpy.spin_once(node, timeout_sec=0.05)
        spin_count += 1
        
    if node.has_odom:
        print("已获取机器人位姿，可以开始控制")
    else:
        print("警告：未收到里程计数据，控制将相对于底盘坐标系")
    
    try:
        while rclpy.ok():
            # 使用非阻塞方式获取按键，超时100ms
            key, has_input = get_key_with_timeout(settings, timeout=0.1)
            
            if has_input:
                # 处理按键输入
                if key == 'w':
                    # 世界坐标系 +X 方向（向前）
                    node.set_world_velocity(node.v, 0.0)
                elif key == 's':
                    # 世界坐标系 -X 方向（向后）
                    node.set_world_velocity(-node.v, 0.0)
                elif key == 'a':
                    # 世界坐标系 +Y 方向（向左）
                    node.set_world_velocity(0.0, node.v)
                elif key == 'd':
                    # 世界坐标系 -Y 方向（向右）
                    node.set_world_velocity(0.0, -node.v)
                elif key == '[':
                    # 向左旋转
                    node.set_angular_velocity(node.w)
                elif key == ']':
                    # 向右旋转
                    node.set_angular_velocity(-node.w)
                elif key == ' ':
                    # 空格键停止
                    node.stop()
                elif key == 'z':
                    node.chassis_type = 1  # VELOCITY mode
                    print("切换到速度控制模式")
                elif key == 'x':
                    node.chassis_type = 2  # FOLLOW_GIMBAL mode
                    print("切换到跟随云台模式")
                elif key == '\x03':  # CTRL-C
                    break
                else:
                    # 其他按键，忽略
                    pass
            else:
                # 没有按键输入，停止运动（松开即停）
                node.stop()
            
            # 处理 ROS2 回调
            rclpy.spin_once(node, timeout_sec=0)
            
    except KeyboardInterrupt:
        pass
    finally:
        # 确保机器人停止
        node.stop()
        node.publish_cmd()
        
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n已退出")


if __name__ == '__main__':
    main()
