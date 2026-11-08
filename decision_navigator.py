#!/usr/bin/env python3
"""Sentry decision navigator — 重写版状态机 v3.

优先级规则（从高到低）:
  1. 比赛未开始 (is_play != 4) → 原地等待, posture=3
  2. 比赛刚结束 (is_play 从4变0) → 取消目标, 导航去补给点
  3. 补给规则 (own_hp <= 120 或 bullet_allowance < 30) → 取消当前目标, 导航去补给点
  4. 起伏路区域 (Odometry 在坐标框内) → spin=0, scan=0, 速度×0.3
  5. 主线阶段 (is_play 刚变为4时, 一次性走 point_4→point_5→point_1)
  6. 进攻阶段 (outpost_HP > 0) → 目标点1, 每10秒 posture 在 1↔2 间切换
  7. 回防阶段 (outpost_HP == 0) → 目标点2/3 交替(mode!=2持续15秒后切换), 每10秒 posture 在 1↔2 间切换

关键机制:
  - 补给离开时根据当前 outpost_HP 决定去目标点1 还是目标点2
  - 补给过程中 posture=3, spin=0, scan=0
  - outpost_HP 从0变>0时立即切回目标点1进攻
  - 主线走完后清除 MAINLINE, 进入进攻/回防阶段
"""

import math
import sys
from enum import Enum, auto
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rm_serial_driver_nav_msgs.msg import FromDecision, FromSentry
from std_msgs.msg import Float64
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point


# ---------------------------------------------------------------------------
# 枚举定义
# ---------------------------------------------------------------------------

class Phase(Enum):
    WAIT = auto()             # 原地等待比赛开始
    MAINLINE = auto()         # 比赛开始, 一次性走 point_4→point_5→point_1
    OUTPOST_ATTACK = auto()   # 在目标点1 进攻前哨站
    SUPPLY = auto()           # 补给区
    RETREAT_DEFENSE = auto()  # 目标点2/3 回防交替


# ---------------------------------------------------------------------------
# 辅助类: 周期切换器
# ---------------------------------------------------------------------------

class ToggleTimer:
    """每隔 interval 秒在 val_a 和 val_b 之间切换一次."""

    def __init__(self, interval: float, initial_a: bool = True):
        self.interval = interval
        self._state: bool = initial_a
        self._accum: float = 0.0
        self._running: bool = False

    def reset(self, start_a: bool = True) -> None:
        self._state = start_a
        self._accum = 0.0
        self._running = True

    def stop(self) -> None:
        self._running = False
        self._accum = 0.0

    def tick(self, dt: float) -> Optional[bool]:
        if not self._running:
            return None
        self._accum += dt
        if self._accum >= self.interval:
            self._accum -= self.interval
            if self._accum >= self.interval:
                self._accum = self._accum % self.interval
            self._state = not self._state
            return self._state
        return None

    def current(self) -> Optional[bool]:
        return self._state if self._running else None

    def is_running(self) -> bool:
        return self._running


# ---------------------------------------------------------------------------
# 主类
# ---------------------------------------------------------------------------

class DecisionNavigator(Node):

    # ---------- 默认点位占位值(需替换为实车坐标) ----------
    POSE_DEFAULTS = {
        'point_1': [9.25,  3.8,  0.0],    # 目标点1: 进攻前哨站
        'point_2': [0.90, -1.30,  math.pi], # 目标点2: 己方半场回防
        'point_3': [0.90, 1.65,  math.pi], # 目标点3: 另一回防点
        'point_4': [3.76,  -6.60,  0.0],    # 起伏路途经点(近目标点1侧)
        'point_5': [1.19,  -6.60,  0.0],    # 起伏路途经点(近己方侧)
        'supply':  [-1.75, -5.00,  0.0],    # 补给区
    }

    # 主线顺序: point_4 → point_5 → point_1
    MAINLINE_ROUTE = [5, 4, 1]

    def __init__(self):
        super().__init__('decision_navigator')

        # ---------- 参数声明 ----------
        self.declare_parameter('action_name', 'navigate_through_poses')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('tick_period_sec', 0.2)
        self.declare_parameter('default_speed', 0.70)

        # 补给阈值
        self.declare_parameter('supply_hp_threshold', 120)       # own_hp <= 120 → 触发补给
        self.declare_parameter('supply_ammo_threshold', 30)     # bullet_allowance < 30 → 触发补给
        self.declare_parameter('supply_exit_hp', 380)         # 离开补给: own_hp >= 380
        self.declare_parameter('supply_exit_ammo', 280)        # 离开补给: bullet_allowance >= 280
        self.declare_parameter('supply_arrival_radius', 0.3)   # 进入补给区的判定半径

        # 目标点重发距离阈值参数
        self.declare_parameter('goal_resend_distance_threshold', 0.2)  # 距离目标点超过此值时重发导航
        self.declare_parameter('goal_resend_interval_sec', 1.0)          # 距离检查间隔

	    # 起伏路区域(坐标框, 需用户在参数中填入)
        self.declare_parameter('terrain_x_min', -0.25)
        self.declare_parameter('terrain_x_max', 4.06)
        self.declare_parameter('terrain_y_min', -7.50)
        self.declare_parameter('terrain_y_max', -5.54)
        self.declare_parameter('terrain_slow_ratio', 1.0)       # 起伏路减速倍率

        # 姿态/目标点切换周期
        self.declare_parameter('posture_toggle_sec', 10.0)      # posture 1↔2 切换周期
        self.declare_parameter('retreat_switch_sec', 15.0)       # mode!=2 持续N秒后切换回防点

        for name, default in self.POSE_DEFAULTS.items():
            self.declare_parameter(name, default)

        # ---------- 参数读取 ----------
        self._action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._tick_period = self.get_parameter('tick_period_sec').get_parameter_value().double_value
        self._default_speed = self.get_parameter('default_speed').get_parameter_value().double_value

        self._supply_hp_threshold = self.get_parameter('supply_hp_threshold').get_parameter_value().integer_value
        self._supply_ammo_threshold = self.get_parameter('supply_ammo_threshold').get_parameter_value().integer_value
        self._supply_exit_hp = self.get_parameter('supply_exit_hp').get_parameter_value().integer_value
        self._supply_exit_ammo = self.get_parameter('supply_exit_ammo').get_parameter_value().integer_value
        self._supply_arrival_radius = self.get_parameter('supply_arrival_radius').get_parameter_value().double_value

        # 读取目标点重发参数
        self._goal_resend_distance_threshold = self.get_parameter('goal_resend_distance_threshold').get_parameter_value().double_value
        self._goal_resend_interval_sec = self.get_parameter('goal_resend_interval_sec').get_parameter_value().double_value

        self._terrain_x_min = self.get_parameter('terrain_x_min').get_parameter_value().double_value
        self._terrain_x_max = self.get_parameter('terrain_x_max').get_parameter_value().double_value
        self._terrain_y_min = self.get_parameter('terrain_y_min').get_parameter_value().double_value
        self._terrain_y_max = self.get_parameter('terrain_y_max').get_parameter_value().double_value
        self._terrain_slow_ratio = self.get_parameter('terrain_slow_ratio').get_parameter_value().double_value

        self._posture_toggle_sec = self.get_parameter('posture_toggle_sec').get_parameter_value().double_value
        self._retreat_switch_sec = self.get_parameter('retreat_switch_sec').get_parameter_value().double_value

        # ---------- 读取点位 ----------
        self._poses = {
            key: self._read_pose_parameter(key)
            for key in self.POSE_DEFAULTS
        }

        # ---------- 发布器 ----------
        self._speed_pub = self.create_publisher(Float64, '/speed_multiplier', 10)
        self._posture_pub = self.create_publisher(FromDecision, '/posture_number', 10)

        # ---------- 订阅器 ----------
        self.create_subscription(FromSentry, 'simulator/from_sentry', self._from_sentry_callback, 10)
        self.create_subscription(Odometry, '/red_standard_robot1/odometry', self._odom_callback, 10)

        # ---------- Nav2 Action Client ----------
        self._action_client = ActionClient(self, NavigateThroughPoses, self._action_name)
        ok = self._action_client.wait_for_server(timeout_sec=60.0)
        if not ok:
            self.get_logger().fatal(f'未找到 action server "{self._action_name}"')
            self._startup_failed = True
            return

        # ---------- 状态变量 ----------
        self._is_play: int = 0
        self._mode: int = 0
        self._own_hp: Optional[float] = None
        self._allowance: Optional[float] = None
        self._outpost_hp: Optional[float] = None
        self._outpost_just_revived: bool = False

        self._phase: Phase = Phase.WAIT
        self._prev_phase: Phase = Phase.WAIT

        # 补给子状态: 防止在补给区重复发目标
        # NAVIGATING=正在前往补给区; WAITING=已在补给区等待补给完成; DONE=补给完成待离开
        self._supply_phase: str = 'IDLE'

        # 主线阶段: 索引和完成标志
        self._mainline_idx: int = 0
        self._mainline_done: bool = True

        # 导航目标生命周期
        self._goal_seq: int = 0
        self._goal_pending: bool = False
        self._goal_handle = None
        self._goal_phase: Optional[Phase] = None
        self._current_goal_point_key: Optional[str] = None  # 当前目标点名称

        # 目标点重发机制计时器
        self._goal_resend_timer: float = 0.0  # 累计时间

        # 发布状态缓存
        self._current_speed: Optional[float] = None
        self._last_phase = None
        self._last_posture: Optional[int] = None
        self._last_spin: Optional[int] = None
        self._last_scan: Optional[int] = None
        self._last_terrain_flag: Optional[int] = None

        # Odometry 缓存 (odom坐标系下)
        self._robot_x: Optional[float] = None
        self._robot_y: Optional[float] = None

        # Odometry 缓存 (map坐标系下, 用于减速区域判断)
        self._robot_x_map: Optional[float] = None
        self._robot_y_map: Optional[float] = None

        # TF2 Buffer 和 Listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 回防点切换逻辑: mode!=2 持续 N 秒后切换
        self._retreat_mode_bad_timer: float = 0.0      # 累计 mode!=2 的秒数
        self._retreat_switch_pending: bool = False      # 标记需要切换回防点
        self._retreat_arrived: bool = False             # 标记已到达回防点,开始计时
        self._retreat_point_idx: int = 2                # 当前回防点索引: 2 或 3,初始为 point_2
        self._posture_toggle = ToggleTimer(interval=self._posture_toggle_sec, initial_a=True)

        # ---------- 初始化发布 ----------
        self._publish_speed(self._default_speed)
        self._publish_posture(posture=3, spin=0, scan=0, terrain_flag=0)

        self.get_logger().info('Decision navigator (v3) 已启动')
        self._timer = self.create_timer(self._tick_period, self._tick)

    # -------------------------------------------------------------------------
    # 参数读取
    # -------------------------------------------------------------------------

    def _read_pose_parameter(self, name: str) -> PoseStamped:
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f'参数 {name} 必须是 [x, y, yaw]')
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = float(values[0])
        pose.pose.position.y = float(values[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quaternion(float(values[2]))
        return pose

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x, q.y = 0.0, 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # -------------------------------------------------------------------------
    # 回调
    # -------------------------------------------------------------------------

    def _from_sentry_callback(self, msg: FromSentry):
        own_hp = self._get_float(msg, 'own_hp', 'ownHp')
        allowance = self._get_float(msg, 'allowance', 'bullet_allowance')
        outpost_hp = self._get_float(msg, 'outpost_hp', 'outpost_HP')
        is_play = self._get_int(msg, 'is_play', 'isPlay', default=0)
        mode = self._get_int(msg, 'mode', default=0)

        prev_is_play = self._is_play
        self._is_play = is_play
        self._mode = mode

        if own_hp is not None:
            self._own_hp = own_hp
        if allowance is not None:
            self._allowance = allowance

        # 检测 is_play: 0→4, 触发主线出发
        if prev_is_play != 4 and is_play == 4:
            self.get_logger().info('比赛开始, 启动主线阶段')
            self._trigger_mainline()

        # 补给阈值实时检测
        if self._is_need_supply() and self._phase not in (Phase.SUPPLY, Phase.WAIT):
            self.get_logger().warn(
                f'补给规则触发(战斗中打断): own_hp={self._own_hp}, allowance={self._allowance}'
            )

        # 检测 outpost_HP: 0 → >0 事件
        if outpost_hp is not None and self._outpost_hp is not None:
            if self._outpost_hp == 0.0 and outpost_hp > 0.0:
                self._outpost_just_revived = True
                self.get_logger().info(f'前哨站复活: outpost_HP={outpost_hp}, 准备切回目标点1')

        self._outpost_hp = outpost_hp

        self.get_logger().debug(
            f'from_sentry: is_play={is_play}, mode={mode}, '
            f'own_hp={own_hp}, allowance={allowance}, outpost_hp={outpost_hp}'
        )

    def _odom_callback(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

        # 直接将 odom 坐标系的坐标当成 map 坐标系的坐标来用
        self._robot_x_map = msg.pose.pose.position.x
        self._robot_y_map = msg.pose.pose.position.y

    @staticmethod
    def _get_float(msg, *names, default=None) -> Optional[float]:
        for name in names:
            if hasattr(msg, name):
                try:
                    return float(getattr(msg, name))
                except (TypeError, ValueError):
                    pass
        return default

    @staticmethod
    def _get_int(msg, *names, default=None) -> Optional[int]:
        for name in names:
            if hasattr(msg, name):
                try:
                    return int(getattr(msg, name))
                except (TypeError, ValueError):
                    pass
        return default

    # -------------------------------------------------------------------------
    # 主循环
    # -------------------------------------------------------------------------

    def _tick(self):
        dt = self._tick_period

        # 核心: 判断当前相位
        self._phase = self._determine_phase()

        # 更新姿态/目标点切换器
        self._update_timers(dt)

        # 检查是否需要重发导航目标(在前往目标点途中,距离超过阈值时重发)
        self._check_goal_resend(dt)

        # 发送或管理导航目标
        self._maybe_send_goal()

        # 发布控制信号
        self._apply_control()

    def _update_timers(self, dt: float):
        if self._posture_toggle.tick(dt) is not None:
            cur = self._posture_toggle.current()
            posture = 1 if cur else 2
            label = "进攻" if cur else "防御"
            self.get_logger().info(f'posture 切换: {posture}({label})')

        # 回防点切换逻辑: mode!=2 持续 N 秒后触发切换
        # 只有在已到达回防点后才开始计时，路途不计时
        if self._phase == Phase.RETREAT_DEFENSE and self._retreat_arrived:
            if self._mode != 2:
                self._retreat_mode_bad_timer += dt
                if self._retreat_mode_bad_timer >= self._retreat_switch_sec:
                    self._retreat_mode_bad_timer = 0.0
                    self._retreat_switch_pending = True
                    self._retreat_arrived = False
                    self.get_logger().info(f'回防目标切换: mode!=2 持续 {self._retreat_switch_sec}s')
            else:
                self._retreat_mode_bad_timer = 0.0

    def _check_goal_resend(self, dt: float):
        """在前往目标点途中,每秒检查一次与目标点的距离,超过阈值则重发导航目标."""
        # 只有在有活跃导航目标且已经获得目标点信息时才检查
        if self._goal_handle is None or self._current_goal_point_key is None:
            return
        # 只有在 map 坐标可用时才检查
        if self._robot_x_map is None or self._robot_y_map is None:
            return
        # 补给阶段在到达补给点前需要重发,到达后(WAITING)不需要
        if self._phase == Phase.SUPPLY and self._supply_phase != 'NAVIGATING':
            return
        # 主线阶段: 逐点推进,到达后由 _on_goal_succeeded 推进索引
        # 回防/进攻阶段: 持续检查直到距离小于阈值

        self._goal_resend_timer += dt
        if self._goal_resend_timer < self._goal_resend_interval_sec:
            return
        self._goal_resend_timer = 0.0

        # 计算小车(map坐标系)到目标点的距离
        target_pose = self._poses[self._current_goal_point_key]
        dx = self._robot_x_map - target_pose.pose.position.x
        dy = self._robot_y_map - target_pose.pose.position.y
        distance = math.hypot(dx, dy)

        if distance > self._goal_resend_distance_threshold:
            self.get_logger().info(
                f'距离目标点 {self._current_goal_point_key} 还有 {distance:.3f}m > '
                f'{self._goal_resend_distance_threshold}m, 重发导航目标'
            )
            # 取消当前目标并重新发送
            self._cancel_current_goal()
        else:
            self.get_logger().debug(
                f'距离目标点 {self._current_goal_point_key} {distance:.3f}m, 已到达阈值内'
            )

    def _cancel_current_goal(self):
        """取消当前导航目标,但不重置状态标志,以便 _maybe_send_goal 重新发送."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        # 不重置 _goal_pending, 让 _maybe_send_goal 在下一周期发送新目标
        # 保留 _current_goal_point_key 和 _goal_phase 以便重新发送相同目标

    # -------------------------------------------------------------------------
    # 核心: 相位判断
    # -------------------------------------------------------------------------

    def _determine_phase(self) -> Phase:
        # ---- 规则0: 比赛未开始 ----
        if self._is_play != 4:
            self._stop_all_timers()
            self._supply_phase = 'IDLE'
            return Phase.WAIT

        # ---- 规则1: 补给规则 ----
        #
        # 核心逻辑 (enter/exit 分离):
        #   进入补给: 任意一项低于低阈值 → _is_need_supply() == True
        #   离开补给: 必须两项都达到高阈值 → _is_supply_ready() == True
        #
        # 子状态机:
        #   IDLE     → 不在补给
        #   NAVIGATING → 正在前往补给区
        #   WAITING  → 已在补给区,等待补给完成
        #   DONE     → 补给完成,待离开(只在相位切换前临时状态)
        #
        if self._phase == Phase.SUPPLY:
            # 正在补给: 无论是否在补给区,持续保持SUPPLY直到_is_supply_ready()
            if self._is_supply_ready():
                # 补给完成 → 重置子状态,允许离开
                self._supply_phase = 'IDLE'
                # 不在这里返回SUPPLY,让下面的规则接管
            else:
                # 补给未完成,保持在SUPPLY
                self._supply_phase = 'WAITING'
                return Phase.SUPPLY
        else:
            # 不在SUPPLY相位: 检测是否需要进入
            # 如果在减速区域内，不能切换状态，需等待离开后才能切换
            if self._is_need_supply():
                if self._is_in_terrain():
                    # 在减速区域内，暂不切换，保持在当前状态
                    self.get_logger().debug(
                        f'需要补给但处于减速区域内，等待离开: hp={self._own_hp}, ammo={self._allowance}'
                    )
                    self._supply_phase = 'PENDING'  # 标记为等待离开减速区域
                else:
                    # 已离开减速区域，可以进入补给状态
                    self._stop_all_timers()
                    self._supply_phase = 'NAVIGATING'
                    return Phase.SUPPLY
            else:
                self._supply_phase = 'IDLE'

        # ---- 规则2: 主线阶段 ----
        if not self._mainline_done:
            return Phase.MAINLINE

        # ---- 规则3: 进攻阶段 ----
        if self._outpost_just_revived:
            self._outpost_just_revived = False
            self._retreat_mode_bad_timer = 0.0
            self._retreat_switch_pending = False
            self._retreat_arrived = False
            self._posture_toggle.reset(start_a=True)
            return Phase.OUTPOST_ATTACK

        if self._outpost_hp is not None and self._outpost_hp > 0:
            self._retreat_mode_bad_timer = 0.0
            self._retreat_switch_pending = False
            self._retreat_arrived = False
            if not self._posture_toggle.is_running():
                self._posture_toggle.reset(start_a=True)
            return Phase.OUTPOST_ATTACK

        # ---- 规则4: 回防阶段 ----
        if self._outpost_hp is not None and self._outpost_hp == 0:
            if self._is_in_terrain():
                self.get_logger().debug(
                    f'outpost已倒下但处于减速区域内，等待离开后进入回防'
                )
                return Phase.MAINLINE
            self._posture_toggle.reset(start_a=True)
            # mode!=2 持续超时: 触发切换标志
            if self._retreat_switch_pending:
                self._retreat_switch_pending = False
                self._retreat_point_idx = 3 if self._retreat_point_idx == 2 else 2
                self.get_logger().info(f'回防切换: 前往 point_{self._retreat_point_idx}')
            return Phase.RETREAT_DEFENSE

        # ---- 兜底 ----
        self._stop_all_timers()
        return Phase.WAIT

    def _is_need_supply(self) -> bool:
        hp_low = self._own_hp is not None and self._own_hp <= self._supply_hp_threshold
        ammo_low = self._allowance is not None and self._allowance < self._supply_ammo_threshold
        return hp_low or ammo_low

    def _is_supply_ready(self) -> bool:
        hp_ok = self._own_hp is not None and self._own_hp >= self._supply_exit_hp
        ammo_ok = self._allowance is not None and self._allowance >= self._supply_exit_ammo
        return hp_ok and ammo_ok

    def _is_at_supply_point(self) -> bool:
        """判断机器人是否在补给点附近."""
        if self._robot_x_map is None or self._robot_y_map is None:
            return False
        pose = self._poses['supply']
        dx = self._robot_x_map - pose.pose.position.x
        dy = self._robot_y_map - pose.pose.position.y
        return math.hypot(dx, dy) < self._supply_arrival_radius

    def _stop_all_timers(self) -> None:
        self._posture_toggle.stop()
        self._retreat_mode_bad_timer = 0.0
        self._retreat_switch_pending = False
        self._retreat_arrived = False

    def _trigger_mainline(self):
        """is_play 从非4变为4时, 启动主线阶段."""
        self._cancel_goal()
        self._stop_all_timers()
        self._mainline_idx = 0
        self._mainline_done = False
        self._outpost_just_revived = False
        self._supply_phase = 'IDLE'
        self._supply_return_done = False
        self._retreat_point_idx = 2

    # -------------------------------------------------------------------------
    # 导航目标发送
    # -------------------------------------------------------------------------

    def _maybe_send_goal(self):
        # 如果有待处理的目标,不发送新目标
        # 但允许在取消当前目标后重新发送(用于重发机制)
        if self._goal_pending:
            return

        target_phase, point_key = self._nav_target()
        if target_phase is None:
            return

        # 如果当前已有活跃目标且目标相同,不重发
        if self._goal_handle is not None:
            if (target_phase == self._goal_phase and
                    point_key == self._current_goal_point_key):
                return

        self._send_goal(target_phase, point_key)

    def _nav_target(self):
        """返回 (phase, point_key) 或 (None, None)."""
        phase = self._phase

        # ---- 等待: 原地不动 ----
        if phase == Phase.WAIT:
            return (None, None)

        # ---- 补给: 前往补给点或原地等待补给完成 ----
        if phase == Phase.SUPPLY:
            if not self._is_supply_ready():
                # 补给未完成
                if self._supply_phase == 'NAVIGATING':
                    # 正在前往补给区: 发导航目标
                    self._supply_phase = 'NAVIGATING'
                    return (Phase.SUPPLY, 'supply')
                else:
                    # _supply_phase == 'WAITING': 已在补给区,原地等待补给恢复
                    # 不发新目标,让机器人停在补给点上
                    return (None, None)

            # 补给完成 → 根据局势决定下一目标
            self._supply_phase = 'IDLE'
            if self._outpost_hp is not None and self._outpost_hp > 0:
                return (Phase.OUTPOST_ATTACK, 'point_1')
            else:
                return (Phase.RETREAT_DEFENSE, 'point_2')

        # ---- 主线: 顺序发 point_4 → point_5 → point_1 ----
        if phase == Phase.MAINLINE:
            if self._mainline_idx < len(self.MAINLINE_ROUTE):
                idx = self.MAINLINE_ROUTE[self._mainline_idx]
                return (Phase.MAINLINE, f'point_{idx}')
            else:
                self._mainline_done = True
                self._posture_toggle.reset(start_a=True)
                self.get_logger().info('主线完成, 进入进攻/回防阶段')
                return (None, None)

        # ---- 进攻: 目标点1 ----
        if phase == Phase.OUTPOST_ATTACK:
            # 到达目标点后不再重复发送,由 _check_goal_resend 处理途中重发
            if self._current_goal_point_key == 'point_1' and self._goal_handle is not None:
                return (None, None)
            return (Phase.OUTPOST_ATTACK, 'point_1')

        # ---- 回防: 目标点2/3 交替 ----
        if phase == Phase.RETREAT_DEFENSE:
            self._retreat_arrived = False   # 出发了,路上不计时
            point = f'point_{self._retreat_point_idx}'
            # 到达目标点后不再重复发送,由 _check_goal_resend 处理途中重发
            if self._current_goal_point_key == point and self._goal_handle is not None:
                return (None, None)
            return (Phase.RETREAT_DEFENSE, point)

        return (None, None)

    def _send_goal(self, phase: Phase, point_key: str):
        self._goal_seq += 1
        self._goal_pending = True
        self._goal_phase = phase
        self._current_goal_point_key = point_key
        self._goal_resend_timer = 0.0  # 重置重发计时器

        pose = self._copy_pose(self._poses[point_key])
        # 不设置 pose.header.stamp，保持零时间戳，避免 tf2 外推错误。
        # 零时间戳会告诉 tf2 使用最新的可用变换，而非在特定时间查找。
        # 原先 self.get_clock().now() 在 use_sim_time 未正确配置时会用
        # 系统时间（Unix epoch 秒），与 Gazebo 仿真时间差 ~56 年，
        # 导致 "Lookup would require extrapolation into the future" 错误。

        self.get_logger().info(f'发送目标: {phase.name} → {point_key}')

        goal = NavigateThroughPoses.Goal()
        goal.poses = [pose]

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=lambda fb, seq=self._goal_seq: None,
        )
        future.add_done_callback(
            lambda fut, seq=self._goal_seq, ph=phase:
            self._goal_response_callback(seq, ph, fut)
        )

    def _goal_response_callback(self, seq: int, phase: Phase, future):
        if seq != self._goal_seq:
            return
        self._goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'发送目标失败: {exc}')
            self._goal_handle = None
            return

        if not goal_handle.accepted:
            self.get_logger().error(f'目标被拒绝: {phase.name}')
            self._goal_handle = None
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, seq=seq, ph=phase:
            self._goal_result_callback(seq, ph, fut)
        )

    def _goal_result_callback(self, seq: int, phase: Phase, future):
        if seq != self._goal_seq:
            return
        self._goal_handle = None

        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'获取导航结果失败: {exc}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'目标完成: {phase.name}')
            self._on_goal_succeeded(phase)
            # 目标完成后清空当前目标点记录,不再重复发送
            self._current_goal_point_key = None
            self._goal_phase = None
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'目标已取消: {phase.name}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'目标中止: {phase.name}')

    def _on_goal_succeeded(self, phase: Phase):
        # 主线阶段: 逐点推进索引
        if phase == Phase.MAINLINE:
            if self._mainline_idx < len(self.MAINLINE_ROUTE):
                self._mainline_idx += 1
        # 补给目标到达: 切换到等待子状态,不再重复发目标
        elif phase == Phase.SUPPLY:
            self._supply_phase = 'WAITING'
            self.get_logger().info(f'到达补给区,等待补给完成: hp={self._own_hp}, ammo={self._allowance}')
        # 回防目标到达: 开始 mode!=2 倒计时
        elif phase == Phase.RETREAT_DEFENSE:
            self._retreat_arrived = True
            self._retreat_mode_bad_timer = 0.0
            self.get_logger().info(
                f'到达回防点 point_{self._retreat_point_idx}, 开始监测 mode, '
                f'持续 {self._retreat_switch_sec}s 后切换'
            )

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_pending = False
        self._current_goal_point_key = None
        self._goal_phase = None
        self._goal_resend_timer = 0.0
        # 补给目标被取消(战斗打断),重置子状态为导航中,下次可重新出发
        if self._phase == Phase.SUPPLY:
            self._supply_phase = 'NAVIGATING'
        # 回防目标被取消: 重置到达标记,路上不计时
        elif self._phase == Phase.RETREAT_DEFENSE:
            self._retreat_arrived = False

    def _cancel_current_goal(self):
        """取消当前导航目标,但保留状态以便 _maybe_send_goal 重新发送相同目标."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_pending = False
        # 不重置 _current_goal_point_key 和 _goal_phase,
        # 让 _maybe_send_goal 在下一周期重新发送相同目标
        self._goal_resend_timer = 0.0

    def _copy_pose(self, pose: PoseStamped) -> PoseStamped:
        new_pose = PoseStamped()
        new_pose.header.frame_id = pose.header.frame_id
        new_pose.header.stamp = pose.header.stamp
        new_pose.pose.position.x = pose.pose.position.x
        new_pose.pose.position.y = pose.pose.position.y
        new_pose.pose.position.z = pose.pose.position.z
        new_pose.pose.orientation.x = pose.pose.orientation.x
        new_pose.pose.orientation.y = pose.pose.orientation.y
        new_pose.pose.orientation.z = pose.pose.orientation.z
        new_pose.pose.orientation.w = pose.pose.orientation.w
        return new_pose

    # -------------------------------------------------------------------------
    # 控制输出
    # -------------------------------------------------------------------------

    def _apply_control(self):
        phase = self._phase
        in_terrain = self._is_in_terrain()

        # ---- spin_flag / scan ----
        # 比赛未开始(WAIT)或起伏路区域: spin=0, scan=0
        # 其他所有阶段: spin=1, scan=1 (巡逻/进攻/补给/回目标点均启用)
        if phase == Phase.WAIT:
            spin = 0
            scan = 0
        elif in_terrain:
            spin = 0
            scan = 0
        else:
            spin = 1
            scan = 1

        # ---- speed ----
        speed = self._default_speed * self._terrain_slow_ratio if in_terrain else self._default_speed

        # ---- posture ----
        if phase in (Phase.WAIT, Phase.SUPPLY):
            posture = 3
        elif phase in (Phase.OUTPOST_ATTACK, Phase.RETREAT_DEFENSE):
            cur = self._posture_toggle.current()
            posture = 1 if cur else 2
        else:
            posture = 3

        self._publish_speed(speed)
        self._publish_posture(posture, spin, scan, terrain_flag=1 if in_terrain else 0)

        # ---- 日志: 相位切换时打印 ----
        if phase != self._last_phase:
            self.get_logger().info(
                f'相位切换: {self._last_phase.name if self._last_phase else "None"} → {phase.name}'
            )
            self._last_phase = phase

    def _is_in_terrain(self) -> bool:
        if self._robot_x_map is None or self._robot_y_map is None:
            return False
        if self._terrain_x_min == 0.0 and self._terrain_x_max == 0.0:
            return False
        return (self._terrain_x_min <= self._robot_x_map <= self._terrain_x_max and
                self._terrain_y_min <= self._robot_y_map <= self._terrain_y_max)

    def _publish_speed(self, speed: float):
        if self._current_speed is not None and math.isclose(
                self._current_speed, speed, rel_tol=1e-6):
            return
        msg = Float64()
        msg.data = speed
        self._speed_pub.publish(msg)
        self._current_speed = speed
        self.get_logger().debug(f'速度倍率: {speed:.2f}')

    def _publish_posture(self, posture: int, spin: int, scan: int, terrain_flag: int):
        state = (posture, spin, scan, terrain_flag)
        if state == (self._last_posture, self._last_spin, self._last_scan, self._last_terrain_flag):
            return

        msg = FromDecision()
        msg.posture = posture
        msg.spin_flag = spin
        msg.scan = scan
        msg.terrain_flag = terrain_flag
        self._posture_pub.publish(msg)

        self._last_posture = posture
        self._last_spin = spin
        self._last_scan = scan
        self._last_terrain_flag = terrain_flag

        posture_label = {1: "进攻", 2: "防御", 3: "移动"}.get(posture, str(posture))
        terrain_label = "起伏路" if terrain_flag == 1 else "普通"
        self.get_logger().info(
            f'发布姿态: posture={posture}({posture_label}), '
            f'spin_flag={spin}, scan={scan}, terrain_flag={terrain_flag}({terrain_label}), phase={self._phase.name}'
        )


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def main():
    rclpy.init(args=sys.argv)
    node = DecisionNavigator()

    if getattr(node, '_startup_failed', False):
        node.destroy_node()
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，退出')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
