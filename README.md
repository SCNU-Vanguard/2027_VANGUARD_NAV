# 2027_VANGUARD_NAV

SCNU-Vanguard 2027 赛季哨兵/步兵导航工作空间（ROS 2 Humble + Gazebo）。

## 目录结构

```
ros_ws/
├── src/
│   ├── pb2025_sentry_nav/          # 导航核心：Nav2 配置、point_lio、地形分析、重定位
│   ├── pb2025_robot_description/   # 机器人 URDF/Xacro 描述
│   ├── rmu_gazebo_simulator/       # Gazebo 仿真启动与场地模型
│   ├── rmoss_core/                 # RoboMaster 通用组件（底盘/云台/相机）
│   ├── rmoss_gazebo/               # rmoss 仿真插件
│   ├── rmoss_gz_resources/         # Gazebo 模型资源
│   ├── rmoss_interfaces/           # 自定义 msg/srv 接口
│   ├── z-localization/             # ZLIO 定位（替代方案）
│   ├── zloc_bridge/                # ZLIO 与 Nav2 的桥接
│   ├── chassis_cmd_world/          # 世界坐标系下的底盘速度指令
│   └── sdformat_tools/             # SDF 工具
├── docs/                           # 定位方案对比、集成与调试记录
├── start.sh                        # 常用启动命令速查
└── stop_all.sh                     # 清理残留进程
```

## 环境依赖

- ROS 2 Humble
- Gazebo（gz-sim）
- GTSAM 4.2（`.deps/deb/` 下提供了 deb 包）
- 其他依赖通过 `rosdep` 安装

## 编译

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 运行

终端 1 —— 启动仿真：

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

终端 2 —— 启动定位与导航：

```bash
./stop_all.sh nav
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True
```

更多命令见 `start.sh`，定位方案的取舍见 `docs/`。

## 说明

- **不包含**编译产物（`build/`、`install/`、`log/`）、录包（`bags/`）和仿真地图点云（`*.pcd`，体积过大）。
  地图需要各自用 `point_lio` 离线建图生成，方法见 `docs/`。
- 各 `src/` 子包源自上游开源仓库（SMBU-PolarBear-Robotics-Team、rmoss 等），
  此处已合并为单一仓库，包含本队的本地修改。
