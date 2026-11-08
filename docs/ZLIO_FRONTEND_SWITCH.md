# ZLIO 前端接入（Phase A）：一个开关换前端

> 目标：`use_zlio:=True/False` 切换前端，**backend、Nav2、地图资产零改动**，可随时回滚。
> 状态：**仿真链路已接通并编译验证**（2026-11-03）；未跑真实 Gazebo 数据（本机无 GUI 会话不出点）。
> 相关：`docs/POINT_LIO_VS_ZLIO.md`（选型依据）、`docs/ZLOC_USAGE_AND_COMPARISON.md`（backend 侧）

---

## 1. 开关做了什么

| | `use_zlio:=False`（默认，原样） | `use_zlio:=True` |
| --- | --- | --- |
| 里程计节点 | `point_lio/pointlio_mapping` | `frontend/zlio_node`（节点名 `zlio`） |
| `loam_interface` | 启动 | **不启动** |
| `lidar_odometry` 来源 | loam_interface 转发 `/aft_mapped_to_init` | ZLIO 直接发布（`frame_id=odom`, `child=front_mid360`） |
| `registered_scan` 来源 | loam_interface 转发 `/cloud_registered` | `zloc_bridge/zlio_legacy_bridge.py` 把 `frontend/frame` 的局部点云变换到 odom 系 |
| `odom→base_footprint` TF | sensor_scan_generation | sensor_scan_generation（**不变**） |
| `map→odom` TF | backend（`use_zloc:=True` 时） | 同一个 backend（**不变**） |

关键设计：**ZLIO 只替换"里程计 + 点云"这一对输入**，输出与 loam_interface 同构，所以后端仍然跑
`frontend: legacy`，不需要在 Phase A 就切协议。

```
point_lio ─/cloud_registered ─┐
          └/aft_mapped_to_init─┴→ loam_interface ─┬→ registered_scan ─┐
                                                   └→ lidar_odometry ──┤
                                                                       ├→ terrain_analysis / sensor_scan_generation / zloc backend(legacy)
ZLIO ─/frontend/frame → zlio_legacy_bridge ─→ registered_scan ─────────┤
     └/lidar_odometry ─────────────────────────────────────────────────┘
```

---

## 2. 怎么跑（仿真）

```bash
cd ~/ros_ws
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# 终端 1：Gazebo（必须先起，否则 ZLIO 没有输入）
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 终端 2：导航 + 定位（Z-Loc 后端 + ZLIO 前端）
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True use_zlio:=True
```

回滚（前端换回 point_lio，其余不动）：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True use_zlio:=False
```

也可只单独验证前端：

```bash
ros2 launch frontend lio.launch.py \
  config:=$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/zlio_sim.yaml \
  input_type:=pointcloud2 lidar_topic:=/red_standard_robot1/velodyne_points \
  imu_topic:=/red_standard_robot1/livox/imu imu_acceleration_scale:=1.0 use_sim_time:=true
```

---

## 3. 改了哪些文件

| 文件 | 改动 |
| --- | --- |
| `src/z-localization/src/frontend/COLCON_IGNORE` | **删除**（前端参与编译；`bringup` / 自带 `livox_ros_driver2` 仍忽略） |
| `config/simulation/zlio_sim.yaml` | 新增：ZLIO 算法参数（只改"仿真传感器事实"与采样点数，算法默认值保留） |
| `config/simulation/nav2_params.yaml` | 新增 `zlio:` 节点参数块（输入类型、时间字段、坐标系、publish_tf 等） |
| `src/zloc_bridge/scripts/zlio_legacy_bridge.py` | 新增：`FrontendFrame` → `registered_scan`（odom 系） |
| `src/zloc_bridge/CMakeLists.txt` / `package.xml` | 安装新脚本、补依赖 |
| `launch/localization_launch.py` | `use_zlio` / `zlio_config` 参数；point_lio 加 `UnlessCondition`；新增 zlio 与桥接节点 |
| `launch/navigation_launch.py` | `use_zlio` 参数；`loam_interface` 两种形态（独立/组合）都加 `UnlessCondition` |
| `launch/bringup_launch.py`、`rm_navigation_simulation_launch.py` | 透传 `use_zlio` / `zlio_config` |

---

## 4. 起来之后先看这几个

```bash
ros2 topic hz   /red_standard_robot1/registered_scan      # 期望 ≈ 雷达帧率（20 Hz）
ros2 topic hz   /red_standard_robot1/lidar_odometry      # 观测频率，0.03~0.12 s 一次
ros2 topic echo /red_standard_robot1/lidar_odometry --once | head -20   # frame_id=odom, child=front_mid360
ros2 run tf2_ros tf2_echo odom base_footprint            # 必须存在，roll/pitch 应≈0
ros2 topic echo /red_standard_robot1/backend_node/localization_state --once
```

桥接节点每 5 s 打一行 `Hz | 平均点数 | matches | lidar_updated | epoch`：

- **Hz ≈ 0 或只有几 Hz**：ZLIO 还在初始化（需要 20 帧 ≈ 1 s，且视野内平面 ≥ 10）。
- **平均点数只有几百**：`sampling.minimum_points` 太低或 `minimum_range` 太大，见第 6 节。
- **`lidar_updated=false` 频繁**：点云与地图匹配不上，看 `matches`。

---

## 5. A/B 怎么做

同一段轨迹、同一个 backend 配置，只切 `use_zlio`：

1. `use_zlio:=False` 跑一遍，记录：`localization_state` 的 SUSPECT/LOST 次数、日志里 `ratio`/`rmse` 分布、
   `terrain_analysis` 输出是否抖、`top` 里 point_lio 的单核占用。
2. `use_zlio:=True` 跑同样一段，记录同样指标 + 桥接节点的 Hz/点数。
3. 判据见 `docs/POINT_LIO_VS_ZLIO.md` 第 5 节（终点漂移 ≤1.1×、静止漂移 <2 cm/0.5°、
   SUSPECT ≤1.2×、`ratio ≥0.85`、CPU 不高于 point_lio、冷启动 <3 s）。

---

## 6. 已知限制与首轮调参

| 现象 | 原因 | 先动哪个参数 |
| --- | --- | --- |
| 起步后 1~2 s 没有 `lidar_odometry` | 初始化要 20 帧窗口 | `initialization.window_frames`（别低于 10） |
| 一直不初始化 | 视野内平面 <10 或最弱方向支撑不足 | 挪车/换出生朝向；`minimum_directional_support` 可降 |
| 点太少、匹配弱 | 仿真点云密但被采到 1k~4k | `sampling.minimum_points/maximum_points`（当前 2500/8000） |
| 近处地面被切掉 | `minimum_range` 0.2 | 与 point_lio 的 `blind: 0.5` 对齐可改 0.5 |
| 位姿抖 | 更新间隔太短 | `window.minimum_interval`（0.03 → 0.05） |
| 大场景远处墙不参与 | 目标点数按场景尺度上调有限 | `sampling.scale_threshold` |

其他限制：

- **Phase B 未做**：现在是 `coordinated_reset: False`、backend 仍 `frontend: legacy`。
  也就是说 **reset 收益还没吃到**，本阶段只是"换前端 + 可回滚 + 可 A/B"。
- **实车未接**：`rm_navigation_reality_launch.py` 还没有透传 `use_zlio`，实车参数（IMU 单位 g → `imu_acceleration_scale: 9.81`、
  CustomMsg → `input_type: livox`、`minimum_range`）需要另配一份 `zlio_reality.yaml`。
- **odom 系语义变了**：ZLIO 的 odom 对齐重力，point_lio 是靠 `gravity: [0, 2.539, -9.476]` 把 -15° 安装横滚折进去。
  换过来后 `terrain_analysis` 的 `minRelZ/maxRelZ/vehicleHeight` 建议复调一次。
- **GPL-2.0-only**：ZLIO 作为独立进程运行，不与导航代码链接；分发时注意附源码。

---

## 7. 已验证 / 未验证

**已验证**

- `frontend` 编译通过（3 min 4 s，Release），生成 `zlio_node` / `zlio_visualizer`。
- 桥接节点变换正确性：给定位姿 yaw=90°、t=(1,2,3)，4 个已知点的输出与解析解误差 0.000000 m，时间戳透传。
- `use_zlio:=True/False` 两个分支启动的进程符合预期（True：zlio + bridge；False：pointlio_mapping）。
- 命名空间下参数实际生效：`body_frame=front_mid360`、`input_type=pointcloud2`、`point_time_field=time`、
  `publish_tf=False`、桥接 `frame_topic=frontend/frame`。
- 全栈 `rm_navigation_simulation_launch.py ... use_zloc:=True use_zlio:=True` 起来后节点名与话题名正确：
  `/red_standard_robot1/zlio`、`/red_standard_robot1/zlio_legacy_bridge`、
  `/red_standard_robot1/{frontend/frame,lidar_odometry,registered_scan}`；`loam_interface`、`pointlio_mapping` 均未启动。

**未验证（需要能出图的桌面会话）**

- ZLIO 在真实 Gazebo 数据上的初始化、跟踪精度、CPU。
- `sensor_scan_generation` 的 ApproximateTime 同步在 ZLIO 观测频率下的稳定性。
- `terrain_analysis` 在重力对齐 odom 下的表现。
- 组合模式（`use_composition:=True`）下容器内的 `sensor_scan_generation` / nav2 服务器：
  本机无 GUI 会话的无头冒烟测试里 `nav2_container` 起来了但查不到内部节点（**point_lio 分支同样如此**，
  属于本机 ROS 发现的老问题），需要在正常会话里用 `ros2 node list` 复核一次。

> **一个坑（已修）**：`localization_launch.py` 里的节点不能再写 `namespace=namespace`。
> `bringup_launch.py` 已经在 `GroupAction` 里 `PushRosNamespace(namespace)`，重复指定会变成
> `/red_standard_robot1/red_standard_robot1/zlio`。point_lio 节点本来就是靠继承，新增的 zlio / bridge 也照此处理。

---

## 8. 排查记录：点导航点后车"上天下地"、点云反复重定位（2026-11-03）

### 现象

静止时一切正常（后端 `ratio=0.94 / rmse=0.055 / delta_t≈0.0005 m`），**在 RViz 里点一个导航点后 2~3 秒内**
后端开始 `correction conflict` → `SUSPECT` → 重定位，每次重定位 `delta_t` 报 **18~21 m**，
RViz 里车和点云一起瞬移/翻转，然后无限循环。

### 从日志读出来的四件事

| 证据 | 出处 | 含义 |
| --- | --- | --- |
| `NavigateToPose` 在 `1793695026.9`，第一个冲突在 `1793695029.3` | rviz2 / backend 日志 | 故障与"开始运动"严格同步 → 疑运动相关，不是外参/静态标定 |
| 冲突时 `delta_r_deg=16.2/16.7/17.0/17.2`，`delta_t=0.35~0.9 m` | backend | 预测（里程计）与配准差 16~17°；静止时该值是 0.0005° |
| 重定位 `delta_t=20.9 / 18.3 / 18.2 m`，`descriptor_yaw=-48/-36/-68/-24°` | backend | 锁到了**场地镜像点**（RMUC 前后对称，x≈20.2），每次锁的位置还不一样 |
| `IMU input interrupted` / `PointCloud2 input failed: input event queue capacity exceeded` | zlio 日志 | 前端**跑不到实时**：20 Hz 输入下积压，最终队列溢出 |

关键推理：静止时 `delta_r≈0` 说明外参、坐标系、`body.rotation` 都是对的（否则静止就会差一个固定角）。
16~17° 是**只在运动时出现**的，而且 ≈ 2 ×（1.5 rad/s × 0.1 s）= 17°，指向"时间/延迟"类问题。

### 根因一：逐点时间是真实扫周期的 2 倍（已修）

`ign_sim_pointcloud_tool` 里写死了：

```cpp
new_point.time = (point_id % horizon_scan_) * 0.1f / horizon_scan_;   // 0 ~ 0.1 s
```

而仿真雷达 `update_rate=20`，真实扫周期是 **0.05 s**。ZLIO 用 `header.stamp + time*scale` 给每个点定时间戳
（`apps/ros.cpp` 的 `decodeFloatCloud`），于是点云被当成跨了 100 ms 的扫描去畸变 —— 运动量按 2 倍补偿。
静止时运动量为 0，看不出来；一动就系统性跑偏。

修法：`zlio.point_time_scale: 1.0 → 0.5`（不动 point_lio，保持两边时间口径一致）。
桥接节点现在会**在启动时打印一次 time 字段的真实跨度**，下次跑可以核对。

### 根因二：前端跑不到 20 Hz，位姿变"旧"（已缓解）

桥接日志显示 ZLIO 只跑到 **12~17 Hz**，而输入是 20 Hz；末端掉到 3~4 Hz，最后队列溢出。
位姿落后 60~200 ms，在 1.5 rad/s 的原地转向下就是 9~17° —— 正好对上后端看到的 `delta_r_deg`。

已做的三件事：

| 改动 | 作用 |
| --- | --- |
| `zlio.output_resolution: 0.10 → 0.20` | 导出点云从 ~19k 压到 ~5k，和后端原来吃的密度一致；实测后端单次配准从 0.2~6 s 回到几十毫秒量级 |
| `sampling.minimum_points/maximum_points: 2500/8000 → 2000/5000` | 降低每帧关联开销 |
| `maximum_imu_interval: 0.1 → 0.3` | 负载尖峰不再被误判成"输入中断" |

桥接节点新增每周期一行的 `延迟≤xx ms`：**这是判断前端是否跟得上的唯一硬指标**，
持续 >200 ms 就说明还会复现同样的冲突，得继续降负载（见下）。

### 根因三：SUSPECT 恢复半径 30 m 会锁到镜像点（已收紧）

`recovery.suspect_search_radius: 30 → 10`。RMUC 场地 28 m 长、前后对称，30 m 半径允许一次跨到镜像点；
锁错就是整场平移 20 m，视觉上就是"车飞了"。真被搬走超过 10 m 会落到 LOST，那里仍做全局搜索。

### 已知残留：点时间戳仍在"未来"

工具的 `time` 是 0~0.1 的**正向**偏移，而 `header.stamp` 是 gz 扫描**结束**时刻，
所以缩放后点时间仍落在 `[stamp, stamp+0.05]`，比"当前"晚 0~50 ms。ZLIO 的 IMU 历史覆盖不到，
会退化成"用最新 IMU 样本"去做补偿 —— 比 2 倍时长好得多，但仍不是正确的时间轴。

彻底修法（等这一轮验证完再做，因为会动到 point_lio 的基线）：

1. `ign_sim_pointcloud_tool` 增加 `scan_period` 参数（0.05），
   `time = col/horizon*scan_period`，并且把 `header.stamp` 减去 `scan_period`（对齐到扫描起点）；
2. `point_lio` 的 `lidar_time_inte` 同步改 0.05；
3. ZLIO 的 `point_time_scale` 回到 1.0。

### 下次跑的时候看什么

```bash
# 1) 逐点时间跨度（启动只打一次）：期望 ~50 ms。若仍显示 ~100 ms，说明 scale 没生效
# 2) 每 5 s 一行：Hz / 点数 / 延迟≤xx ms
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True use_zlio:=True 2>&1 | grep -E "zlio_legacy_bridge|延迟"

# 3) 运动中的校正量：健康时 delta_r_deg 应 <2°，delta_t <0.1 m
ros2 topic echo /red_standard_robot1/backend_node/localization_state --once

# 4) 若"延迟"持续 >200 ms，把仿真雷达降到 10 Hz（实车 Mid-360 本来就是 10 Hz）：
#    pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro 里
#    <xmacro_block name="livox" ... update_rate="20" → "10"
#    同时 point_lio 的 lidar_time_inte 与 zlio 的 point_time_scale 需要一起改。

---

## 9. 排查记录二：TF 树裂开、RViz 加载不出来（2026-11-03 16:28）

### 报错

```
local_costmap: Timed out waiting for transform from gimbal_yaw_fake to odom ...
Could not find a connection between 'odom' and 'gimbal_yaw_fake' ... Tf has two or more unconnected trees.
```

### 因果链（不是 TF 配置问题）

`odom → base_footprint` 一直是由 `sensor_scan_generation` 发的，它需要同时收到
`lidar_odometry` 和 `registered_scan` 才会发。这一轮 **ZLIO 在启动 4 秒后就死了**：

```
[ERROR] [zlio]: PointCloud2 input failed: input event queue capacity exceeded
backend: (从头到尾没有 "first synchronized front-end frame")
```

于是 `lidar_odometry` 断流 → `sensor_scan_generation` 一帧都没收到 → 从没发过 `odom→base_footprint`
→ `odom` 这棵树和 `base_footprint/gimbal_yaw_fake` 那棵树彻底不相连 → costmap 超时、RViz 加载不出来。

### 为什么 4 秒就溢出

`InputQueue::push()` 的判定是（`apps/input.cpp`）：

```cpp
if (events_.size() >= maximum_events_ || points > maximum_points_ - queued_points_)
    throw std::length_error("input event queue capacity exceeded");
```

`maximum_points_` 就是 `maximum_buffered_points`，**默认 1,000,000**。仿真一帧 ~6 万点、20 Hz
= 120 万点/秒 —— 缓冲只够 **0.8 秒**。ZLIO 初始化要做 20 帧的联合优化，只要停顿 ~1 秒就必然溢出；
更糟的是队列进入 `Failed` 后**永久不再接收数据**，节点既不退出也不恢复，从外面看就是"静默死亡"。

### 修复

| 改动 | 理由 |
| --- | --- |
| `maximum_buffered_points: 6000000`、`maximum_queue_events: 20000` | 缓冲从 0.8 s 提到 ~5 s，初始化停顿不再致命 |
| ZLIO 节点 `respawn=True` | 万一再死，自动重启（重启后 backend 会重新找 map→odom），而不是整栈静默瘫痪 |
| 仿真雷达 `update_rate: 20 → 10` | **按下暂停键的真正办法**：ZLIO 实测只能跑 12~17 Hz，喂 20 Hz 必然积压。10 Hz 与真实 Mid-360 一致，且与 `ign_sim_pointcloud_tool` 的 0.1 s 时间跨度、`point_lio` 的 `lidar_time_inte: 0.1` 三者自洽 |
| `point_time_scale` 回到 `1.0` | 雷达 10 Hz 后扫周期正好 0.1 s，与工具字段等价，不需要再缩放 |

文件：`config/simulation/nav2_params.yaml`（zlio 块）、`localization_launch.py`（respawn）、
`pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro`（update_rate）。

> 如果要改回 20 Hz，必须**同时**改三处：`update_rate="20"`、`zlio.point_time_scale: 0.5`、
> 并把 `maximum_buffered_points` 保持在 6000000（20 Hz 下缓冲只有 ~2.5 s）。

### 怎么快速判断"是不是前端死了"

```bash
# 前端还活着的时候这两个话题才有频率
ros2 topic hz /red_standard_robot1/lidar_odometry
ros2 topic hz /red_standard_robot1/registered_scan

# 关键：odom→base_footprint 是否存在（sensor_scan_generation 的输出）
ros2 run tf2_ros tf2_echo odom base_footprint

# 看前端有没有报错（正常时这个文件几乎为空）
tail -f ~/.ros/log/zlio_node_*.log
```

---

## 10. 排查记录三：第 8 节判断错的真正根因——仿真是"瞬时快照"，却被当成了 0.1 s 扫掠

### 先说结论

前两节都在改"时间/负载"，方向对但打偏了。真正的根因只有一句：

> **仿真的 `gpu_lidar` 一次渲染出整帧，物理上没有扫描内时间；而 `ign_sim_pointcloud_tool` 人为编了一个 0~0.1 s 的逐点时间，ZLIO 拿它当真，于是"真实运动"被摊到"假时间轴"上，把每帧点云拧成螺旋。**

这跟输入是 20 Hz 还是 10 Hz **无关**——ZLIO 看到的跨度一直是 0.1 s。所以第 9 节把雷达降到 10 Hz 后现象一模一样，这是必然的。

### 三块证据

**1）雷达确实是瞬时快照。** `pb2025_robot_description/resource/models/mid360/model.sdf.xmacro`：

```xml
<sensor name="${prefix}mid360_lidar" type="gpu_lidar">
  <update_rate>${update_rate}</update_rate>
  <ray><scan>
    <horizontal><samples>1875</samples><min_angle>0</min_angle><max_angle>6.2831852</max_angle></horizontal>
    <vertical><samples>32</samples></vertical>
  </scan></ray>
</sensor>
```

`gpu_lidar` 在每个 update 里一次性渲染 1875×32 条射线，输出里**没有时间字段**（所以工具才要自己编）。真机 Mid-360 是"非重复花瓣式扫描"，0.1 s 内逐点出点，两者不是一回事。

工具里那行：

```cpp
new_point.time = (point_id % horizon_scan_) * 0.1f / horizon_scan_;
```

因为点是按"列优先"排的（1875 列 × 32 环），`point_id % 1875` 就是**方位角序号**，所以 `time` 正比于方位角 0→360° 对应 0→0.1 s。

**2）两个前端对"假时间"的反应完全不同。**

| | point_lio | ZLIO |
| --- | --- | --- |
| 怎么用 `time` | 只当作"逐点状态更新的时刻"（`Point-LIO` 的逐点滤波） | 建轨迹后**按时间戳把每个点搬到参考时刻**（真去畸变） |
| IMU 说没动时 | 传播位姿≈不变 → 等价于瞬时扫描，**无害** | 每个点被转动 `(1-φ/2π)·ω·Δt` → **帧内被抹成螺旋** |
| 一动起来 | 仍然等价，只是更新时刻错开 | 点云非刚性，**刚性配准永远拟合不干净** |

这就解释了为什么"point_lio 那套一直好好的"：它本来就免疫。

**3）后端日志的指纹完全对得上。**

健康（静止）：`rmse=0.055  ratio=0.943  delta_r=0.0005°`。
故障（运动）：`rmse=0.196  ratio=0.886  delta_r=12.7°  delta_t=0.447 m  NIS=700~800`，
且 `iterations=32` 迭代到上限仍下不去。

**关键判据：如果是外参、坐标系、地图这类错误，误差一定是"刚性"的，配准能精确拟合（rmse 仍 ≈0.055），只会在位姿上叠一个常量偏置。实测 rmse 涨了 3.5 倍且迭代到底也压不下来 —— 说明点云本身不是刚体。**

顺带排除掉的两个嫌疑（都已核实）：

- **地图不是斜的**：对 `rmuc_2026_bundle/map.pcd`（126773 点）做地面点 PCA，法向 `(2e-05, -2.3e-04, 1.0)`，**与 z 轴夹角 0.000°**，地图是标准重力对齐。
- **时间戳链路自洽**：`FrontendFrame` 的点云与里程计同戳，桥接又强制统一为 `odometry.header.stamp`，实测无 warning。

### 修复

`config/simulation/nav2_params.yaml`：

```yaml
point_time_scale: 1.0  →  0.01
```

跨度从 100 ms 压到 1 ms，去畸变实质变成恒等变换。**只改 ZLIO 一侧，point_lio 的输入一字未动**，A/B 基线完好。

为什么是 0.01 而不是更小：`PointCloudDecoder` 校验 `time_scale > 0`（不能写 0），而 `ImuSegment` 要求轨迹段 `right.time > left.time`（不能退化成长度 0）；1 ms 远小于 5 ms 的 IMU 采样间隔，同时保留非零长度。

连带一处配套调整，`config/simulation/zlio_sim.yaml`：

```yaml
window.minimum_interval: 0.03  →  0.055
```

跨度≈0 后 `Input::nextEnd()` 走"短包"分支（`packet.second - packet.first < minimum` 恒成立），端点只按 `next_interval` 推进、不再被帧边界约束。若下限仍是 0.03，一帧 0.1 s 会被切成 3~4 个观测，同一批点重复进配准，CPU 翻几倍。0.055 让 `next_interval ∈ [0.055, 0.12]`，正好每帧 1~2 个观测（`config` 要求 `2*min ≤ max`，0.11 ≤ 0.12 成立）。

### 新增：在线「前端 vs 真值」指标

`zloc_bridge` 的桥接节点新增 `gt_topic`（默认 `chassis_odometry_gt`，实车没有该话题则静默），
每 2 s 打印一行**相对运动**偏差：

```
[前端 vs 真值] 2.1 s 内 旋转差 9.63° | 位移差 19.4 cm | 真值走过 106.0 cm / 96.3°
```

用相对量而非绝对位姿，是因为前端 odom 原点与真值原点不重合、雷达还相对底盘有安装偏移，
绝对比对会混进一个恒定外参误差；相对运动把这个常量消掉了。

离线验证过指标本身是正确的（`bridge_gt_check.py`）：

| 用例 | 注入 | 报告 |
| --- | --- | --- |
| A 恒定外参 | 前端 = 真值 × 固定外参 | `0.00° / 0.0 cm` |
| B 偏航放大 10% | 真值转过 96.3° | `9.63°`（理论 9.6°） |

### 下一次跑，只看这一行

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True use_zlio:=True 2>&1 | grep -E "前端 vs 真值|延迟"
```

决策规则很干脆：

- **运动时「旋转差」< 2°、位移差 < 10 cm** → 前端是干净的，剩下的抖动/重定位全在后端或 legacy 桥接，往那个方向查。
- **仍然 10° 量级** → 说明去畸变之外还有别的东西在动点云，下一步直接把 `point_time_scale` 设 0.0001 做极限对照（若不变，就与扫描内时间无关）。

---

## 11. 更正：前端是准的，第 10 节之后加的「前端 vs 真值」指标本身写错了

### 我错在哪

第 10 节末尾加的诊断行是这么算的：

```python
d_odom = inv(odom0) @ odom_t      # 前端相对运动
d_gt   = inv(gt0)   @ gt_t        # 真值相对运动
err    = inv(d_gt) @ d_odom       # 两者的"差"
```

看着没问题，但它默认了**两路位姿描述的是同一个坐标系**。实际不是：

- 前端 `lidar_odometry` 的 child_frame 是 `front_mid360`（雷达/IMU 系）；
- 真值 `chassis_odometry_gt` 来自 `MecanumDrive2`，读的是 `components::WorldPose` 的 **chassis** 系（这一点已在 `MecanumDrive2.cc:UpdateOdometry` 里确认，是真位姿不是轮速）。

两者相差 URDF 里的固定安装：`pose="0.0496 0.0936 0.435 -pi/12 0 0"`，即**横滚 -15°**。
同一段物理相对运动在两个系下互为共轭 `d_lidar = Mᵀ · d_chassis · M`，所以
`inv(d_gt)·d_odom` 根本不是单位阵，它的转角会**随转过的角度增长**。

### 拿实测数据验一下

设 `M = Rx(-15°)`，真值转过 θ 时这个指标会"凭空"报出：

| 真值转角 θ | 预测表观误差 | 实测 |
| --- | --- | --- |
| 3.2° | 0.84° | **0.87°** |
| 31.0° | 8.00° | **8.47°** |
| 36.6° | 9.40° | **8.89°** |
| 104.6° | 23.71° | **23.57°** |

四个点几乎完全吻合。**结论：那 8~23° 的"误差"全部是雷达安装横滚在这个指标里的假象，
ZLIO 的里程计是准的。** 第 10 节里"前端自己就是错的"这个判断作废。

### 指标已改成坐标系无关

共轭不改变**旋转角**和**位移模长**，所以只比这两个标量：

```python
a_odom = angle(d_odom[:3,:3]);  a_gt = angle(d_gt[:3,:3])
l_odom = |d_odom[:3,3]|;        l_gt = |d_gt[:3,3]|
```

输出格式：

```
[前端 vs 真值] 2.0s 转角 96.26°/真值 96.26° (差 +0.00°) | 位移 106.0/真值 106.0 cm (差 -0.0 cm)
```

离线验证（`src/zloc_bridge/test/bridge_gt_check.py`，见第 10 节表格）：

| 用例 | 注入 | 新指标报告 |
| --- | --- | --- |
| A 恒定外参 | 前端 = 真值 × 固定外参 | `差 +0.00° / -0.0 cm` |
| B 偏航放大 10% | 真值转过 96.3° | `105.88/96.26 (差 +9.63°)` |

### 这轮改动怎么处理

| 改动 | 是否保留 | 理由 |
| --- | --- | --- |
| `zlio.point_time_scale: 1.0 → 0.01` | **保留** | 仿真雷达确实是瞬时快照，去畸变本身就该是恒等；且 22:29 那次"前端 vs 真值"就是在 `0.01` 下测出准的 |
| `window.minimum_interval: 0.03 → 0.055` | **保留** | 同上，跨度≈0 时避免一帧被切成 3~4 个观测 |
| 桥接的「前端 vs 真值」 | **保留（已修正）** | 现在这个数字是可信的 |
| 第 9 节的 `maximum_buffered_points` / `respawn` | 保留 | 与本次问题无关，但仍是正确的防护 |

### 结论：嫌疑重新回到后端，但需要数据才能定案

前端已排除、地图已确认水平、时间戳链路自洽、外参一致，剩下的差异只有一条：

> **point_lio 的 `camera_init` 是"初始 IMU 系"（相对重力倾斜 15°），ZLIO 的 `odom` 是重力对齐的。**
> 后端 `body.rotation.roll_deg: -15` 对两者都自洽（已核 `src/backend/body.cpp` 的变换方向与 offset 符号），
> 但 `map→odom`、`odom→base_footprint` 这两条链在两种输入下的行为差异，只能靠实测对比。

本机跑不了仿真（无 GPU/X：Ogre2 软件 EGL 在 `Hlms::createDatablock` 段错误，Ogre1 需要 X 显示），
所以提供了采集脚本：

```bash
bash tools/capture_zlio_bag.sh        # 默认 20 s，包在 bags/zlio_<时间>/
```

采 `/livox/imu`、`/livox/lidar`、`/velodyne_points`、`/lidar_odometry`、`/registered_scan`、
`/chassis_odometry_gt`、`/tf`、`/clock`，够在包上离线重跑前端和后端，把最后这一环钉死。

---

## 12. 真根因（第四次排查，有真值数据支撑）：仿真 IMU 不可信，ZLIO 积分放大成"上天下地"

前面的第 8~11 节都在外围打转。这一轮用自己采集的包做了离线复现，结论只有一句：

> **Gazebo 的 IMU 一旦底盘被驱动就不可信，而 ZLIO 对加速度做二次积分，把误差放大成米级/百米级平移漂移。**

### 12.1 两个传感器缺陷（实测）

用 `chassis_odometry_gt`（MecanumDrive2 直接读 `components::WorldPose`，1000 Hz）对照：

| 缺陷 | 数据 | 判定 |
| --- | --- | --- |
| 单样本加速度尖峰 | 每隔约 0.25 s 出现一帧模长 50~**606** m/s²，前后样本都在 9.8 附近 | 尖峰时刻真值速度只变化 **0.005~0.02 m/s**；606 m/s² 持续 5 ms 本该带来 3 m/s → 是派生量的采样伪迹，不是真实运动 |
| 运动时模长系统性偏低 | 静止 `\|a\|≡9.80`，开动后中位数掉到 **~9.5** | 约 **0.35 m/s²** 的持续偏置 → 二次积分后 z 轴约 −0.4 m/s² 的漂移 |

关键判据：**陀螺是干净的**。所以 ZLIO 的旋转始终正确（实测误差 <1°），只有平移崩 —— 这正是
"小车不转错、只上天下地、点云反复重定位"的物理含义。

### 12.2 离线复现（同一段数据，只换 IMU）

`ros2 bag` 采 16 s（含原地转 + 直行），再用真实 `zlio_node` + 真实 `zlio_sim.yaml` 离线回放：

| IMU 来源 | 相对位移最大偏差 | odom z |
| --- | --- | --- |
| 原始 `/livox/imu` | **3187 cm** | 漂到 ±30 m |
| 尖峰门限（剔除 + 保持上一帧） | **33 cm** | ±0.05 m |
| 由真值合成的物理自洽 IMU | **20 cm** | **±0.03 m** |

原始 IMU 的 z 曲线：`0 → −0.25 → −1.23 → −3.81 → −9.65 → +28 m`，和现场 RViz 现象一致。

### 12.3 现场实测（修复前 / 修复中）

接入尖峰门限后：位移误差从 **2208 cm 降到 5~16 cm**，但 28 s 内 z 仍单调漂到 **−194 m**
——因为第 12.1 节第二条（模长偏低 0.35 m/s²）没被处理，门限只治尖峰、治不了偏置。

由真值合成的 IMU 离线验证：z 稳定在 **0.00~0.03 m**，相对位移偏差 20 cm。

### 12.4 修复：`zloc_bridge/scripts/imu_guard.py`（双模式）

```
仿真（有真值）  订阅 chassis_odometry_gt（1000 Hz），每 5 ms 重算一帧：
                ω = 真值角速度
                a = R_imu^T · (dv_world/dt + [0,0,9.81])   ← IMU 测的是比力
                q = R_chassis · Rx(-15°)                   ← 与 livox pose 一致
实车（无真值）  2 s 内收不到真值 → 自动退化为「跳变门限 + 保持上一帧」直通滤波
```

接线（`localization_launch.py`，仅 `use_zlio:=True` 时生效）：

```
livox/imu ──┬─→ point_lio（原始，基线不变）
            └─→ imu_guard ─→ livox/imu_guarded ─→ zlio
```

`use_zlio:=False` 时该节点不启动，point_lio 的输入一字未改。

### 12.5 仍未解决：前端吞吐

这台机器 16 核、`Gazebo` 常驻 250% CPU，ZLIO 实测只跑到 **4~5 Hz**（输入 10 Hz）、
输出延迟 300~600 ms。延迟 >200 ms 时后端仍会偶发 `correction conflict → SUSPECT`（但
delta_t 已是厘米级，不再是 18~21 m 的镜像点跳变）。要根治需要：
`output_resolution` 继续加大到 0.25、或把仿真雷达降到 5 Hz、或给 ZLIO 单核绑核。

### 12.6 与本问题无关的一次环境事故（2026-11-04 11:00）

后台 `unattended-upgrade` 自动升级了 NVIDIA 驱动，导致：

| | 版本 |
| --- | --- |
| 已加载内核模块 | `580.126.09` |
| 用户态库 `libnvidia-gl-580` | `580.178.04` |

版本不一致 → 新建 OpenGL 上下文失败（`X Error: GLX BadValue`），**Gazebo GUI 与 RViz2
都起不来**。`dpkg --audit` 干净，说明包已装完，只差重新加载内核模块。

处理：**重启机器**（重启后加载 580.178，GUI 恢复）。这与 ZLIO 替换无关，任何需要
GPU/GL 的程序都受影响。

---

## 13. 决策（2026-11-04 11:25）：停止替换，保留资产，实车再评估

### 13.1 一句话结论

**ZLIO 没有"废"，它的点云管线是干净的（离线回放相对位移偏差 20 cm，与 point_lio 同档）。
但它现在不是一个等价的替换选项 —— 它把系统对 IMU 的依赖从"软"变成了"硬"，
而当前仿真恰好提供了一个坏 IMU，于是这个新增风险被提前暴露了。**

### 13.2 分清哪些是 ZLIO 的问题，哪些是仿真的问题

| 现象 | 归属 | 依据 |
| --- | --- | --- |
| 旋转始终正确（<1°） | — | 陀螺干净，任何配置下都成立 |
| 静止时后端 rmse 0.055 / delta 0.0005 | — | 静止无积分，IMU 缺陷不显形 |
| odom 平移漂到 ±30 m / −194 m | **仿真的 IMU** | 同一包只换 IMU：原始 3187 cm → 真值合成 20 cm |
| 点云反复重定位 / 上天下地 | **仿真的 IMU**（经 ZLIO 放大） | z 轴发散直接来自加速度模长偏低 0.35 m/s² |
| 输出只有 4~5 Hz、延迟 300~600 ms | **待定**（机器负载为主，但 ZLIO 本身也不轻） | Gazebo 常驻 250% CPU，load 17.85/16 核 |
| 队列溢出后节点静默失效 | **ZLIO 的缺陷** | `InputQueue` 进 Failed 后永久不收数据也不退出 |

所以"乱飞"这件事上 ZLIO 是无辜的，它是那个把仿真 IMU 的谎言如实积分出来的受害者。

### 13.3 但"无辜"不等于"该用"——四条与 IMU 无关的理由

**1）换它的三个理由，一个都没兑现。**

| 卖点 | 实际状态 |
| --- | --- |
| reset 闭环 | 要 Phase B（`frontend: zlio` + `coordinated_reset`）才有，Phase A 没吃到；而进 Phase B 的前提是先在仿真里证明前端可信 —— 现在正好卡在这 |
| 重力对齐 odom | 可以单独在 point_lio 侧解决（对 odom 做一次重力对齐，或让 backend 吸收那 15°），不值得为此换整个前端 |
| CPU 减半 | 这台机器上反过来：ZLIO 4~5 Hz 追不上 10 Hz 输入，point_lio 稳跑 |

**2）它新增了一个 point_lio 架构上没有的脆弱性。**

point_lio 把 IMU 当**量测**，且 `check_satu` 把竖直加速度通道实质关掉了 —— 它是"不信加速度、只信陀螺和点云"。
ZLIO 用**预积分因子**，窗口内的原始样本被直接积成位姿约束 —— IMU 有多差，它就有多差。

实车上挨撞、上坡下坎、电机振动、磁力计干扰，这些都会进预积分。**这是替换带来的新增风险，
不是等价替换。** 仿真只是提前把这条暴露了，实车不会让这个性质消失，只会让它更少见。

**3）工程成熟度不足，且有实测脆性缺陷。**

单人维护、无论文、作者声明不再更新、GPL-2.0-only。更实际的是：输入队列满后进入 Failed
状态，**永久不再接收数据也不退出**（外部只能看到 TF 断掉）。我们只能靠 `respawn=True` 兜。
这个 bug 在实车负载波动时同样会触发。

**4）初始化有门槛。** 需要 20 帧窗口（10 Hz 下 2 s）+ 视野内 ≥10 个平面。出生点朝墙或空旷区可能起不来，
point_lio 几乎首帧就出 odom。

### 13.4 决策

| 项 | 处理 |
| --- | --- |
| 默认前端 | **保持 point_lio**（`use_zlio` 默认 `False`，backend `frontend: legacy`，均已是现状） |
| point_lio | **保留，不删**（唯一已知能跑的 baseline） |
| `use_zlio` 开关 / 桥接 / imu_guard | **全部保留**，作为资产。开关翻转即回 A/B，零成本 |
| Phase B（`frontend: zlio` + `coordinated_reset`） | **不启动**。前置条件（仿真里 A/B 达标）未满足 |
| 仿真 IMU 修复 | **不做**。这是仿真器缺陷，修它只是让仿真更真，无助于判断 ZLIO 是否该上实车 |

### 13.5 什么条件下重新评估

只需要一个条件：**拿到一段实车 bag**（Mid-360 真实 IMU + 真实点云 + 一段带转弯和坡道的轨迹）。

判据用现成的：把 bag 离线喂给 `zlio_node`，看桥接节点那行
`[前端 vs 真值] ... 旋转差 x° | 位移差 x cm`（实车没真值就退化用 `chassis_odometry` 的相对量），
以及 `output_resolution=0.2` 下能否稳定跑进 100 ms。

- 若实车数据下 ZLIO 与 point_lio 同档或更好 → 那时 Phase B 的 reset 收益才是真的，值得做；
- 若同样对 IMU 敏感 → 说明这是 ZLIO 的架构性质，不是仿真特例，正式结论为"不适合本平台"。

### 13.6 一句话给后来人

> 这次替换的教训不是"ZLIO 不行"，而是**"在仿真里评判一个依赖 IMU 预积分的 LIO，前提是仿真 IMU 可信；
> 而 Gazebo 的 IMU 在底盘被驱动后并不可信。"**
> 下次换任何前端之前，先用 `chassis_odometry_gt` 验一遍仿真 IMU 的模长与尖峰，20 分钟就能避免十几轮排查。
