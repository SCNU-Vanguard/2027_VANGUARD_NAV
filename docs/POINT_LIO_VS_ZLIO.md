# Point-LIO → ZLIO 前端替换评估

> 评估对象：`src/pb2025_sentry_nav/point_lio`（HKU Point-LIO 的 RM25 fork）→ `src/z-localization/src/frontend`（东北大学 T-DT 的 ZLIO）
> 评估日期：2026-11-03　当前状态：`map→odom` 已由 zloc **backend**（legacy 模式）接管，前端未动
> 结论先说：**两者精度打平，换的价值在于工程配套（reset / 重力对齐 / CPU），不在精度。** 建议按"可插拔 + A/B"推进，不要直接删掉 point_lio。

> **实施状态（2026-11-03）**：Phase A 已落地 —— `use_zlio:=True/False` 切换前端，backend 与 Nav2 零改动，
> 桥接节点把 `FrontendFrame` 适配成 legacy 的 `registered_scan`。用法与限制见
> [`docs/ZLIO_FRONTEND_SWITCH.md`](ZLIO_FRONTEND_SWITCH.md)。Phase B（`frontend: zlio` + `coordinated_reset`）尚未开始。

---

## 0. 决策速查

| 如果你的痛点是… | 该不该换 | 说明 |
| --- | --- | --- |
| 定位飞了要重启节点、重新找图 | **换** | ZLIO 支持 warm reset + 与 backend 协调 reset，point_lio 只能整节点重启 |
| 静止时 odom 抖、地面分层、costmap 抖 | **换** | Point-LIO 逐点更新在低速下抖动明显（ZLIO 作者本人的原话） |
| 算力吃紧（Orin 上还要跑视觉/自瞄） | **换** | 作者自测 CPU 15% vs 35.5%（单核%），内存 144 vs 166 MB |
| 场地空旷/长走廊、几何退化 | **都别指望前端** | 两者的退化处理都在 backend（可观测性 + 权重），前端都是 ESKF |
| 现在实车已经稳定、没人有空调参 | **不换** | 引入的是新 bug 风险，收益是"更好的架构"，不是"更好的成绩" |
| 需要 BSD 许可做闭源/商业化 | **不换** | ZLIO 是 GPL-2.0-only，point_lio 是 BSD-3-Clause |

---

## 1. 先把两个东西摆正

**它们是同一类算法，不是范式替换。** 都是紧耦合 LiDAR-IMU ESKF + 点到面残差 + 体素平面地图。换前端不会改变系统架构，只改变"更新粒度、地图实现、初始化方式、工程接口"。

| | Point-LIO（本仓库 fork） | ZLIO |
| --- | --- | --- |
| 出身 | HKU-MARS，AIS 2022 论文，社区 fork 极多 | 东北大学 T-DT，2026 RM 哨兵开源，**无对应论文** |
| 代码规模 | 约 1 万行 C++（含 fork 的 prior_pcd 分支） | 约 9 千行 C++17，带 unit/integration/ros 测试 |
| 许可 | BSD-3-Clause | GPL-2.0-only |
| 维护 | 上游稳定，fork 分支自维护 | 作者自述"大概率不会再更新，除非重大 bug" |
| 定位 | 单文件式 LIO，自成一体 | **为 zloc backend 量身定制的前端**，与后端是成对设计 |

关键差异一句话：**Point-LIO 是"一个独立 LIO"，ZLIO 是"一套定位系统的前半部分"。** 你已经把后半部分（backend）接进来了，所以这个差别的价值被放大了。

---

## 2. 算法逐项对比

| 维度 | Point-LIO | ZLIO | 对哨兵的实际影响 |
| --- | --- | --- | --- |
| 误差状态建模 | SO(3) 直积流形（旋转乘性、其余加性） | SE₂(3) ⋉ se(3) 半直积，把导航状态与零偏一起纳入 | ZLIO 姿态/速度误差的更精确线性化，理论上大角速度下线性化误差更小；工程上感知不强 |
| 更新粒度 | **逐点**更新，出 4~8 kHz 位姿 | 点云分段，**自适应 30~120 ms** 一次更新（8~33 Hz）+ IMU 频率预测输出 | Point-LIO 冗余带宽对 nav2（≤50 Hz）无用；ZLIO 更平滑，但单次校正延迟更大 |
| 去畸变 | 逐点反向传播，区间内按匀速假设 | DLIO 式连续时间分段轨迹（位置三次多项式、姿态积分角速度） | 云台/底盘变速时 ZLIO 的畸变补偿更准 |
| 地图 | iVox 自适应体素 + 原始点 | VoxelMap 概率平面 + 一阶/二阶矩统计 | ZLIO 官方数据：CPU 降 62~73%、内存降 87~91%（相对原 VoxelMap 实现，不是相对 Point-LIO） |
| 残差建模 | 点到面，简单协方差 | 点到面 + **平面参数协方差 + 表面散布 τ² + 同平面残差相关性** | ZLIO 不会因为"点打得多"就过度相信某个平面，长走廊/薄墙更稳 |
| 初始化 | 首帧 IMU 估重力，**立即出 odom** | **20 帧窗口** + 多帧共面约束 + IMU 预积分联合优化，**允许非静止启动** | Point-LIO 起得快；ZLIO 起得准（云台在转也能起），代价是 1~2 s 空窗 + 需要 ≥10 个平面 |
| IMU 饱和 | 把 IMU 当"状态"而非"输入"（`use_imu_as_input: False`）+ 饱和裁剪 → **强项** | 标准 ESKF，IMU 是输入；没有专门的饱和处理 | 哨兵车速 3~4 m/s、雷达装在底盘不跟云台转，这条优势基本吃不到 |
| 退化场景 | 声称"不会飞"，但无可观测性度量，退化时默默漂 | 前端不专门处理，**由 backend 做可观测性 + 白化 + NIS** | 这是架构选择：ZLIO 把这件事放在正确的位置（你已经在用） |
| 在线动态点滤除 | 无 | 无（作者明确放弃，改做离线清理工具） | 打平 |
| odom 系 | `camera_init` = 初始 IMU 系，**不对齐重力** | **对齐重力**（init 后把坐标系旋到重力方向） | 你仿真的雷达是 30° roll 装的，point_lio 的 odom 一直是个"斜坡系"，terrain_analysis 靠 `useSorting` 硬扛；ZLIO 的 z 轴是真竖直 |
| 输出协方差 | **不填**（Odometry 里 covariance 全 0） | 输出状态协方差 + 逐观测元数据（matches / lidar_updated / epoch） | legacy 模式下 backend 只能用固定过程噪声反推可信度 |

### 数字证据（重要，也要打折扣）

ZLIO 作者的 13 方法对比（IndoorOffice1，Mid-360，三次运行取中位数）：

| 方法 | 参考位置 MSE | 平均 CPU | 峰值内存 |
| --- | --- | --- | --- |
| RKO-LIO | 3.07 cm | 7.5% | 95.3 MB |
| Super-LIO | 2.61 cm | 12.6% | 126.2 MB |
| **ZLIO** | **2.54 cm** | **15.0%** | **143.8 MB** |
| FAST-LIO2 | 3.04 cm | 17.4% | 220.7 MB |
| Faster-LIO | 2.55 cm | 22.7% | 133.9 MB |
| **Point-LIO (iVox)** | **2.47 cm** | **35.5%** | **166.3 MB** |
| PV-LIO | 2.54 cm | 106.1% | 1679.5 MB |

**必须打的三个折扣**：

1. 作者自测、单一数据集、自己调参、无第三方复现、无同行评审 → 只能当"量级参考"，不能当承诺。
2. 精度项 **2.47 vs 2.54 cm** 属于同一档，**"为了更准"不是替换理由**。
3. 场景是室内办公室，不是你 28×15 m 的 RM 场地；场地里墙少、遮挡多、有人跑动，两者的相对表现可能换位。

---

## 3. 放到你这套系统里看（这才是决定性的部分）

### 3.1 现状链路

```
point_lio ──/cloud_registered──┐
          └─/aft_mapped_to_init─┴─► loam_interface ─┬─► registered_scan ─► terrain_analysis ─► Nav2
                                                    ├─► lidar_odometry ─► sensor_scan_generation ─► odom→base_footprint TF
                                                    └────────────────────► zloc backend (legacy 模式)
                                                                              └─► map→odom TF
```

### 3.2 point_lio 在这个位置上的三个隐性成本

1. **没有 reset 能力。** backend 判定 LOST 时只能冻结 TF，**无法重置前端**。你只能重启 `pointlio_mapping`，`camera_init` 归零，odom 跳变，backend 重新 anchor。比赛中这段是几秒到十几秒的盲区，而且重启后雷达姿态已变，初始化质量还不保证。
2. **信息贫乏。** odom 无协方差、无 epoch、无"本帧激光更新是否生效"的标记。backend 只能靠 NIS 反推这次校正该不该信。
3. **静止抖动。** 逐点更新在低速下的抖动是公认问题（ZLIO 作者正是因为这个才把更新间隔做成 30~120 ms 自适应）。下发给 terrain_analysis 的 `registered_scan` 一旦抖，地面高程分层、`terrain_map` 边缘就抖，costmap 跟着抖。

### 3.3 换 ZLIO 能直接拿到的三件事

1. **reset 闭环。** 前端 warm reset 只清位置和 yaw，**保留速度、倾角、IMU 零偏**；配合 backend 的 `coordinated_reset`（冻结输出 → 事务化重置 → 在边界续接），恢复时不用重新初始化，backend 只需重找 `map→odom`。对"被撞/被判罚搬动/开机位置不对"这些场景，这是实打实的收益。
2. **重力对齐的 odom。** 顺带修正你现在"雷达 30° roll 安装 → odom 是斜的"这个隐患，terrain_analysis 的 `minRelZ/maxRelZ/vehicleHeight` 才回到设计语义上。
3. **CPU 减半。** backend 已经占了约 24% 单核，前端从 35% 降到 15% 是能明显改善整机余量的。

### 3.4 换掉会失去什么

| 失去的 | 严重程度 | 备注 |
| --- | --- | --- |
| 逐点更新的极端运动冗余 | 低 | 哨兵不跑 75 rad/s；雷达在底盘上不随云台高速转 |
| 首帧立即出 odom | 中 | ZLIO 要 20 帧窗口（仿真 20 Hz ≈ 1 s）+ ≥10 平面 + 最弱方向支撑 ≥15；出发前多等 1~2 s |
| point_lio 内置 `prior_pcd` 先验图模式 | 低（其实是好事） | 和 backend 是重复且冲突的设计（ZLIO 设计文档第 1.2 节专门批评了"先验图塞进紧耦合 LIO"） |
| BSD-3 许可 | 中 | GPL-2.0-only。当前是跨进程调用，风险可控；要闭源分发就得重新评估 |
| 成熟度/可查资料 | **中高** | Point-LIO 有论文、有大量复现、bug 基本被踩平；ZLIO 是单人项目、AI 整理过、作者声明不再更新 |

---

## 4. 更换的实际工作量

### 4.1 编译（0.5 天）

- `src/z-localization/src/frontend/COLCON_IGNORE` 去掉；`bringup` 保持忽略（会和导航栈重复起驱动/RViz）。
- 依赖：Eigen3 / yaml-cpp / nanoflann / OpenMP / `localization_interfaces`（已编）/ `livox_ros_driver2`（工作区已有，注意是导航项目那份 1.0.0）。
- **不需要** GTSAM（那是 backend 的）。

### 4.2 配置（`config/lio.yaml`，1 天）

| 项 | 注意 |
| --- | --- |
| `sensor.body_from_lidar` | 只要 **LiDAR→IMU**（Mid-360 内部），和整车安装姿态无关 —— 这是 ZLIO 相对 point_lio 的一个好处，仿真/实车安装差异不影响前端 |
| `imu_acceleration_scale`（节点参数） | 仿真 IMU 是 m/s² 取 `1.0`；实车 Livox 驱动是 g，取 `9.81`。填错的表现是"初始化就飞" |
| `sensor.minimum_range` | 默认 0.5；你仿真雷达 min 0.1，建议先按 0.2~0.3 |
| `sampling.minimum_points/maximum_points` | 默认只留 1k~4k 点。仿真一帧 1875×32≈60k 点，默认会把点砍太狠 → 先抬到 6k~10k |
| `window.maximum/minimum_interval` | 0.12/0.03 s。仿真 20 Hz（50 ms/帧）下等于一帧一到两次更新，正常 |
| `initialization.window_frames` | 20 帧。仿真 20 Hz ≈ 1 s；别为"起得快"调到 5，速度和零偏会不可观 |

### 4.3 桥接（推荐分两阶段，2~3 天）

**Phase A：ZLIO 走 legacy，与 point_lio 做 A/B（backend 配置零改动）**

写一个 `zloc_bridge` 节点（或改造 `loam_interface`）：订阅 `/frontend/frame`，把局部点云按同帧 odom 变换到 odom 系，发 `registered_scan` + `lidar_odometry`。

两个必须对上的细节：
- `lidar_odometry.header.child_frame_id` 必须等于下游 `sensor_scan_generation` 的 `lidar_frame`（`front_mid360`），或者把那边改成 `imu` 并补 `imu→lidar` 静态变换。差 3 cm 和 0°/90° 的旋转，错了表现为"点云整体偏一点"。
- 点云无 intensity —— `terrain_analysis` 自己会写 intensity，`pointcloud_to_laserscan` 无该字段时不做强度过滤，**都能兼容**，不用改。

**Phase B：backend 切 `frontend: zlio`**（拿到 reset 闭环，去掉 loam_interface 与"odom/点云分离输入"）
- `common.frontend: zlio`、`sensor_frame: imu`、`input_cloud_in_local_frame: true`、`frame_topic/prediction_topic/session_topic`。
- 前端节点参数 `coordinated_reset: true`（统一 bringup 默认开）。
- 下游仍需要 `registered_scan`/`lidar_odometry` 喂 terrain_analysis → 桥接节点保留，只是数据源从 legacy 话题改成 `/frontend/frame`。

### 4.4 调参与验证（3~5 天）

- 重力对齐后 `terrain_analysis` 的 `minRelZ/maxRelZ`、`vehicleHeight` 需复调（现在的参数是在"斜坡 odom"下调出来的）。
- 仿真雷达安装姿态核对结论：真正被 `spawn_robots.launch.py` 使用的模型是
  `pb2025_sentry_robot.sdf.xmacro`，livox 为 `pose="0.0496 0.0936 0.435 -pi/12 0 0"`，即 **roll −15°**，
  与 `point_lio` 里 `gravity: [0, 2.539, -9.476]` 和 `zloc_backend.yaml` 的 `body.rotation` 一致。
  仓库里另有一个 `simulation_robot.sdf.xmacro`（+30° / +90°）**未被使用**，不要拿它当依据。
- 别做的事：同时开 point_lio 的 `prior_pcd` 和 backend；把 ZLIO 的 odom 直接当 map 系用；用 `bringup.launch.py` 起整套。

---

## 5. A/B 判据（跑同一段仿真轨迹 + 同一段实车数据）

| 指标 | 测法 | 通过线 |
| --- | --- | --- |
| 终点漂移 | 仿真真值 vs odom，3 m/s 绕场 3 圈 | ≤ point_lio 的 1.1 倍 |
| 静止漂移 | 静止 5 min 的位置/角度变化 | < 2 cm / 0.5° |
| 平滑度 | `registered_scan` 相邻帧同一栅格高度差 p95 | ≤ 3 cm，且优于 point_lio |
| backend 健康度 | `localization_state` 的 SUSPECT/LOST 次数、日志 `ratio`/`rmse` | SUSPECT 次数 ≤ point_lio 的 1.2 倍；`ratio ≥ 0.85` |
| 算力 | 单核 CPU%、峰值内存 | CPU ≤ point_lio；内存 ≤ 250 MB |
| 启动 | 冷启动到首个 `/odometry` | < 3 s |
| 恢复 | 触发 `/localization_reset` 到 `NORMAL` | < 2 s，且 TF 跳变 < 0.1 m |
| 异常恢复 | 杀前端 / 断输入，观察 backend 是否自发恢复 | 不产生 > 0.1 m 的 TF 跳变 |

任一项不达标 → 保留 point_lio，把 Phase A 的桥接留着（它本身也有价值：让前端变成可插拔）。

---

## 6. 风险清单

**技术风险**
1. 初始化条件：出生点面向单面墙 / 视野内平面 < 10 时可能起不来或慢；仿真出生点在场地角落，**必须实测**。
2. ZLIO 在 20 Hz + 200 Hz IMU 的仿真配置下未经验证（作者是 10 Hz Mid-360）；自适应窗口和采样参数可能需要重调。
3. 单人维护、无论文、声明不再更新 → 踩到 bug 基本只能自己修（GPL 允许，但要有人力）。
4. 两套自研栈（backend + frontend）叠加后，出问题的定位面变大：是前端漂了、还是后端拒了、还是外参错了。

**工程风险**
5. 配置项从 point_lio 的 ~50 项变成 ZLIO 的 ~80 项 + backend 的 ~440 行 yaml，调参心智负担上升。
6. 你在仿真里调的 ZLIO 参数搬到实车要重来（雷达高度、外参、点云密度、blind 都变）。

**许可风险**
7. GPL-2.0-only：跨进程运行、不静态链接，通常不传染你的导航代码；但**分发**（给友队、上开源仓库）时 ZLIO 部分必须附源码。现在的 backend 已经是 GPL，风险等级没有变化，只是又多了一个。

---

## 7. 结论

**建议换，但换的理由要摆正：不是为了更准（2.47 vs 2.54 cm，打平），而是为了"和 backend 配成一套"——reset 闭环、重力对齐、CPU 减半。**

推进方式（和你上次换 backend 一样的思路，保留回滚能力）：

1. **先插拔，不删代码**：加 `use_zlio:=True/False`，两条前端路径共用同一个 backend（Phase A 的 legacy 桥接保证这一点）。
2. **A/B 达标再切协议**：Phase B 打开 `frontend: zlio` + `coordinated_reset`，这时候才真正吃到 reset 收益。
3. **实车验证前不删 point_lio**：point_lio 是"已知能跑"的对照系，删了就没有 baseline 了。

**如果你们现在没有"重启恢复 / 抖动 / 算力"的具体痛点，那这次替换的期望收益小于引入新 bug 的代价** —— backend 在 legacy 模式下已经能工作，point_lio 继续用是完全合理的选择。这种情况下更值得先做的是把 A/B 桥接写出来（成本低、收益独立），把选择权留在手上。

---

## 附：参考

- Point-LIO 论文：*Point-LIO: Robust High-Bandwidth LiDAR-Inertial Odometry*, AIS 2022
- ZLIO 设计说明：`src/z-localization/docs/design.md`（第 2 章：误差状态、初始化、去畸变、地图、面建模、时空自适应）
- ZLIO 参数：`src/z-localization/src/frontend/docs/parameters.md`、`config/lio.yaml`
- 已完成的 backend 替换：`docs/ZLOC_USAGE_AND_COMPARISON.md`、`docs/ZLOC_INTEGRATION_HANDOFF.md`
