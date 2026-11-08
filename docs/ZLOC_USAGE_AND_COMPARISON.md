# Z-Localization 定位后端：使用说明与 small_gicp 对比

> 适用仓库：`/home/l/ros_ws`（pb2025_sentry_nav + src/z-localization）
> 状态：**仿真已跑通并稳定**（2026-11-03），实车尚未验证
> 相关文档：`docs/ZLOC_INTEGRATION_HANDOFF.md`（实施细节与故障排查）

---

## 1. 它替换了什么

原来 `map→odom` 由 `small_gicp_relocalization` 发布，它把当前累积点云和整张先验 PCD 做一次 GICP 迭代，
收敛与否都直接广播。现在改为由 Z-Localization 的 **backend** 发布：

```
point_lio ──► loam_interface ──┬─► /lidar_odometry  ─┐
                               └─► /registered_scan ─┴─► backend（legacy 模式）
                                                          │
                                          ┌───────────────┼───────────────┐
                                          ▼               ▼               ▼
                                   map→odom (TF)   定位状态(String)    位姿+协方差
                                          │
                    zloc_bridge/map_odom_fallback（backend 冻结时接管）
```

**关键点**：point_lio 与前端完全不动，只换了 `map→odom` 的来源。因此这是可逆改动，
去掉 `use_zloc:=True` 即恢复原状。

backend 内部做的事（与 small_gicp 的本质差别就在这里）：

| 环节 | 内容 |
| --- | --- |
| 局部配准 | 从先验 bundle 按当前位姿取 12 m / 16 关键帧子图，点到面配准 + 平面有效性验收 |
| 融合 | GTSAM 因子图，里程计因子 + 先验观测因子，NIS 卡方检验决定接受/丢弃 |
| 全局重定位 | Scan Context 描述子召回 → 多 yaw 种子几何验证 → 一致性检查 |
| 状态管理 | `INITIALIZING / NORMAL / SUSPECT / OBS_DEGRADED / LOST` + `pose_trusted` + `tf_frozen` |
| 输出 | 位姿 + 6×6 协方差 + 状态字符串；只有可信时才发 TF |

---

## 2. 前置工作（换机器必做）

### 2.1 依赖：GTSAM 4.2（硬依赖，装不上就跑不起来）

backend 用 GTSAM 做因子图优化。本机 `sudo`/`su` 都不可用，所以是**解包到本地前缀**的方式：

```bash
# 有 root 的正常机器：
sudo apt install ros-humble-gtsam

# 无 root（本机现状）：
mkdir -p ~/ros_ws/.deps/deb && cd ~/ros_ws/.deps/deb
apt-get -o APT::Sandbox::User=root download ros-humble-gtsam
cd ~/ros_ws/.deps && dpkg -x deb/ros-humble-gtsam_*.deb gtsam
```

无 root 路径下**每次运行都要**带：

```bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

编译时还要 `export CMAKE_PREFIX_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble:$CMAKE_PREFIX_PATH`。

### 2.2 编译（两个坑）

```bash
cd ~/ros_ws
export PATH=/usr/bin:$PATH                    # 必须系统 python3.10，不要用托管的 3.13
source /opt/ros/humble/setup.bash
colcon build --packages-select localization_interfaces backend \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

- **坑 1**：PATH 上有 python3.13 时，ROS 的 `em`（empy）模块找不到，报 `ModuleNotFoundError: em`。
- **坑 2**：CMake 会把 `Python3_EXECUTABLE` 写进缓存，改 PATH 也没用。必须
  `rm -rf build/localization_interfaces build/backend install/...` 后再编，或显式传参。
- 全量 Release 首次约 **19 分钟**，之后增量很快。

`src/z-localization/src/` 下的 `frontend`、`bringup`、`livox_ros_driver2` 已加 `COLCON_IGNORE`：
前两个暂不用，后者与导航项目自带同名包（1.1.0 vs 1.0.0）冲突。

### 2.3 先验地图包（bundle）

backend 不吃单个 PCD，吃的是 **关键帧 + Scan Context 描述子**的目录：

```
pcd/simulation/rmuc_2026_bundle/
├── keyframes.txt        # 190 个关键帧位姿
├── map.pcd              # 全图（体素 0.15）
├── clouds/000000.pcd …  # 每帧局部点云
└── descriptor/…         # Scan Context
```

三个硬约束：

1. **必须与 nav2 栅格图在同一个 map 系**。这是最容易错的地方，详见
   `ZLOC_INTEGRATION_HANDOFF.md` 4.3 节。仿真里 `世界→map = (+10.40, −0.05)`，
   用错了就表现为"点云和栅格图整体错开一个固定距离，导航一直带着这段偏差"。
2. `--sc-lidar-height` 必须等于配置里 `common.scan_context.lidar_height`（本例都是 0.5），
   否则启动时报 `saved SC config does not match runtime config` 并拒绝加载。
3. 采样框要取**场地内墙范围**（本例 `−3.4..24.2 × −7.4..7.3`），外扩到墙外会生成无效关键帧。

### 2.4 外参（`body`）

```yaml
body:
  frame: base_footprint
  rotation: {roll_deg: -15.0, pitch_deg: 0.0, yaw_deg: 0.0}
  offset:   {x: -0.0496, y: -0.0936, z: -0.512}
```

仿真是 URDF 理论值（雷达相对 `base_footprint` 在 0.512 m 高、横滚 −15°）。
**上实车必须用 `body_calibration` 实测重标**，这个外参错了会直接表现为"怎么调都差一点点"。

### 2.5 启动提示（`startup_hint`）

后端把 hint 当作 `map→sensor`，所以要给**雷达在 map 系下的位姿**：

```yaml
startup_hint:
  enabled: true
  force_until_matched: true
  x: 0.30        # 出生点世界 (-10.10, 0) → map (0.30, -0.05)
  y: -0.05
  z: 0.512       # 雷达离地高度，不是 0
  yaw_deg: 0.0
```

- 给 `z: 0` 会让初始猜测整整低 0.5 m。
- 不给 hint 也能定位，但 RMUC 场地前后对称，描述子容易锁到**镜像点**（实测锁到 x≈20.2）。
- 实车没有固定出生点时关掉 `enabled`，靠描述子 + 几何验证自己找。

### 2.6 兜底节点（`zloc_bridge/map_odom_fallback`）

backend 在 `INITIALIZING / SUSPECT / LOST` 时**不发布** `map→odom`，Nav2 会报
`Timed out waiting for transform` 整栈停摆。这个节点在 backend 静默时用 last-known 或 identity 接管，
并带 0.6 s / 0.3 s 去抖，避免和后端抢着广播。

---

## 3. 怎么用

### 3.1 启动（仿真）

```bash
cd ~/ros_ws
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# 终端 1：Gazebo（必须先起，否则 point_lio 没输入）
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 终端 2：导航 + 定位
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True
```

三个 launch 参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `use_zloc` | `False` | `True` 时关掉 small_gicp 和静态 `map→odom`，改由 backend 发布 |
| `zloc_config` | `config/simulation/zloc_backend.yaml` | 后端配置文件 |
| `zloc_prior_map` | `pcd/simulation/rmuc_2026_bundle` | 先验地图包目录 |

### 3.2 回滚到 small_gicp

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=False \
  prior_pcd_file:=$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_world.pcd
```

> **注意**：`prior_pcd_file` 的默认值是 `pcd/simulation/<world>.pcd`，而原来的 `rmuc_2026.pcd`
> 在并入 Z-loc 时被改名成了 `rmuc_2026_world.pcd`，所以**不显式传这个参数回滚会加载失败**。
> 另外 small_gicp 的 `init_pose` 默认全零，要求先验图坐标系里"机器人起点≈原点"；
> 世界系点云不满足这个条件，回滚后大概率要在 RViz 里手动给一次 `2D Pose Estimate`
> （发到 `initialpose` 话题）才能锁上。

### 3.3 运行期看什么

**状态话题**（每秒刷新，字符串，可直接 grep）：

```bash
ros2 topic echo /red_standard_robot1/backend_node/localization_state --once
# status=NORMAL;pose_trusted=true;tf_frozen=false;tf_published=true;epoch=0;...
```

| 状态 | 含义 | 应对 |
| --- | --- | --- |
| `INITIALIZING` | 还没锁上（等数据 / 验证没过） | 看日志 `tryRelocalization` 的 `corr_ratio` |
| `NORMAL` | 正常，可信，发 TF | — |
| `SUSPECT` | 观测不可信，TF 冻结 | 单帧触发会自动恢复；持续出现说明场景退化 |
| `OBS_DEGRADED` | 连续 7 次校正冲突 | 位姿可能在漂，检查是否被搬动 |
| `LOST` | 彻底丢失 | 调用 relocalize 服务 |

**日志里两个关键数字**（每帧一行 `factor accepted / conflicted`）：

| 字段 | 正常值 | 异常含义 |
| --- | --- | --- |
| `ratio`（支持率） | **0.9 以上** | 掉到 0.1 量级 → 先验图与点云对不上（体素/坐标系问题） |
| `rmse` | **0.03~0.06 m** | 涨到 0.15 以上 → 配准滑到局部解 |
| `NIS` | **1e-3 以下** | 到 10 以上 → 测量与预测矛盾，校正会被丢弃 |
| `delta_t` | **毫米级** | 恒定在 0.1 以上 → 校正一直没生效（说明被 NIS 拒了） |
| `iterations` | 1~5 | 打满上限 → 偶发不收敛 |

**手动干预的服务**：

```bash
# 触发一次全局重定位（搬动过机器人、或 LOST 时用）
ros2 service call /red_standard_robot1/backend_node/relocalize std_srvs/srv/Trigger

# 保存当前运行轨迹为地图包
ros2 service call /red_standard_robot1/backend_node/save_map std_srvs/srv/Trigger

# 统一重置
ros2 service call /localization_reset std_srvs/srv/Trigger
```

**其他输出**：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `~/localization_pose` | `PoseWithCovarianceStamped` | 带协方差的位姿，决策层可按不确定度降速 |
| `/localization/odometry` | `Odometry` | 同上，odom 形式 |
| `/localization/lio_world_cloud` | `PointCloud2` | map 系实时点云，RViz 里核对是否贴合先验图 |

### 3.4 调参表

只列**会真的去动**的参数（都在 `config/simulation/zloc_backend.yaml`）：

| 参数 | 当前值 | 作用 | 什么时候动 |
| --- | --- | --- | --- |
| `registration.voxel_resolution` | 0.20 | 目标点云体素 | **别改小**。0.4 会给点云"造厚度"导致平面验收失败（支持率天花板 0.10）；0.2 时 0.93 |
| `registration.min_correspondence_ratio` | 0.3 | 支持率门槛 | 实测 0.92，有余量。实车换真实建图 bundle 后可恢复 0.5 |
| `registration.min_coverage_sectors` | 3 | 方位扇区门槛 | 同上，实车恢复 6 |
| `registration.measurement_translation_std_floor` | 0.03 | 测量噪声下限 | 静止抖动 / 一律判冲突时调大 |
| `registration.measurement_rotation_std_floor_deg` | 0.5 | 同上（旋转） | 同上 |
| `registration.max_iterations` | 32 | 配准迭代上限 | 频繁出现"打满未收敛"时加大；正常帧只用 1~5 次 |
| `tracking.target_radius` | 12.0 | 跟踪子图半径 | 场景结构稀疏（长走廊/空旷）时调大，让远处墙进入目标 |
| `tracking.target_keyframes` | 16 | 跟踪子图关键帧数 | 同上 |
| `tracking.source.max_time` | 0.3 | 源点云累计窗口 | 里程计静止抖动大时调小 |
| `common.odom_cloud_sync.max_interval` | 0.1 | 里程计/点云同步容差 | 实车建议 0.05 |
| `common.scan_context.lidar_height` | 0.5 | 描述子高度 | **必须与生成 bundle 时一致** |

### 3.5 重新生成地图资产

改了场地、换了外参、或 map 系平移动了，都要走这三步（顺序不能变）：

```bash
# 1. 场地 STL → 世界系点云
python3 src/pb2025_sentry_nav/pb2025_nav_bringup/scripts/stl_to_pcd_converter.py \
  src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2026/meshes/rmuc_2026.stl \
  src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_world.pcd \
  --spacing 0.05 --blind-zone 0

# 2. 平移到 map 系（偏移量必须按栅格图反推，不能用出生点当原点）
python3 src/zloc_bridge/scripts/shift_pcd.py \
  --src .../rmuc_2026_world.pcd --dst .../rmuc_2026_mapframe.pcd \
  --dx 10.40 --dy -0.05 --dz -0.23

# 3. 生成关键帧地图包
ros2 run backend build_pcd_map_bundle \
  --input .../rmuc_2026_mapframe.pcd --out-bundle .../rmuc_2026_bundle \
  --x-min -3.4 --x-max 24.2 --y-min -7.4 --y-max 7.3 \
  --sensor-z 0.5 --bundle-step 1.5 --map-voxel 0.15 --scan-voxel 0.4 \
  --descriptor-type scan_context --sc-lidar-height 0.5

# 4. 让 install 空间看到新目录（按文件建链，必须重建）
colcon build --packages-select pb2025_nav_bringup --symlink-install
```

配好后可以随时自检对齐情况（不用起仿真）：

```bash
cd src/zloc_bridge/scripts
python3 align_pcd_to_pgm.py --yaml .../map/simulation/rmuc_2026.yaml \
  --pcd .../rmuc_2026_mapframe.pcd --z-min 1.9 --z-max 2.3
# 期望：dx/dy 都在 ±0.05 以内

python3 render_map_overlay.py --yaml .../rmuc_2026.yaml \
  --pcd .../rmuc_2026_mapframe.pcd --z-min 1.9 --z-max 2.3 --out /tmp/overlay.png
```

---

## 4. 与 small_gicp 的对比

### 4.1 结构差异（这条决定了能力边界）

| | small_gicp_relocalization | Z-Localization backend |
| --- | --- | --- |
| 配准对象 | **整张**先验 PCD（全图） | 按位姿取 **12 m / 16 关键帧**局部子图 |
| 初始值 | 上一帧结果 / `init_pose` / `initialpose` | 上一帧结果，或 startup hint，或描述子召回 |
| 全局搜索 | **无**。只有局部精化 | Scan Context 召回 + 多 yaw 种子验证 + 一致性检查 |
| 收敛判据 | `result.converged`，不收敛就沿用旧值 | 支持率 + 覆盖扇区 + 残差 + 平面有效性 + 可观测性（5 项） |
| 融合 | 无。直接替换结果 | GTSAM 因子图，NIS 卡方检验，冲突容忍 7 次 |
| 丢失恢复 | **不能** | 状态机 + relocalize 服务 + 自动重定位 |
| 发布 TF | **无条件**，20 Hz 固定频率 | 仅在可信时发布；不可信时冻结，由桥接节点兜底 |
| 输出信息 | 只有 TF | TF + 位姿 + 6×6 协方差 + 状态字符串 |
| 点云累计 | 累积全部帧，每 0.5 s 清一次 | 0.3 s 滑动窗口，带时间跨度约束 |
| 代码规模 | ~240 行 | 数万行，GPL-2.0（跨进程调用，不污染导航代码） |

一句话：small_gicp 是**一个配准器**，backend 是**一套定位系统**。

### 4.2 同输入实测（2026-11-03，同一台机器）

条件：同一份 18 907 点的合成扫描、同一份 map 系先验图、机器人静止在 `(0.30, −0.05)`、
真实值由栅格图反推。两者都用各自"正常配置"。

| 指标 | small_gicp（默认参数） | small_gicp（仓库调优参数*） | Z-Loc backend |
| --- | --- | --- | --- |
| 输出 `map→odom` (x, y) | (−0.196, −0.051) | (−0.203, −0.050) | (0.256, −0.009) |
| 水平误差 | **0.50 m** | **0.50 m** | **0.06 m** |
| `z` 分量 | 0.914 m（明显偏高） | 0.826 m | −0.069 m |
| 配准质量指标 | 无（只有 converged 标志） | 无 | `ratio=0.92, rmse=0.056` |
| 单次配准耗时 | 未暴露，约 80 ms/次（估） | 同上 | **中位 7.2 ms**，p90 13.8 ms，max 17 ms |
| CPU 占比 | 8.8% | 17.5% | 24.2% |
| 常驻内存 | 301 MB | 316 MB | 285 MB |
| 线程数 | 15 | 15 | 28 |
| 地图加载 | 2.46 M 点 + 协方差 + KD 树，约 1.4 s | 同 | 190 关键帧，约 2.6 s |
| 启动后收敛 | 首帧 1 次 `did not converge`，之后收敛 | 0 次 | 首帧一次命中（hint + 扫描匹配） |

\* 调优参数指 yaml 里被注释掉的那组：`global_leaf 0.1 / registered_leaf 0.05 / max_dist_sq 3.0`。

> **两个必须说明的口径问题**：
> 1. 这份扫描是从先验图自己生成的，两者都"自洽"，属于对双方都**偏有利**的考题。
> 2. backend 的水平残差 6 cm 里含测试夹具的偏差：夹具发的点云是水平姿态，而配置里
>    `body.rotation.roll=-15°` 是真实雷达的安装横滚，后端会把它补回来，于是引入了一点偏置。
>    换句话说**真机场景下 backend 的这 6 cm 未必会更大，但也不能直接当精度指标用**。

### 4.3 分维度比较

**精度**

small_gicp 在这份数据上偏 0.50 m，且**调 leaf size 无效**（0.25 → 0.1 后仍是 0.50），
说明它不是"精度不够"而是**滑到了另一个局部解**：全图 GICP 在 10 次迭代、`max_dist=1.0 m`
的约束下，从零初值出发收敛到的点并不唯一。这类偏差在换场地、换起始位姿后会变化，
不是可以标定掉的系统误差。

backend 每一帧都会输出支持率和残差，可以直接判断"这次结果可不可信"——这是它精度表现更稳的
根本原因：**不是算得更准，而是不准的时候它不发布**。

**复杂场景能力**（这是替换的主要动机）

| 场景 | small_gicp | backend |
| --- | --- | --- |
| 长走廊 / 空旷区（几何退化） | 直接滑走，且无任何提示 | 计算可观测性，退化时降权/冻结；`weak` 位标明弱方向 |
| 动态障碍 / 人员走动 | 全图配准，被拖走概率高 | 局部子图 + 平面验收 + NIS 检验，异常帧被拒 |
| 被搬动 / 开机位置不对 | **无法恢复**，必须手动 `initialpose` | 描述子召回 + 几何验证自动重定位 |
| 先验图不完整 / 有残影 | 拉低整体配准 | 支持率下降但按扇区判定，局部缺失不致命 |
| 缓慢漂移 | 无回环，靠每帧 GICP 硬拉 | 因子图 + 先验观测持续校正 |

**性能**

- **单帧成本**：backend 更低（7 ms vs ~80 ms）。因为它只在 12 m 子图上做，不做全图最近邻。
- **CPU 总量**：backend 反而更高（24% vs 8.8~17.5%），原因是它除了配准还跑因子图优化、
  状态机、描述子、点云缓存，且处理频率更高（~1.6 Hz vs 2 Hz 但每帧更重）。
- **内存**：相当（285 vs 301~316 MB），backend 的 64 MB 是点云缓存，可配。
- **地图加载**：backend 稍慢（2.6 s vs 1.4 s），但只加载一次。

结论：**换掉之后单帧更省，整体更费**。在哨兵车这种算力有余、但定位可靠性要求高的场景，
这笔账是划算的；如果要挤算力，优先关 `output.world_cloud` 和 `visualization`。

**可运维性**

- small_gicp 出问题时你只看到"GICP did not converge"或者位姿悄悄飘走，没有中间量可查。
- backend 有状态机、支持率、残差、NIS、协方差、且可以用服务强制重定位。
  代价是**配置项多、依赖重（GTSAM）、编译慢**。

### 4.4 什么时候该用哪个

| 情况 | 建议 |
| --- | --- |
| 比赛 / 长期运行，要求丢失可恢复 | backend |
| 只做算法快速验证，不想装 GTSAM | small_gicp + `initialpose` |
| 算力极度紧张（如挂在 Orin Nano 且还要跑视觉） | 需要实测；backend 的 24% 单核需评估 |
| 场地每场都变、没有稳定先验图 | 两者都不合适，先解决建图 |
| 调试阶段想快速看"点云对不对" | 先把 backend 起来但只观察 `lio_world_cloud`，不发 TF |

---

## 5. 已知限制 / 上实车前 TODO

1. **外参未标定**：`body.rotation.roll=-15°`、`offset=(-0.0496,-0.0936,-0.512)` 都是 URDF 理论值。
   上实车先跑 `body_calibration`，否则会一直有"差一点点"的残差。
2. **仿真调参不能直接搬**：`min_correspondence_ratio=0.3`、`voxel_resolution=0.20` 是针对
   STL 合成先验图调的。实车用真实建图的 bundle，平面质量更好，应恢复默认 0.5 / 6 和更大体素。
3. **点云 z 偏移 `-0.23`** 是按场地网格地面高度估的，实车要重新量。
4. **单帧偶发不收敛会立即降 SUSPECT**（`likelihoodValid()` 要求 `converged`，
   `onObservationModelInvalid()` 没有累计容忍）。目前靠 0.6 s 接管去抖掩盖，影响可接受；
   如实车频繁出现，应改判据而不是继续加 `max_iterations`。
5. **只在出生点静止验证过**。移动中、遮挡变化、被撞后恢复这几种情况都还没测。
6. **许可**：backend 是 GPL-2.0-only。当前是跨进程 DDS 调用（聚合），没把源码链进导航包；
   如果以后想把它做成导航包内的库，需要重新评估。
7. **`scan_context` 参数与 bundle 强耦合**，改了就必须重新生成地图包，否则拒绝加载。

---

## 6. 最常用的几条命令

```bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

./stop_all.sh nav                                   # 清残留（最常踩的坑）
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True       # 启动

ros2 topic echo /red_standard_robot1/backend_node/localization_state --once   # 状态
ros2 run tf2_ros tf2_echo map odom                                            # TF
ros2 service call /red_standard_robot1/backend_node/relocalize std_srvs/srv/Trigger  # 手动重定位
```
