# 方案二落地交接：用 Z-Localization 后端替换 small_gicp（仿真）

> 目标：在仿真中用 Z-Localization 后端接管 `map→odom`，替换 `small_gicp_relocalization`。
> 保留 `point_lio` 前端、`loam_interface`、`sensor_scan_generation`、Nav2 全链路与 2D 地图资产。
> **不启用 UWB**（无硬件）。

---

## 1. 改动清单

### 新增

| 文件 | 作用 |
| --- | --- |
| `src/zloc_bridge/` | 新建桥接包（Apache-2.0，不含 GPL 代码） |
| `src/zloc_bridge/scripts/map_odom_fallback.py` | 后端不发 `map→odom` 时兜底，避免 Nav2 停摆；`map→odom` 倾斜 >5° 时告警 |
| `src/zloc_bridge/scripts/fake_lio_publisher.py` | 离线验证用假 LIO 数据源（不参与实际运行） |
| `src/zloc_bridge/scripts/zloc_pose_check.py` | 一次性诊断：在提示位姿附近扫 (roll, yaw)，判定 `support_ok=false` 是几何问题还是门槛问题 |
| `src/zloc_bridge/scripts/inspect_map_bundle.py` | 离线检查地图包：关键帧子图覆盖度、理想扫描的支持率 |
| `src/zloc_bridge/scripts/grab_scan.py` | 抓一帧真实雷达 + 真值位姿存 npz，供离线复查 |
| `config/simulation/zloc_backend.yaml` | 后端配置：legacy 输入、无 UWB、仿真外参 |
| `pcd/simulation/rmuc_2026_world.pcd` | 由场地 STL 采样得到（世界系） |
| `pcd/simulation/rmuc_2026_mapframe.pcd` | 平移到 map 系（世界→map = `+10.40, -0.05`，见 4.3） |
| `pcd/simulation/rmuc_2026_bundle/` | 先验地图包：190 关键帧 + Scan Context 描述子 |

### 修改

| 文件 | 改动 |
| --- | --- |
| `launch/localization_launch.py` | 新增 `use_zloc` / `zloc_config` / `zloc_prior_map`；启用时关闭 small_gicp 与静态 `map→odom`，改为 include `backend.launch.py` + 兜底节点 |
| `launch/bringup_launch.py` | 透传上述三个参数 |
| `launch/rm_navigation_simulation_launch.py` | 透传上述三个参数 |
| `src/z-localization/src/{frontend,bringup,livox_ros_driver2}/COLCON_IGNORE` | 规避 `livox_ros_driver2` 同名包冲突；本次只编译 `backend` + `localization_interfaces` |
| `z-localization/` 整体移入 `src/z-localization/` | 让 colcon 能发现（git 仓库完整保留） |

### 依赖

`backend` 需要 GTSAM 4.2。本机 `sudo` 不可用（`/etc/sudo.conf` 属主被改成 uid 65534），
因此用 `apt-get -o APT::Sandbox::User=root download` 下载 deb 后解包到 `.deps/gtsam`。
运行时需要：

```bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

> 若之后能修好 sudo，执行 `sudo apt install ros-humble-gtsam` 即可摆脱这个前缀。

---

## 2. 运行方式

> 顺序很重要：**先起 Gazebo，再起导航**。先起导航的话 `point_lio` 没有输入，
> 后端会一直停在 `INITIALIZING`，看似"没反应"。

```bash
cd /home/l/ros_ws
source install/setup.bash
export LD_LIBRARY_PATH=/home/l/ros_ws/.deps/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# 终端 1：仿真（必须先起，等场地加载完）
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 终端 2：导航 + Z-Localization 后端
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False use_zloc:=True
```

回滚：把 `use_zloc:=True` 去掉（或显式 `use_zloc:=False`），即恢复 `small_gicp` / 静态 `map→odom` 的原行为。

### 坑：新增 pcd 资产后必须重新 build

`INSTALL_TO_SHARE` 在 `--symlink-install` 下是**逐文件**建符号链接，目录本身是实体。
地图包是在 `colcon build` 之后才生成的，所以 install 空间里看不到，启动时会报：

```
[ERROR] [launch]: Caught exception in launch: Prior map does not exist;
set prior_map:=/absolute/map or mode:=mapping
```

**凡是往 `pcd/`、`map/`、`config/` 里新增文件，都要重新执行一次：**

```bash
colcon build --packages-select pb2025_nav_bringup --symlink-install
```

判断是否生效：

```bash
test -d install/pb2025_nav_bringup/share/pb2025_nav_bringup/pcd/simulation/rmuc_2026_bundle \
  && echo OK || echo MISSING
```

### 数据流

```
Mid360 → point_lio → loam_interface ─┬─ lidar_odometry (odom 系, child=front_mid360)
                                     └─ registered_scan (odom 系点云)
                                            │ legacy 同步（容差 0.1 s）
                                     backend ─┴→ map→odom（仅可信时）
                                            │
                              map_odom_fallback（后端冻结时兜底）
                                            ↓
                                          Nav2
```

关键配置点（`zloc_backend.yaml`）：

- `common.frontend: legacy`，`input_cloud_in_local_frame: false`（输入是 odom 系点云）
- `common.sensor_frame: front_mid360` 必须等于输入里程计的 `child_frame_id`
- `output.legacy.enabled: false`：否则 `base_footprint` 会有 `map` 和 `odom` 两个父节点
- `uwb.enabled: false`

---

## 3. 已验证（离线注入，非 Gazebo）

用 `fake_lio_publisher.py` 按真实接口注入 `lidar_odometry` + `registered_scan`，
扫描由先验图按角度分箱取最近点近似（自带遮挡，垂直视场限制在 Mid-360 的 -7°~+52°）。

| 项 | 结果 |
| --- | --- |
| 后端加载地图包 | ✅ `keyframes=273, descriptor=Scan Context` |
| 参数生效 | ✅ `frontend=legacy, uwb=disabled, odom=/red_standard_robot1/lidar_odometry` |
| 扫描与地图对齐 | ✅ `rmse=0.061 m`，`rank=6/6`，`observable=true` |
| 定位状态 | ✅ `status=NORMAL;pose_trusted=true;tf_published=true` |
| 车体位姿 | ✅ `x=-0.040`（真值 0），误差 < 5 cm |
| `map→odom` | ✅ `translation=[0.013, 0.019, -0.080]`，旋转接近单位 |
| 收敛稳定性 | ✅ `joint_NIS≈1e-3`，`delta_t≈1e-4 m`（不再跳变） |
| 兜底节点 | ✅ 后端静默时发 identity；后端接管后停止 |
| launch 参数 | ✅ `--show-args` 可见 `use_zloc/zloc_config/zloc_prior_map` |

---

## 4. 未完成 / 需要你确认的

1. **Gazebo 没跑起来**：本环境的 ignition GUI 因 D-Bus 授权失败而整栈退出，只留下
   `/rosout` 和 `/parameter_events`。上面的验证是用离线注入做的等效验证，
   真机/有显示的机器上请按第 2 节跑一遍完整仿真。
2. **对应率门槛被放宽**（`registration.min_correspondence_ratio: 0.15`、
   `min_coverage_sectors: 3`）。原因是先验图由 STL 面采样合成，点密度与遮挡分布和真实扫描不同，
   默认 0.5/6 会一直卡在 `support_ok=false`。**换成真实建图或上实车后必须恢复 0.5 / 6。**
3. **场地镜像歧义**：rmuc 场地前后对称，不给提示时描述子会锁到镜像位置（实测锁到 x≈20.2，
   正好是场地中心对称点）。已用 `startup_hint` 固定为出生位姿（map 原点）。
   实车没有固定出生点时，靠真实场景不对称特征 + UWB/多点重定位解决。
4. **外参是 URDF 理论值**：`roll=-15°, offset=(-0.0496,-0.0936,-0.512)`，
   来自 `pb2025_robot_description`，未实测标定。上实车前用 `body_calibration` 重标。
5. **地图系偏移是算出来的，不是猜的**：map 系点云按 `(+10.40, -0.05, -0.23)` 平移。
   前两项由栅格图反推（见 4.3），`-0.23` 仍是按场地网格地面高度估的，实车需要重新测。

---

## 4. 常见故障

### 4.1 RViz 里看不到机器人 / 代价地图空白 / 后端一直 INITIALIZING

几乎都是**孤儿进程**导致的，分两种：

**A. 残留的 `component_container_isolated` 占住名字**

`use_composition:=True` 时，`loam_interface`、`sensor_scan_generation`、`map_server`、
nav2 各 server 都是被 `LoadComposableNodes` 装进 `/red_standard_robot1/nav2_container` 的。
上一次中断的 launch 如果留下容器进程，新 launch 的同名容器就会加载失败，
结果是**容器起来了但里面是空的**，日志里不一定报错。表现：

```bash
ros2 node list | grep -E "loam|sensor_scan|map_server"   # 什么都没有
```

**B. 残留的 `ign_sim_pointcloud_tool` 造成多发布者**

每个 nav launch 都会起一个点云转发器。残留两个以上时，`livox/lidar` 有多个发布者
且时间基准不同，`point_lio` 会刷：

```
[ERROR] [li_initialization]: lidar loop back, clear buffer
```

里程计随之失效，`aft_mapped_to_init` 频率会异常（曾测到 3807 Hz）。

**处理：**

```bash
cd /home/l/ros_ws
./stop_all.sh            # 停导航 + Gazebo
./stop_all.sh nav        # 只停导航侧，保留 Gazebo
```

确认干净后再按第 2 节顺序重新启动。

### 4.2 后端长时间停在 INITIALIZING

先确认上游有数据：

```bash
ros2 topic info /red_standard_robot1/livox/lidar       # Publisher count 应为 1
ros2 topic echo /red_standard_robot1/lidar_odometry --once
ros2 topic echo /red_standard_robot1/registered_scan --once
```

都有数据后看后端日志里的 `support_ok` / `corr_ratio`：

- `support_ok=false` 且 `corr_ratio` 远低于 `min_correspondence_ratio`：几何验证过不去。
- 完全没有任何 `tryRelocalization` 日志：后端没收到同步帧，检查 `odom_cloud_sync.max_interval`
  以及 `lidar_odometry` 与 `registered_scan` 的时间戳是否同源。

#### 4.2.1 `corr_ratio` 只有 0.10：`voxel_resolution=0.4` 把点云「做厚」了

最终结论（2026-11-03，已复刻后端逻辑逐项验证）：**不是几何错位，不是外参，是基线配置的
`registration.voxel_resolution: 0.4`**。

0.4 m 体素会把表面点量化成 ±0.2 m 的薄板；局部 PCA 的邻域尺度只有约 0.55 m，薄板厚度和
邻域尺度同量级，`min_plane_eigenvalue_ratio=0.2` 这条验收必然过不去 → **目标点里只有 20.8%
有有效平面**，剩下的全被 `plane.valid` 判掉，于是 `corr_ratio` 天花板就卡在 0.10 附近。

复刻后端平面验收后，理想扫描（从先验图自己生成）的支持率随体素变化：

| 体素 | 目标点数 | 有效平面占比 | 理想扫描支持率 |
| --- | --- | --- | --- |
| 0.40 | 24 246 | 0.208 | **0.093** |
| 0.30 | 34 612 | 0.687 | 0.710 |
| **0.20** | 64 960 | 0.837 | **0.928** |
| 0.15 | 90 818 | 0.856 | 0.935 |

改成 0.20 后实测（离线注入）：`corr_ratio=0.938`、`rmse=0.027`、`support=10441/11129`、
`sectors=36`，跟踪因子耗时反而从 25 ms 降到 8 ms。

推论：`min_correspondence_ratio` 不需要再靠放宽来迁就，仿真里给 0.3、实车用真实建图的
bundle 回到基线 0.5 / 6。

另外 `cond=12.4`、`schur_r=12.2`、`weak=000010` 说明有一个旋转方向偏弱（场地空旷时正常），
如果实车/别的场地出现退化告警，再调 `registration/degeneracy_condition_threshold`。

判断「是几何问题还是门槛问题」用这一条命令，别靠猜：

```bash
# 先起订阅者，再起数据源（本环境下 ROS 发现要求订阅方先存在）
ros2 run zloc_bridge zloc_pose_check.py --ros-args \
  -p bundle:=$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_bundle \
  -p hint_z:=0.512
```

它会在启动提示位置附近扫描 (roll, yaw)，打印支持率排序，最后给出 `VERDICT`：

- `提示位姿本身吻合良好` → 几何没问题，往门槛/体素方向查。
- `需要非零 roll 才能吻合` → 雷达安装横滚没被里程计补偿。
- `需要非零 yaw 才能吻合` → 提示航向或里程计初始朝向偏了。

`map_odom_fallback` 现在还会在 `map→odom` 出现 >5° 的 roll/pitch 时告警，因为那种姿态下
Nav2 的 2D 代价地图会跟着歪。

#### 4.2.2 NORMAL 与 SUSPECT/OBS_DEGRADED 之间反复抖动

症状：进入 NORMAL 后每 4~5 s 一轮

```
factor conflicted: rmse=0.181, support=398/3795, delta_t=0.148, delta_r_deg=2.03, NIS=51~73/6
state -> OBS_DEGRADED（correction conflict 1/7 … 7/7）→ SUSPECT
tryRelocalization: matched KF 171 … delta_t=0.0005  → NORMAL
```

诊断要点：**跟踪因子和重定位因子对同一个位姿的判断相差 10~15 cm / 1.6~2°**，而重定位给的是
0.0005 m。而且 `delta_t` 一直不变，说明校正被 NIS 否掉、位姿原地不动，同一份偏差被反复测出来。

根因仍是 4.2.1 的体素问题：有效平面只剩 20%，跟踪配准的约束太弱、残差面太浅，于是稳定地滑到
一个偏 1.6° 的局部解；同时因为平面太少，残差反而算得很"自信"，NIS 直接冲到 45~73 把校正全丢掉。

处理（本仓库已改）：

- `registration.voxel_resolution: 0.4 → 0.20`（治本，支持率 0.10 → 0.94，见 4.2.1）
- `tracking.target_radius 8 → 12`、`target_keyframes 11 → 16`（让跟踪与重定位的目标密度一致）
- `tracking.source.max_time 0.5 → 0.3`
- `registration.measurement_translation_std_floor: 0.03`、`measurement_rotation_std_floor_deg: 0.5`
  （给测量一个合理的地板噪声，避免"静止时协方差收紧 → 一律拒收 → 抖动"）
- `registration.max_iterations: 16 → 32`（减少偶发打满迭代未收敛，见 4.2.3）

改完的离线复验：`corr_ratio=0.938 / rmse=0.027 / NIS≈1e-9 / 单帧 8 ms`，跟踪因子不再冲突。

改完看跟踪因子那行日志的 `delta_t`：回落到厘米级就说明盯对了；如果仍是 0.15，
下一步查 `tracking/source` 的累计窗口和 `lidar_odometry` 在静止时的漂移量。

#### 4.2.3 仿真实机复验（2026-11-03 12:34，真实 Gazebo 数据）

上表都是离线注入的结论，下面是**真实仿真数据**跑出来的结果（`backend_node_*.log`）：

| 运行 | 状态切换 | 跟踪因子 接受/冲突/模型无效 | `corr_ratio` |
| --- | --- | --- | --- |
| 12:26 | 1 次 → NORMAL | 60 / 0 / 0 | 0.938 |
| 12:33 | 1 次 → NORMAL | 37 / 0 / 0 | 0.917 ~ 0.954 |
| 12:34 | 1 次 → NORMAL，之后 1 次 SUSPECT | 30 / 0 / 2 | 0.901 ~ 0.954 |

典型稳态日志：

```
factor accepted: rmse=0.0565 support=5379/5859 ratio=0.918 sectors=36
                 min_eig=0.225 rank=6/6 observable=true weak=000000
                 iterations=4 NIS=0.00107/6 elapsed_ms=13.2
global correction committed: delta_t=0.0004m delta_r_deg=0.001
```

对比修复前：`ratio=0.10`、`NIS=51~73`、`delta_t=0.15`、每 4 s 抖一轮。现在 `delta_t` 是亚毫米，
`NIS` 在 1e-3 量级，迭代次数 2~5，单帧 8~18 ms。

初始定位也变好：`tryRelocalization` **第一次就用 startup_hint 命中**（`corr_ratio=0.954`、
`yaw_seed=0deg`），不再需要靠描述子搜 15 次、也不会锁到场地镜像对称点上。

**残留问题：单帧偶发不收敛会立刻降 SUSPECT。** 12:34 那次有两帧 `converged=false`
（其中一帧 `elapsed_ms=182`，是正常帧的 10 倍），而 `likelihoodValid()` 要求 `converged`
为真，`LocalizationStateMachine::onObservationModelInvalid()` 又是**一次即降级**（不像
correction conflict 要累计 7 次）：

```
state -> SUSPECT (observation model invalid: ... ratio=0.900816 ... elapsed_ms=182.6)
```

影响可控：`map_odom_fallback` 的接管去抖是 0.6 s，单帧 SUSPECT 撑不到触发，所以不会出现
两个节点同时广播；下一帧正常即可自愈。为降低概率把 `max_iterations` 提到 32
（正常帧 2~5 次迭代，只在难帧上多花约 30 ms）。如果实车仍频繁出现，再考虑把
`converged` 的判据或状态机的容忍次数一起调，而不是继续加迭代上限。

---

## 4.3 点云与栅格图错开一个固定距离

症状：RViz 里点云相对栅格图朝某个方向整体错开（仿真里约 0.3 m），而且这一偏移会一直
带到导航——因为定位锁的是先验点云，点云又和栅格图不在同一个 map 系里，`map→odom`
就把这份偏差固定下来了。

**根因**：先验点云的平移量当初是「出生点当作 map 原点」估的 `(+10.10, 0)`，但 nav2 的
栅格图用的是另一套原点。用栅格图本身可以把它算出来，不需要猜：

| 依据 | 数值 |
| --- | --- |
| `rmuc_2026.yaml` | `origin [-3.60, -7.60]`, 560×302 @0.05 → x[-3.60, 24.40] y[-7.60, 7.50] |
| 图像尺寸 | 28.00 × 15.10 m，正好是场地**内墙**范围（不是含墙厚的外沿 29.15 × 16.06） |
| 占用格质心 | x=**10.406** y=**−0.052** |
| 极值中点 | x=**10.400** y=**−0.050** |

场地在世界系里关于原点对称，所以「世界原点 → map」就是 **(10.400, −0.050)**。原来用
10.10，于是整整差了 0.30 m。

验证与修正都用脚本做，别再手算：

```bash
cd src/zloc_bridge/scripts

# 互相关求「点云要平移多少才能贴住栅格图」
python3 align_pcd_to_pgm.py \
  --yaml ../pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2026.yaml \
  --pcd  ../pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_mapframe.pcd \
  --z-min 1.9 --z-max 2.3        # 只取高墙，避开斜坡/高地顶面

# 直接看叠加图（白=空闲 黑=占用 红=点云墙点）
python3 render_map_overlay.py \
  --yaml .../rmuc_2026.yaml --pcd .../rmuc_2026_mapframe.pcd \
  --z-min 1.9 --z-max 2.3 --out /tmp/overlay.png

# 平移点云
python3 shift_pcd.py --src .../rmuc_2026_mapframe.pcd \
  --dst .../rmuc_2026_mapframe.pcd --dx 0.30 --dy -0.05
```

高度带的取法很重要：**`--z-min 0.35` 会把斜坡和高地顶面算进来**，那些在栅格图里是
合法空闲区，叠加图会糊成一片红，看着像"到处都错"，其实只是取错了高度。

改完必须三件事一起做，少一件都还会错开：

1. 重建 `rmuc_2026_bundle`（位置见第 5 节，采样框改成场地内墙范围）；
2. 改 `startup_hint`：出生点在世界 (-10.10, 0)，对应 map **(0.30, -0.05)**；
3. `colcon build --packages-select pb2025_nav_bringup --symlink-install`
   ——install 空间是按文件建链的，新目录不重建就看不到。

修复后的离线验证（`fake_lio_publisher`）落点是 `(0.256, -0.009)`，目标 `(0.30, -0.05)`，
残差约 6 cm；改之前是 `(-0.04, 0.04)`，正好差 0.30 m。

**注意 `fake_lio_publisher` 的一个坑**：它必须把点云减去 `(sensor_x, sensor_y, sensor_z)`
后再发布。如果直接把 map 坐标系的点云标成 `odom` 帧发出去，配准结果恒为 0，
这个离线验证就**测不出**绝对偏移。第一版就踩了这个坑。

---

## 5. 重新生成地图资产


```bash
# 1. 场地 STL → 点云（世界系）
python3 src/pb2025_sentry_nav/pb2025_nav_bringup/scripts/stl_to_pcd_converter.py \
  src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2026/meshes/rmuc_2026.stl \
  src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_world.pcd \
  --spacing 0.05 --blind-zone 0

# 2. 平移到 map 系。偏移量必须按栅格图反推，不能用出生点当原点：
#    世界→map = (+10.40, -0.05)，推导见 4.3。
python3 - <<'EOF'
import numpy as np
p='src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_world.pcd'
raw=open(p,'rb').read(); m=b'DATA binary\n'; i=raw.find(m)+len(m)
n=int([l.split()[1] for l in raw[:i].decode().splitlines() if l.startswith('POINTS')][0])
a=np.frombuffer(raw[i:i+n*12],dtype=np.float32).reshape(-1,3).copy()
a[:,0]+=10.40; a[:,1]+=-0.05; a[:,2]-=0.23
open(p.replace('_world','_mapframe'),'wb').write(bytes(bytearray(raw[:i])+a.astype(np.float32).tobytes()))
EOF

# 3. 生成关键帧地图包与描述子（采样框取场地内墙范围，别外扩到墙外）
ros2 run backend build_pcd_map_bundle \
  --input   $PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_mapframe.pcd \
  --out-bundle $PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_bundle \
  --x-min -3.4 --x-max 24.2 --y-min -7.4 --y-max 7.3 \
  --sensor-z 0.5 --bundle-step 1.5 --map-voxel 0.15 --scan-voxel 0.4 \
  --descriptor-type scan_context --sc-lidar-height 0.5
```

`--sc-lidar-height` 必须与 `zloc_backend.yaml` 里 `common.scan_context.lidar_height` 一致，
否则后端会以 `saved SC config does not match runtime config` 拒绝加载。

---

## 6. 离线验证复现

```bash
# 后端
ros2 launch backend backend.launch.py \
  config_file:=$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/zloc_backend.yaml \
  use_sim_time:=false mode:=localization \
  prior_map:=$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_bundle

# 假 LIO
ros2 run zloc_bridge fake_lio_publisher.py --ros-args \
  -p pcd:=$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2026_mapframe.pcd \
  -p radius:=12.0

# 观察
ros2 topic echo /backend_node/localization_state --once
ros2 run tf2_ros tf2_echo map odom
```
