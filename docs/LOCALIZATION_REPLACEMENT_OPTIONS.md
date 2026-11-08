# 用 Z-Localization 替换现有定位：候选方案

> 目标：提高哨兵车的定位精度。
> 前提事实（已核对源码，非推测）：
> - `small_gicp_relocalization` 无 `enable_relocalization` 参数（`start.sh` 里那段说明已过期）。
> - Z-loc 后端支持 `common.frontend=legacy`，用近似时间同步（默认 `max_interval=0.05 s`）配对
>   `sync_odom_topic` 与 `cloud_topic`（`src/backend/apps/node.cpp:107-140`）。
> - `input_cloud_in_local_frame: false` 时，后端接受 **odom 系点云** 并自行转回传感器系。
> - `point_lio` 的 `/aft_mapped_to_init` 与 `/cloud_registered` 时间戳同为 `lidar_end_time`，可被直接配对。

---

## 0. 先搞清楚：精度到底丢在哪

换算法之前必须先排掉下面两类"非算法"误差，否则换什么都不会提升：

| 误差源 | 现状 | 消除手段 | 量级 |
| --- | --- | --- | --- |
| 雷达-车体外参 | `nav2_params.yaml` 里 `extrinsic_T: [-0.011,-0.02329,0.04412]` + URDF `-pi/12` roll，手填、未标定 | Z-loc `body_calibration` | 数 cm 级平移、度级姿态，直接进入位姿输出 |
| 先验点云质量 | 来自 point_lio 建图，含动态残影 | `clean_map_bundle` + CloudCompare | 脏图会让所有配准系统性偏移 |
| point_lio 带先验图 | README 自述"比不带先验更容易飘" | 关掉 `prior_pcd` 试一次 | 可能是白捡的收益 |
| 时间同步 | point_lio 与点云同 stamp，问题不大 | legacy `max_interval` 收紧 | 影响配准成败 |

**只有把上面几条排干净，"换定位算法"的收益才可测量。**

---

## 1. 方案一：只换前端（ZLIO 替换 point_lio）

**替换边界**：`point_lio` 退出；`loam_interface`、`sensor_scan_generation`、`small_gicp`、2D 地图、导航全保留。

**数据流**

```
Mid360 → ZLIO(/lidar_odometry, /frontend/frame)
       → 适配节点：补 imu→front_mid360，输出 lidar_odometry(odom 系, child=front_mid360) + registered_scan
       → sensor_scan_generation（不变）→ small_gicp（不变）→ Nav2
```

**解决的精度问题**：局部里程计的漂移与退化场景鲁棒性。
**不解决**：全局定位可信度、丢失后恢复、长距离累计漂移（`small_gicp` 仍是单帧配准、无状态）。

**改动清单**
- 新增 `zl_odom_adapter` 节点（约 100 行）：订阅 `/lidar_odometry` + `/frontend/frame`，发布 `lidar_odometry`、`registered_scan`。
- `localization_launch.py`：`point_lio` 换 `frontend`，删 `prior_pcd_file` 传参。
- 地图资产、Nav2 参数、TF 结构全部不动。

**风险**：ZLIO 初始化依赖 20 帧联合估计 + 重力初猜，仿真点云是机械式 pattern，能否收敛未验证；收益不确定（作者自评前端"无显著优势"）。

**适用**：只想动最小范围，或先拿实机 bag 做一次 A/B 对比。

---

## 2. 方案二：只加后端（point_lio 保留）★ 推荐先做

**替换边界**：`small_gicp_relocalization` 退出（保留为降级兜底）；`point_lio`、`loam_interface`、`sensor_scan_generation`、Nav2、地图资产全保留。**不写业务代码，只改配置 + launch。**

**数据流**

```
Mid360 → point_lio → loam_interface ─┬─ lidar_odometry (odom 系, child=front_mid360)
                                     └─ registered_scan (odom 系点云)
                                          ↓ legacy 同步配对
                                   Z-loc backend → map→odom（仅在可信时发）
                                          ↓
                                   Nav2（其余不变）
```

**为什么这条最省**：`loam_interface` 的两个输出天然满足 legacy 协议——同一个源、同一个 stamp、点云在 odom 系，
配合 `input_cloud_in_local_frame: false` 后端能直接吃。不需要写适配节点。

**关键配置**（`config/backend.yaml` 基础上改）

```yaml
common:
  frontend: legacy              # 走 legacy 而不是 zlio
  odom_frame: odom
  sensor_frame: front_mid360    # 必须等于输入里程计的 child_frame_id
  map_frame: map
  odom_topic: /red_standard_robot1/lidar_odometry
  sync_odom_topic: /red_standard_robot1/lidar_odometry
  cloud_topic: /red_standard_robot1/registered_scan
  input_cloud_in_local_frame: false      # 关键：输入是 odom 系点云
  odom_cloud_sync:
    max_interval: 0.05
  body:
    frame: base_footprint       # 车体位姿直接给到导航要的坐标系
    rotation: {roll_deg: ?, pitch_deg: ?, yaw_deg: ?}   # 必须重标
    offset: {x: ?, y: ?, z: ?}                          # 含雷达到地面的 z
prior_map:
  enabled: true
  directory: <你的地图包>
uwb:
  enabled: false
output:
  legacy:
    publish_tf: false           # 必须关：否则 base_footprint 有两个父节点（map 和 odom）
```

**解决的精度问题**：全局累计漂移（因子图 + 回环）、初始定位与绑架恢复（描述子全局重定位）、
给出协方差让决策层降速、无先验图时可用 UWB 兜底。

**风险与对策**

| 风险 | 对策 |
| --- | --- |
| legacy 同步不上（点云与里程计戳差异 > 0.05 s） | 先用 `ros2 topic hz` + `ros2 topic echo --field header.stamp` 核一遍；不行就调 `max_interval` |
| namespace 前缀 | 导航全链路在 `/red_standard_robot1` 下，Z-loc 话题要写全名或给节点加 `__ns` |
| `map→odom` 空窗期 Nav2 停摆 | 加一个兜底节点：`tf_frozen` 时发最后可信值；同时把 `pose_trusted` 接进决策层 |
| 地图不同源 | 同一份精修 PCD 同时出 `pgm/yaml` 与 bundle，写基准点校验 |

**验证**：同一段 bag 回放，对比 z-loc 的 `~/localization_pose` 与 Nav2 里 `/amcl_pose` 或人工标注点位的误差。

---

## 3. 方案三：前后端全换（ZLIO + backend）★ 终态

**替换边界**：`point_lio`、`small_gicp` 均退出，Z-loc 成为唯一定位源。

**数据流**

```
Mid360 → ZLIO ─┬─ /frontend/frame (位姿 + imu 系去畸变点云) → backend（原生协议）
               └─ /lidar_odometry, /odometry (odom→imu)
backend → map→odom
适配节点 → lidar_odometry(odom→front_mid360) + registered_scan(odom 系) → sensor_scan_generation → terrain/Nav2
```

**收益**：前后端同源，建图-清理-定位一条流水线；reset/回环/UWB 语义一致；长期可维护性最好。
**代价**：需要写适配节点（把 `FrontendFrame.cloud`（imu 系）转成 `PointCloud2` 并补 `imu→front_mid360` 静态外参）；
外参、`body` 与 `base_footprint` 的 z 语义、地图资产全部要重做。

**建议**：**不要第一个做**。方案二做完后，所有中间产物（地图、外参、状态门控、TF 兜底）都能直接复用，
届时再换前端只是"换一个进程 + 加一个适配节点"。

---

## 4. 方案四：双定位 + 仲裁（不建议现在做）

两套并行，适配层按状态切 `map→odom` 来源（z-loc 可信用它，不可信切回 `small_gicp`）。

- 好处：赛场冗余，定位挂了不至于直接停摆。
- 代价：`map→odom` 只能有一个发布者，必须写仲裁节点 + 处理切换时的位姿跳变；算力与内存翻倍。
- 结论：等方案二跑稳、确认算力有余量后再考虑。

---

## 5. 对比

| | 方案一 只换前端 | 方案二 加后端 | 方案三 全换 | 方案四 双定位 |
| --- | --- | --- | --- | --- |
| 是否要写代码 | 适配节点 ~100 行 | **不需要** | 适配节点 ~150 行 | 仲裁节点 |
| 改配置/launch | 中 | 中 | 大 | 大 |
| 地图资产重做 | 否 | **是** | 是 | 是 |
| 外参重标 | 否 | **是** | 是 | 是 |
| 解决累计漂移 | 否 | **是** | 是 | 是 |
| 解决丢失恢复 | 否 | **是** | 是 | 是 |
| 精度提升预期 | 低-中 | **高** | 高 | 高 |
| 失败回退成本 | 低 | **低**（删 launch 行） | 高 | 中 |
| 建议顺序 | 备选 | **第一步** | 第二步 | 以后 |

**推荐路径：方案二 → 方案三。** 两者共用同一套后端与地图，
方案二做完后再切前端属于增量改造，不存在返工。

---

## 6. 落地顺序

1. 关掉 `point_lio` 的 `prior_pcd`，同 bag 对比一次漂移（10 分钟，可能白捡收益）。
2. 拆包名：删掉 `z-localization/src/livox_ros_driver2`（与导航侧同名不同版本），
   `backend`/`frontend`/`bringup` 改名（如 `zloc_backend`），确保能同时 `colcon build`。
3. 用 `body_calibration` 标真实外参，先用它把 `nav2_params.yaml` 的 `extrinsic_T/R` 修正——
   这一步不换任何算法就能验证"精度问题是不是外参造成的"。
4. 建图 → `clean_map_bundle` → CloudCompare 精修 → **一次生成** `pgm/yaml` 与 bundle。
5. 按方案二接后端，先只观察（不发 TF），对比两套位姿；确认稳定后再关 `small_gicp`、开放 `map→odom`。
6. 接 `pose_trusted` 到 `decision_navigator.py`：不可信 → 停车 → `/localization_reset` → 恢复。
7. 评估方案三。
