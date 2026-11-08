# 关闭 small_gicp 重定位操作指南（实车）

> 面向对象：另一台机器上的 AI 编码助手 / 工程师
> 目标：在 **实车** 导航中关闭 `small_gicp_relocalization` 重定位功能，改为发布恒等 `map -> odom` 静态变换，其余导航功能（Nav2、代价地图、决策节点）保持正常。
> 适用工程：`pb2025_sentry_nav`（本指南基于该仓库实际验证过的改动，已在仿真与实车两种 launch 上验证）

---

## 0. 必读：核心约束（不遵守会导致 Nav2 全面失效）

**`small_gicp_relocalization` 是当前工程中唯一发布 `map -> odom` 变换的节点。**

证据（源码）：

```cpp
// small_gicp_relocalization/src/small_gicp_relocalization.cpp
void SmallGicpRelocalizationNode::publishTransform()
{
  ...
  transform_stamped.header.frame_id = map_frame_;    // "map"
  transform_stamped.child_frame_id = odom_frame_;    // "odom"
  ...
  tf_broadcaster_->sendTransform(transform_stamped);
}
```

**因此绝对不可以"直接删掉 small_gicp 的启动项"**。一旦删除且没有替代发布者：
- `map` 坐标系不存在
- `global_costmap` 报 `Timed out waiting for transform from xxx to map`
- `local_costmap` 的 `StaticLayer` 报 `"map" passed to lookupTransform argument target_frame does not exist`
- `map_server` 无法正常提供静态地图
- Nav2 的 goal 下发、路径规划全部失败

**正确做法**：关闭 small_gicp 的同时，用一个 `static_transform_publisher` 发布 **恒等** 的 `map -> odom`。

---

## 1. 实现原理

```
关闭后：  map  ──(静态恒等 TF)──>  odom  ──(point_lio 里程计)──>  chassis/base
```

- `map` 原点 == `odom` 原点
- 定位完全依赖 `point_lio` 的里程计，**会随时间累积漂移**（这是本方案的已知代价）
- 决策节点若把 odom 坐标直接当 map 坐标用（本工程 `decision_navigator.py` 即如此），则无需改动

---

## 2. 改动总览（共 5 个文件）

| # | 文件 | 作用 |
|---|---|---|
| 1 | `pb2025_nav_bringup/launch/localization_launch.py` | **核心**：新增开关、条件启停 small_gicp、新增静态 TF |
| 2 | `pb2025_nav_bringup/launch/bringup_launch.py` | 透传 `use_relocalization` |
| 3 | `pb2025_nav_bringup/launch/rm_navigation_reality_launch.py` | 实车顶层入口，声明并透传参数 |
| 4 | `pb2025_nav_bringup/config/reality/nav2_params.yaml` | 注释掉 small_gicp 参数段 |
| 5 | `pb2025_nav_bringup/package.xml` | small_gicp 依赖改为非强制 |

> 若那台电脑也需要跑仿真，还要对 `rm_navigation_simulation_launch.py` 和
> `config/simulation/nav2_params.yaml` 做同样处理（步骤 3、4 同理）。

---

## 3. 操作步骤

### 步骤 0：先做 Git 备份（**务必执行**）

```bash
cd src/pb2025_sentry_nav

# 若 git 未配置身份，先在本仓库局部配置（不要用 --global）
git config user.name "local-backup"
git config user.email "local-backup@localhost"

# 创建备份分支（保留当前所有未提交改动）
git branch backup/before-disable-small-gicp-$(date +%Y%m%d-%H%M%S)
git add -A
git commit -q -m "backup: snapshot before disabling small_gicp relocalization"
git log --oneline -2
```

回滚方式：

```bash
git checkout backup/before-disable-small-gicp-<时间戳> -- .
```

---

### 步骤 1：修改 `localization_launch.py`

文件：`pb2025_nav_bringup/launch/localization_launch.py`

#### 1.1 导入 `UnlessCondition`

```python
# 修改前
from launch.conditions import IfCondition

# 修改后
from launch.conditions import IfCondition, UnlessCondition
```

#### 1.2 读取 launch 参数

在 `log_level = LaunchConfiguration("log_level")` 之后新增：

```python
use_relocalization = LaunchConfiguration("use_relocalization")
```

#### 1.3 声明 launch 参数

在 `declare_container_name_cmd` 之后新增：

```python
declare_use_relocalization_cmd = DeclareLaunchArgument(
    "use_relocalization",
    default_value="True",
    description=(
        "Whether to run small_gicp_relocalization. If False, small_gicp is not launched and a "
        "static identity tf (map->odom) is published instead, so that the odom origin "
        "coincides with the map origin."
    ),
)
```

并在 `ld.add_action(declare_container_name_cmd)` 之后加入：

```python
ld.add_action(declare_use_relocalization_cmd)
```

#### 1.4 给 small_gicp 节点加条件（在 `load_nodes` 的 `GroupAction` 内）

```python
Node(
    package="small_gicp_relocalization",
    executable="small_gicp_relocalization_node",
    name="small_gicp_relocalization",
    output="screen",
    condition=IfCondition(use_relocalization),   # <== 新增这一行
    respawn=use_respawn,
    respawn_delay=2.0,
    parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
    arguments=["--ros-args", "--log-level", log_level],
),
```

#### 1.5 删除组合模式下的 small_gicp `ComposableNode`

在 `load_composable_nodes = LoadComposableNodes(...)` 的 `composable_node_descriptions` 列表中，
**删除** 这一段（否则组合模式下 small_gicp 不受开关控制）：

```python
# 删除这一段
ComposableNode(
    package="small_gicp_relocalization",
    plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
    name="small_gicp_relocalization",
    parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
),
```

> 保留 `MapServer` 与 `LifecycleManager` 两个 `ComposableNode` 不动。

#### 1.6 【关键】新增静态 `map -> odom` 发布节点

**必须定义在 `load_nodes` 这个 `GroupAction` 之外**，否则组合模式（`use_composition=True`，实车默认就是 True）下整组不执行，静态 TF 不会启动 —— 这是最容易踩的坑，详见第 5 节。

```python
# NOTE: static_transform_publisher is NOT available as a composable node, so it has to be
# launched as a standalone process in both composition and non-composition modes.
# It is placed outside `load_nodes` so that it is started regardless of `use_composition`.
start_static_map_to_odom_node = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher_map2odom",
    output="screen",
    condition=UnlessCondition(use_relocalization),
    # Must follow the same clock as the rest of the stack, otherwise the
    # static transform stamped with wall-clock time cannot be matched
    # against simulated-time lookups and `map` appears to not exist.
    parameters=[{"use_sim_time": use_sim_time}],
    arguments=[
        "--x", "0.0",
        "--y", "0.0",
        "--z", "0.0",
        "--roll", "0.0",
        "--pitch", "0.0",
        "--yaw", "0.0",
        "--frame-id", "map",
        "--child-frame-id", "odom",
    ],
)
```

#### 1.7 注册该节点

在 `ld.add_action(start_point_lio_node)` 之后、`ld.add_action(load_nodes)` 之前插入：

```python
ld.add_action(start_point_lio_node)
ld.add_action(start_static_map_to_odom_node)   # <== 新增
ld.add_action(load_nodes)
ld.add_action(load_composable_nodes)
```

---

### 步骤 2：透传参数 `bringup_launch.py`

文件：`pb2025_nav_bringup/launch/bringup_launch.py`

1. 读取参数：
```python
use_relocalization = LaunchConfiguration("use_relocalization")
```

2. 声明参数（放在 `declare_use_respawn_cmd` 之后）：
```python
declare_use_relocalization_cmd = DeclareLaunchArgument(
    "use_relocalization",
    default_value="True",
    description=(
        "Whether to run small_gicp_relocalization. If False, small_gicp is not launched and a "
        "static identity tf (map->odom) is published instead, so that the odom origin "
        "coincides with the map origin."
    ),
)
```

3. 注册：
```python
ld.add_action(declare_use_respawn_cmd)
ld.add_action(declare_use_relocalization_cmd)   # <== 新增
ld.add_action(declare_log_level_cmd)
```

4. 传递给 `localization_launch.py` 的 `IncludeLaunchDescription.launch_arguments`：
```python
"use_composition": use_composition,
"use_respawn": use_respawn,
"use_relocalization": use_relocalization,   # <== 新增
"container_name": "nav2_container",
```

---

### 步骤 3：实车顶层入口 `rm_navigation_reality_launch.py`

文件：`pb2025_nav_bringup/launch/rm_navigation_reality_launch.py`

1. 读取参数（`slam = LaunchConfiguration("slam")` 之后）：
```python
use_relocalization = LaunchConfiguration("use_relocalization")
```

2. 声明参数（`declare_slam_cmd` 之后）。

   > **重要**：如果目标机器希望"**默认就关闭**"（不做任何命令行传参也生效），
   > 必须把 `default_value` 设为 `"False"`。
   > 若设为 `"True"`，则每次实车启动都要手写 `use_relocalization:=False`。

```python
declare_use_relocalization_cmd = DeclareLaunchArgument(
    "use_relocalization",
    default_value="False",     # 实车"默认关闭"用 False；保留原行为则用 "True"
    description=(
        "Whether to run small_gicp_relocalization. If False, small_gicp is not launched and a "
        "static identity tf (map->odom) is published, so that the odom origin coincides with "
        "the map origin and only point_lio odometry drifts."
    ),
)
```

3. 注册：
```python
ld.add_action(declare_slam_cmd)
ld.add_action(declare_use_relocalization_cmd)   # <== 新增
```

4. 透传给 `bringup_launch.py`：
```python
"use_composition": use_composition,
"use_respawn": use_respawn,
"use_relocalization": use_relocalization,   # <== 新增
```

> 若那台电脑还要改仿真入口，对 `rm_navigation_simulation_launch.py` 做同样 4 处改动。

---

### 步骤 4：清理 `config/reality/nav2_params.yaml`

文件：`pb2025_nav_bringup/config/reality/nav2_params.yaml`

将 `small_gicp_relocalization:` 整段**注释掉**（不要删除，便于回退）：

```yaml
# NOTE: small_gicp_relocalization is DISABLED when launched with `use_relocalization:=False`.
# When disabled, a static identity tf (map->odom) is published instead, so the odom origin
# coincides with the map origin and only point_lio odometry drifts.
# To re-enable it, launch with `use_relocalization:=True` and uncomment below.
# small_gicp_relocalization:
#   ros__parameters:
#     enable_relocalization: true
#     use_sim_time: False
#     num_threads: 4
#     num_neighbors: 20
#     global_leaf_size: 0.1
#     registered_leaf_size: 0.05
#     max_dist_sq: 3.0
#     map_frame: "map"
#     odom_frame: "odom"
#     base_frame: "base_footprint"
#     robot_base_frame: "gimbal_yaw"
#     lidar_frame: "front_mid360"
#     init_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     # prior_pcd_file: ""
```

> 注意：**YAML 里 `#` 注释的整段必须全部注释**，`ros__parameters` 缩进不能破坏其它节点段落。

---

### 步骤 5：调整 `package.xml`

文件：`pb2025_nav_bringup/package.xml`

把强制的 exec_depend 移除，改为注释说明（避免没装 small_gicp 时构建失败）：

```xml
<exec_depend>pb_omni_pid_pursuit_controller</exec_depend>
<!--
  small_gicp_relocalization is optional: only required when launching with
  use_relocalization:=True. It is intentionally NOT declared as an exec_depend
  so that the package can be built/run without it when relocalization is disabled.
-->
<exec_depend>pointcloud_to_laserscan</exec_depend>
```

> `small_gicp_relocalization` 的 `CMakeLists.txt` 会通过 `FetchContent` 联网拉取
> `koide3/small_gicp`，在离线机器上容易构建失败。若那台机器不需要重定位，
> 更彻底的做法是在 `src/pb2025_sentry_nav/` 下新建 `small_gicp_relocalization/COLCON_IGNORE`
> 让 colcon 跳过该包。

---

## 4. 验证步骤

### 4.1 静态语法检查

```bash
cd src/pb2025_sentry_nav/pb2025_nav_bringup

python3 -m py_compile \
  launch/localization_launch.py \
  launch/bringup_launch.py \
  launch/rm_navigation_reality_launch.py

python3 -c "import yaml; yaml.safe_load(open('config/reality/nav2_params.yaml')); print('YAML OK')"
python3 -c "import xml.dom.minidom as m; m.parse('package.xml'); print('XML OK')"
```

### 4.2 参数是否注册成功

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 launch --show-args pb2025_nav_bringup rm_navigation_reality_launch.py | grep -A3 use_relocalization
```

期望看到 `use_relocalization` 及其默认值（实车按步骤 3 的描述为 `'False'`）。

### 4.3 实际启动验证（实车）

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  use_relocalization:=False > /tmp/disable_gicp.log 2>&1 &
sleep 30
```

逐项检查：

```bash
# ① 静态 TF 进程必须存在（只应有 1 个）
ps -eo comm= | grep -cx static_transform_publisher     # 期望 = 1

# ② 确认它确实在发布 map->odom
grep -A3 "publishing transform" /tmp/disable_gicp.log
# 期望输出：
#   from 'map' to 'odom'

# ③ small_gicp 完全没启动
grep -c "small_gicp" /tmp/disable_gicp.log              # 期望 = 0

# ④ 没有任何 map 坐标系缺失错误
grep -c 'Invalid frame ID "map"' /tmp/disable_gicp.log  # 期望 = 0

# ⑤ map_server 正常激活
grep -E "Activating map_server|Managed nodes are active" /tmp/disable_gicp.log

# ⑥ TF 查询成功（恒等变换）
ros2 run tf2_ros tf2_echo map odom
# 期望 Translation [0.000, 0.000, 0.000]，Quaternion [0,0,0,1]
```

> `tf2_echo` 启动后的第一行常常是
> `Waiting for transform map -> odom: Invalid frame ID "map" ... does not exist`，
> 这是等待 DDS 发现阶段的**正常输出**，等 2~3 秒拿到 Translation 即为成功。

### 4.4 导航功能回归

```bash
# 节点全部在运行
ros2 node list | grep -E "map_server|controller_server|planner_server|bt_navigator"

# 关键：局部/全局代价地图不再报 map 缺失（连续运行 60s 后再看）
grep -c 'StaticLayer: "map" passed' /tmp/disable_gicp.log   # 期望 = 0
grep -c 'Timed out waiting for transform.*to map' /tmp/disable_gicp.log  # 期望 = 0
```

最后在 RViz 中确认：
- 地图正常显示
- 机器人位姿正常
- 可以正常下发导航目标并被规划器响应

---

## 5. 三个必须避开的坑（本指南踩过的实际故障）

### 坑 1：静态 TF 被放进 `GroupAction` 里 → 组合模式下完全不启动

```
错误：  load_nodes = GroupAction(condition=IfCondition(not use_composition),
                                actions=[ ..., static_transform_publisher, ... ])
```

实车 `use_composition` 默认是 `True`，此时 `load_nodes` 整组被禁用，静态 TF 不启动，
于是 `map` 仍然不存在。

**正确做法**：静态 TF 必须是 `LaunchDescription` 的顶层 action（见步骤 1.7）。
原因：`static_transform_publisher` 没有对应的 composable 版本，无法放进
`LoadComposableNodes`，只能作为独立进程运行。

### 坑 2：静态 TF 没继承 `use_sim_time`

- 仿真：其余节点用 Gazebo 时钟，静态 TF 用墙钟 → 时间戳不匹配 → `map` 查不到
- 即使实车 `use_sim_time=False`，也建议统一显式传入，避免两台机器行为不一致

**必须写**：

```python
parameters=[{"use_sim_time": use_sim_time}],
```

### 坑 3：组合模式里遗留了 small_gicp 的 `ComposableNode`

原工程 `load_composable_nodes` 中有一段 small_gicp 的 `ComposableNode`，它不受任何
开关控制。若不删除，`use_relocalization:=False` 时它仍会加载，并与静态 TF
同时发布 `map -> odom`，产生**双发布者**导致定位跳变。

**必须删除**（见步骤 1.5）。

---

## 6. 最终效果与已知影响

### 关闭后的坐标链
```
map ──恒等静态TF(map->odom)──> odom ──point_lio──> chassis ──URDF静态TF──> gimbal_yaw / lidar
```

### 已知影响（需向使用者说明）

| 影响项 | 说明 |
|---|---|
| 定位精度 | 完全依赖 `point_lio`，**长时间运行会累积漂移**，不会自动纠正 |
| `initialpose` | 关闭后不生效（该话题原本仅由 small_gicp 订阅），RViz 的 `2D Pose Estimate` 无实际作用 |
| 地图对齐 | 依赖 `point_lio` 启动时原点与地图原点重合；若不一致需调整地图或起始位姿 |
| 决策节点 | 若按"odom 坐标即 map 坐标"处理（本工程 `decision_navigator.py` 就是），则无需改动 |

### 如何恢复 small_gicp

```bash
# 1. 解锁 config/reality/nav2_params.yaml 中被注释的 small_gicp 参数段
# 2. 启动时显式开启
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py use_relocalization:=True
```

### 回滚全部改动

```bash
cd src/pb2025_sentry_nav
git checkout backup/before-disable-small-gicp-<时间戳> -- .
```

---

## 7. 自检清单（交给 AI 逐条确认）

- [ ] 已创建 Git 备份分支
- [ ] `localization_launch.py`：`UnlessCondition` 已导入
- [ ] `localization_launch.py`：`declare_use_relocalization_cmd` 已声明并 `add_action`
- [ ] `localization_launch.py`：small_gicp 节点带 `condition=IfCondition(use_relocalization)`
- [ ] `localization_launch.py`：`load_composable_nodes` 中已删除 small_gicp 的 `ComposableNode`
- [ ] `localization_launch.py`：静态 TF 节点定义在 `load_nodes` **之外**，且带 `parameters=[{"use_sim_time": use_sim_time}]`
- [ ] `localization_launch.py`：`ld.add_action(start_static_map_to_odom_node)` 已加
- [ ] `bringup_launch.py`：参数已声明 + 已注册 + 已透传给 localization
- [ ] `rm_navigation_reality_launch.py`：参数已声明 + 已注册 + 已透传给 bringup
- [ ] `nav2_params.yaml`：small_gicp 段已完整注释，YAML 仍能解析
- [ ] `package.xml`：已移除强制依赖
- [ ] `py_compile` 全部通过
- [ ] 启动后 `ps -eo comm= | grep -cx static_transform_publisher` = 1
- [ ] 启动后 `grep -c small_gicp 日志` = 0
- [ ] 启动后 `grep -c 'Invalid frame ID "map"' 日志` = 0
- [ ] `ros2 run tf2_ros tf2_echo map odom` 能拿到恒等变换
- [ ] 代价地图无 `map` 相关报错，可正常下发导航目标
