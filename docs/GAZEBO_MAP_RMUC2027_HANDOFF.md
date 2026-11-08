# 提示：把 RMUC 2027 场地模型放入 Gazebo

## 任务

将 RMUC 2027 比赛场地 3D 模型导入 Gazebo，使其可视化正确、具备可用的碰撞、且坐标与本项目仿真逻辑一致。

## 源文件

`frontend/public/assets/map-rmuc2027.glb`

## 已知事实（已经二进制解析核实，可直接依赖，无需再猜）

- 格式：glTF 2.0 二进制（`.glb`），Blender 导出，贴图内嵌，无 Draco 压缩，8.35 MB。
- 根节点 `ArenaChapter4`，无旋转、无缩放、中心位于原点。
- 单位是**米**；坐标轴是 **Y-up**（Gazebo 是 Z-up）。
- 场地比赛区：**28.00 m × 15.00 m**。
- 整体包围盒：**28.25 m × 15.25 m × 高约 3.516 m**；X ∈ [-14.125, 14.125]，横向 ∈ [-7.625, 7.625]，高度 ∈ [-0.24, 3.276]。
- 场地底板 `ArenaChapter4_FieldBlack` = 28.00 × 0.28 × 15.00 m；外围墙厚度 0.125 m（`GraphiteArmor` 为 28.25 × 15.25 外框）。
- 模型还包含红/蓝涂装、中心高地与斜坡、飞镖站盖，以及大量 0.07–0.15 m 级的装饰小件。

## 逻辑侧坐标（`config/rules.json` → `maps.rmuc2027`）

- `width: 28`，`height: 15`，坐标系为**左上原点**的米制坐标，场地即 [0, 28] × [0, 15]。
- glb 中心在原点，因此放置时需平移 **(+14, +7.5)** 才能与逻辑坐标对齐。
- 结构件在逻辑坐标中的位置：红方基地 (2.411, 7.5)、蓝方基地 (25.589, 7.5)、红方前哨 (10.976, 11.4)、蓝方前哨 (17.024, 3.6)、符文 (14.0, 7.5)、z ≈ 2.896、yaw = π/4。
- 结构件模型：`base-ch4.glb`、`outpost-ch4.glb`、`rune-ch4v2.glb`、`tech-core-ch4v2.glb`。
- 地形高度与各平台/斜坡的多边形边界在 `config/rules.json` → `maps.rmuc2027.terrain`（每项含 `polygon`、`elevation`、`slope_deg`、`ramp_start/ramp_end`），是权威数据源。
- 规则书尺寸：`field_length_m: 28`、`field_width_m: 15`、`fence_height_m: 2.4`（`rulebook_2027.match`）。

## 必须满足的约束

- Gazebo 不能直接加载 `.glb`，需自行转换为其支持的网格格式。
- glTF 是 Y-up，Gazebo 是 Z-up，坐标系必须转换正确，否则场地会"立起来"。
- 两侧单位都是米，**不要**做缩放。
- 整张高精度网格不能直接用作 collision（26 个 mesh，会导致加载慢、机器人卡在细节里）；collision 需要简化。
- 视觉与碰撞分离：visual 保留完整外观，collision 只保留地面、围墙、高地、斜坡、台阶、底座等大体量结构；涂装、传感器玻璃、能量单元、资源站等小件不需要碰撞。
- 场地中心必须落在逻辑坐标 (14, 7.5)，否则所有机器人会整体偏移半个场地。
- 不得修改本项目源码，不得重新导出或替换 glb。

## 验收标准

- world 能无报错加载，场地量取约 28 × 15 m。
- 测试刚体放在场地内不掉穿；放在中心高地上方时落在抬高后的表面（约 0.3–0.9 m），而不是 0。
- 机器人放在逻辑坐标 (2.411, 7.5) 时站在红方基地旁，不悬空、不穿墙。

## 参考

- 场地模型：[`frontend/public/assets/map-rmuc2027.glb`](../frontend/public/assets/map-rmuc2027.glb)
- 场地定义：`config/rules.json` → `maps.rmuc2027`
- 规则书尺寸：`config/rules.json` → `rulebook_2027.match`
- 仿真引擎场地结构：[`simulation/src/lib.rs`](../simulation/src/lib.rs:117) 的 `struct Arena { width, height, ... }`
- 前端地图加载：[`frontend/src/ArenaScene.tsx`](../frontend/src/ArenaScene.tsx:450)
