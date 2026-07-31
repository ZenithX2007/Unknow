# SCURM Nav2 迁移差距审计

源项目: `/home/zjxue2007/SCURM_SentryNavigation`

当前项目: `/home/zjxue2007/Unknow`

## 结论

当前 Gen0 Nav2 不是 SCURM 导航栈的等价移植。它只是复用了部分
SCURM 包和少量参数，再按 Gen0 Ackermann 车辆做了临时改造。这个
结构会启动，但控制行为和 SCURM 原版不一致，尤其在局部跟踪和地图
约束上会明显退化。

## 关键差距

| 模块 | SCURM 原版 | 当前 Gen0 | 问题 |
| --- | --- | --- | --- |
| 底盘模型 | `motion_model: Omni`，输出 `vx/vy/wz` | `motion_model: Ackermann`，只用 `vx/wz` | 不能直接照抄 SCURM 控制参数；需要 Gen0 专用控制器/适配逻辑。 |
| 底盘命令链 | `/cmd_vel -> twist_transformer -> /cmd_vel_in_yaw -> ChassisCmd` | `/cmd_vel_nav -> velocity_smoother -> /control/cmd_vel -> cmdvel_to_vehicle` | 缺少 SCURM 的 yaw-frame 速度转换语义；现有链路是车辆关节适配，不是 sentry chassis 适配。 |
| 控制频率 | controller/velocity smoother 80 Hz | controller 10 Hz, smoother 20 Hz | MPPI 预测窗和采样节奏完全不同，不能期待同样效果。 |
| MPPI 参数 | `time_steps=180`, `model_dt=0.014`, `vx/vy_std=0.78`, `ObstaclesCritic`, `TwirlingCritic` | `time_steps=50`, `model_dt=0.1`, `vx_std=0.16`, `CostCritic`, `PathAngleCritic`, `PreferForwardCritic` | 当前参数是另一个控制器调法，不是 SCURM 调法。 |
| 非标准 MPPI 参数 | `adjustThre`, `minAngleDiff`, `adjustHeadingSpeed` | 未实现对应逻辑 | 这些参数不是标准 Humble MPPI 常规参数；若 SCURM 依赖改版 MPPI，当前系统不会执行这段逻辑。 |
| Planner | `ThetaStarPlanner` | `SmacPlannerHybrid` | 当前更像车辆运动学路径搜索，但不是 SCURM 的全局规划结构。 |
| Smoother | `ConstrainedSmoother` | `SimpleSmoother` | SCURM 规划后会平滑路径；当前路径平滑能力不足。 |
| Local costmap frame | `global_frame: map` | 基础配置为 `odom`，overlay 也是局部改写 | 和 SCURM 的 costmap 坐标假设不一致。 |
| Local obstacle layer | `costmap_intensity` on `/terrain_map` | 默认 LaserScan；`scurm_terrain` 只是 overlay | SCURM terrain pipeline 没有作为默认主路径接入。 |
| Global costmap | 静态 2D map + inflation | 默认生成空白 `/tmp/gen0_my_map_nav.yaml` | 没有真正把 Gen0/SCURM 地图接入全局规划；全局 planner 看不到道路/墙体约束。 |
| `/projected_map` | SCURM 原版不靠这个临时图 | 当前生成但 Nav2 StaticLayer 未订阅 | “和我们的地图结合”没有完成。 |
| Recovery | `BackUpTwzFree` 输出 `linear.x/y` 往空旷区域平移 | 为 Ackermann 改过插件，但 BT 中又禁用了主动恢复 | 原版全向 recovery 不能直接用于 Gen0；需要 Ackermann 弧线恢复，并在 BT 中恢复使用。 |
| Relocalization | 好 prior PCD + ICP + FAST-LIO | prior map 不可靠时发散 | 不能用坏 PCD 验证 Nav2；需要先解决地图/定位闭环。 |

## 当前最影响效果的点

1. 全局地图没有接入。默认 blank map 只能防止 costmap out-of-bounds，
   不能提供可导航道路/障碍物约束。

2. 控制器不是 SCURM 控制器。SCURM 的 MPPI 是全向 sentry 底盘参数；
   Gen0 是四轮转向 Ackermann 近似，必须重新设计参数，不能直接套原值。

3. terrain_analysis 已经跑起来，但 Nav2 只在 `GEN0_NAV2_COSTMAP_SOURCE=scurm_terrain`
   时用 overlay 接入，而且 global/local 的 frame 和 static map 关系仍不完整。

4. SCURM 的 recovery 是全向平移逻辑。Gen0 需要“找空旷方向 -> 转成前进/后退弧线”
   的恢复动作，否则局部失败时要么乱动，要么完全不恢复。

## 建议迁移顺序

1. 先固定地图输入。
   当前交付目标是 `my_map`。在 prior yaml/pcd 还不可靠时，让 Nav2
   StaticLayer 直接订阅在线 `/projected_map`，不要再用默认 blank map
   或不对齐的旧 prior map 判断导航效果。

2. 建一个 `nav2_gen0_scurm_params.yaml`，从 SCURM 参数出发，只保留必须
   因 Gen0 车辆运动学变化而改的项。不要在同一个文件里混杂 LaserScan、
   blank map、Ackermann MPPI 临时调参。

3. 恢复 SCURM BT 的 `ComputePath -> SmoothPath -> FollowPath` 结构，使用
   `ThetaStarPlanner + ConstrainedSmoother` 作为第一版对齐基线。

4. 把 local costmap 主路径切到 `costmap_intensity` + `/gen0_mapping/terrain_map`。
   global costmap 用真实静态 map；等 `/projected_map` 稳定后再考虑动态融合。

5. 单独重写 Ackermann recovery。不要直接使用 SCURM 原版 `linear.y` 恢复。

6. 最后再调 MPPI/Ackermann 参数。先保证 map、costmap、BT、速度链路符合架构，
   再看 `vx/wz` 和 steering 几何。

## 下一步实现目标

下一步应停止继续调当前 `nav2_gen0_params.yaml` 的小参数。应新增一个
SCURM-aligned Gen0 导航 profile，并显式启动它：

```bash
GEN0_WORLD=my_map \
GEN0_NAV2_PROFILE=scurm_gen0 \
GEN0_NAV2_MAP_SOURCE=projected_map \
GEN0_NAV2_COSTMAP_SOURCE=scurm_terrain \
./run_gen0_nav2.sh
```

这个 profile 的成功标准不是“节点 active”，而是：

- `/global_costmap` 来自在线 `/projected_map` relay，不是 blank map 或错误 prior map。
- `/local_costmap` 使用 `/gen0_mapping/terrain_map`。
- `/plan` 能绕开静态障碍。
- `/cmd_vel_nav` 没有 `linear.y`。
- `/control/cmd_vel` 与 steering joint 和 FAST-LIO yaw 符号一致。

## 第一阶段实现

已新增第一版 profile：

```text
gen0_gz_sim_ros2/gen0_main/config/nav2_gen0_scurm_params.yaml
gen0_gz_sim_ros2/gen0_main/behavior_tree/ackermann_scurm_recovery.xml
gen0_gz_sim_ros2/gen0_main/behavior_tree/ackermann_scurm_through_poses.xml
```

`run_gen0_nav2.sh` 新增：

```bash
GEN0_NAV2_PROFILE=scurm_gen0
```

该 profile 当前做了这些对齐：

- 默认 `GEN0_NAV2_COSTMAP_SOURCE=scurm_terrain`。
- 默认 `GEN0_NAV2_LOCALIZATION_MODE=odom_only`，用于没有可靠 prior PCD 时的短距离验证。
- 默认 `GEN0_NAV2_MAP_SOURCE=projected_map`，用在线 `/projected_map` relay 到
  `/map`，绕开当前不可用或不对齐的 prior yaml/pcd。
- Planner 切到 `nav2_theta_star_planner/ThetaStarPlanner`。
- Smoother 切到 `nav2_constrained_smoother/ConstrainedSmoother`。
- BT 保留 SCURM 的 `ComputePath -> SmoothPath -> FollowPath` 结构；原版
  `GoalUpdater` 在 Gen0 当前 Nav2 栈中会阻塞普通 goal 接收，已作为适配项移除。
- Local costmap 使用 `costmap_intensity::ObstacleLayerIntensity` on `/gen0_mapping/terrain_map`。
- Global costmap 使用在线 `/projected_map` 作为 StaticLayer 输入；map_server 发布到
  `/map_yaml_unused`，避免 prior map 污染验证。
- MPPI 仍保留 Gen0 Ackermann 约束：`linear.y=0`、`min_turning_r=4.5`、`wz_max=0.12`。

这只是第一阶段。它把架构拉回 SCURM 主线，但还没有完成最终调参和
地图坐标校准。当前端到端验证结果：

- `/map_server topic_name` 为 `/map_yaml_unused`，`/map` 来自在线 `/projected_map`。
- `/planner_server GridBased.plugin` 为 `ThetaStarPlanner`。
- `/smoother_server SmoothPath.plugin` 为 `ConstrainedSmoother`。
- `/controller_server FollowPath.motion_model` 为 `Ackermann`。
- 普通 `NavigateToPose` action 和 `/goal_pose` 入口均能触发规划、平滑、跟踪并到达目标。
