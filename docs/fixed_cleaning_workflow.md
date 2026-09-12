# 一个来回的固定清扫任务

使用原来的 `run_gen0_full_stack.sh` 启动。去程按指定顺序清扫 13 个垃圾，在东端路口完成一次前进掉头，回程清扫另外 5 个垃圾并停车。垃圾位置从所选场景 JSON 读取；落叶继续排除。启动和停止脚本已恢复，功能变更放在后端。

```bash
# 先正常停止旧仿真；需要从 SDF 原始出生位置开始。
./stop_gen0_full_stack.sh
GEN0_START_FIXED_ROUTE=true GEN0_START_NAV2=true GEN0_START_EPSILON=false ./run_gen0_full_stack.sh
```

首次使用新增代码需要重新构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gen0_main
source install/setup.bash
```

新增依赖为 `python3-scipy`（已列入 package.xml）。后端把启动脚本传入的速度限制到 0.45 m/s；转弯和接近垃圾时降低清扫中心参考速度到 0.20 m/s。参考进度会等待实际车辆跟上，所以这些不是保证达到的行驶速度。

## 规划与跟踪

- `config/fixed_cleaning_route.json` 记录去/回程垃圾实体顺序、起终点和东端掉头区域；垃圾坐标始终来自 `worlds/trash_scenarios/my_map/<scenario>.json`。
- `cleaning_trajectory.py` 按实际 SDF 轴距、轮距及转向上限构造轨迹。清扫点位于 base_link 前 1.75 m，以其经过垃圾中心为约束，再通过 Ackermann 无侧滑约束重建后轴姿态。
- 去回程使用清扫点插值，掉头使用前进圆弧连接。启动时检查所有目标的覆盖、顺序、曲率和世界网格截面的车身净空；没有可行连接就不启动行驶。
- `fixed_cleaning_controller.py` 使用清扫中心速度前馈和位置误差反馈，限制速度、加速度及曲率。参考进度基于 Gazebo 消息时间，仿真暂停不推进任务。
- 掉头连接段单独使用局部轨迹投影更新进度，以不超过 0.20 m/s 的清扫中心参考速度继续前进纠偏，不再因 8 cm 横向误差冻结参考点。此逻辑仅在去程最后一个垃圾后 0.30 m 至回程第一个垃圾前 0.80 m 启用；两侧垃圾清扫段保留原跟踪规则。0.20 m 偏差停车、实际覆盖确认和转向限幅仍然生效。
- PoseArray 的顺序并不稳定。固定任务默认从出生位姿识别世界车辆位姿，不再硬编码数组第 15 项。显式指定 `pose_index` 仍可用于诊断。
- 每个垃圾只有在实际清扫中心的相邻观测连线距目标不超过 0.10 m 时才记为覆盖。参考点经过不等于清扫完成。
- 跟踪误差大于 0.20 m、位姿突变、时间回退、漏过目标或出现第二个速度发布者会停车并报告 `failed`；不会盲目追踪身后的目标。
- 当前行人场景的实际 pose 话题用于前方停止区域检查。阻塞或数据过期时停车等待，不改变固定任务的去回程顺序。

## 清扫确认与诊断

保持原启动脚本的垃圾清除开关，后端不会自行开启清除。未设置 `GEN0_TRASH_CLEANUP=true` 时只验证行驶覆盖，不等待清除节点。开启后，`trash_cleanup_node` 使用车身完整覆盖垃圾的判据并调用 Gazebo remove 服务；仅在服务成功后更新清除反馈。

```bash
ros2 topic echo /gen0_cleaning/status
ros2 topic echo /gen0_cleaning/cleanup_status
```

`completed` 必须同时满足：18 个清扫目标全部实际覆盖、到达路线终点、垃圾清除节点确认剩余数量为 0。

主要状态：`waiting_pose`、`waiting_cleanup`、`waiting_obstacle`、`tracking`、`failed`、`incomplete_cleanup`、`completed`。`incomplete_cleanup` 表示行驶覆盖完成但实体清除未完成；不能视为任务成功。失败后先检查日志并重新从原始出生位置启动，不要在中途手动解除状态。

原入口 `run_gen0_full_stack.sh` 仍支持 `GEN0_START_FIXED_ROUTE=true`，按原有逻辑关闭 EPSILON 并把可选 Nav2 输出隔离到 `/preview/`。不要同时开启 mapping drive。未开启垃圾清除时只报告 `coverage_completed`，不会报告完整清扫成功。Nav2、YOLO 和窗口开关均保持原脚本行为。

参考路径 `/gen0_cleaning/reference_path` 使用 `gen0_world` 世界坐标帧。该帧不能直接冒充重定位后的 `map`；在 RViz 中使用前需要正确的世界到导航帧变换。主要运行日志为 `runtime_logs/fixed_cleaning_controller.log` 和 `runtime_logs/trash_cleanup.log`。

## 验证范围

```bash
PYTHONPATH=gen0_gz_sim_ros2/gen0_main python3 -m pytest \
  gen0_gz_sim_ros2/gen0_main/test/test_cleaning_trajectory.py -q
```

回归覆盖完整去回程、全部垃圾实际经过、执行器延迟、有限转向速度、小幅观测噪声、静态边界、错误起点、时间回退、漏扫拦停和位姿数组索引变化。

真实 Gazebo 静态场景整程验证（独立 ROS 域及 Gazebo 分区、无界面，不操作现有仿真）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/verify_gen0_cleaning_gazebo.py
```

使用实际世界碰撞网格、车辆质量/关节/碰撞形状及 Ackermann 插件，保留全部垃圾实体，省略渲染传感器和动态行人。验证成功必须出现 `PHYSICS AND CLEANUP COMPLETE`。日志默认写入 `/tmp/gen0_cleaning_gazebo_validation`。

网格检查使用多个离散高度截面及离散车身采样，不是完整三维连续碰撞证明。控制器没有任意场景的动态绕行或自动补扫能力：超出已验证条件时会停止并报告，而不是保证继续运动。圆弧连接处的有限执行器响应已纳入离线闭环测试，真实运行表现还取决于仿真时序和底盘参数。
