/**
 * @file map_adapter.cc
 * @author GW
 * @brief 地图适配器实现：将集成地图（IntegratedMap）适配为规划器所需的接口
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件实现了 `SscPlannerAdapter` 类，应用了**适配器模式（Adapter Pattern）**。
 *
 * **核心功能**：
 * 1. **接口隔离**：将底层 `IntegratedMap` 转换为 `SscPlannerMapItf`
 * 2. **数据访问封装**：统一提供 ego、参考车道、障碍物与前向轨迹查询
 * 3. **有效性检查**：通过 `IsValid()` 防止规划器访问未初始化地图
 *
 * **设计意图**：
 * 解耦规划器核心算法与具体的地图数据结构。当底层地图格式变化时，只需修改适配器，无需改动规划器核心代码。
 */
#include "ssc_planner/map_adapter.h"

namespace planning {

/**
 * @brief 设置底层地图数据
 * @param map 指向 IntegratedMap 的共享指针
 * @return ErrorType
 *
 * **功能**：
 * 更新适配器内部维护的语义地图快照，并标记为可用。
 */
ErrorType SscPlannerAdapter::set_map(std::shared_ptr<IntegratedMap> map) {
  map_ = map;  // maintain a snapshop of the environment
  is_valid_ = true;
  return kSuccess;
}

/**
 * @brief 检查地图数据是否有效
 * @return bool true表示有效，false表示无效
 *
 * **使用场景**：
 * 在进行任何查询操作前，应检查此标志，防止访问未初始化的空指针。
 */
bool SscPlannerAdapter::IsValid() { return is_valid_; }

/**
 * @brief 获取地图时间戳
 * @return decimal_t 时间戳（秒）
 *
 * **功能**：
 * 返回集成地图生成的时间，用于时间同步和轨迹拼接。
 */
decimal_t SscPlannerAdapter::GetTimeStamp() { return map_->time_stamp(); }

/**
 * @brief 获取自车对象（包含完整参数）
 * @param vehicle 输出参数，Vehicle对象
 * @return ErrorType
 */
ErrorType SscPlannerAdapter::GetEgoVehicle(Vehicle* vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

/**
 * @brief 获取自车状态（位置、速度、加速度等）
 * @param state 输出参数，State对象
 * @return ErrorType
 */
ErrorType SscPlannerAdapter::GetEgoState(State* state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

/**
 * @brief 获取局部参考车道
 * @param lane 输出参数，参考车道
 * @return ErrorType
 *
 * **功能**：
 * 获取自车当前的参考车道（Reference Lane）。
 * 这通常是行为规划（Behavior Planning）层选定的目标车道或当前跟随车道。
 * SSC规划器将基于此车道构建Frenét坐标系。
 */
ErrorType SscPlannerAdapter::GetLocalReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  return kSuccess;
}

/**
 * @brief 获取前向轨迹集合（用于交互或预测）
 * @param behaviors 输出参数，周边车辆的行为列表
 * @param trajs 输出参数，周边车辆的轨迹集合
 * @param sur_trajs 输出参数，更详细的周边车辆轨迹映射（ID -> 轨迹）
 * @return ErrorType
 *
 * **功能**：
 * 从 `ego_behavior` 中提取行为层输出的候选 ego 轨迹及周边车预测。
 * 这些结果会被 `SscPlanner` 用作：
 * - 走廊生成的 seed
 * - 动态障碍物渲染输入
 */
ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  *sur_trajs = map_->ego_behavior().surround_trajs;
  return kSuccess;
}

/**
 * @brief 获取自车离散行为（用于决策引导）
 * @param lat_behavior 输出参数，横向行为（如 LK, LCL, LCR）
 * @return ErrorType
 *
 * **功能**：
 * 获取行为规划层给出的当前 ego 离散横向行为，
 * 供 `SscPlanner` 在多个候选走廊和轨迹中选择最终输出。
 */
ErrorType SscPlannerAdapter::GetEgoDiscretBehavior(
    LateralBehavior* lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().lat_behavior == common::LateralBehavior::kUndefined)
    return kWrongStatus;
  *lat_behavior = map_->ego_behavior().lat_behavior;
  return kSuccess;
}

/**
 * @brief 根据车道ID获取车道信息
 * @param lane_id 目标车道ID
 * @param lane 输出参数，车道对象
 * @return ErrorType
 *
 * **功能**：
 * 从语义车道集合（Semantic Lane Set）中查找指定ID的车道。
 * 用于拓扑搜索和车道几何获取。
 */
ErrorType SscPlannerAdapter::GetLaneByLaneId(const int lane_id, Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
  }
  return kSuccess;
}

/**
 * @brief 获取自车参考车道（同 GetLocalReferenceLane）
 * @param lane 输出参数
 * @return ErrorType
 *
 * **注意**：
 * 此函数的功能与 `GetLocalReferenceLane` 似乎重复，
 * 可能是为了兼容不同的接口定义或未来的扩展。
 */
ErrorType SscPlannerAdapter::GetEgoReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

/**
 * @brief 获取静态障碍物栅格地图
 * @param grid_map 输出参数，2D 栅格地图
 * @return ErrorType
 *
 * **功能**：
 * 获取用于碰撞检测的静态障碍物表示（Grid Map）。
 * 这些数据通常来自激光雷达或占据栅格地图构建模块。
 */
ErrorType SscPlannerAdapter::GetObstacleMap(GridMap2D* grid_map) {
  if (!is_valid_) return kWrongStatus;
  *grid_map = map_->obstacle_map();
  return kSuccess;
}

/**
 * @brief 使用状态和车辆参数进行碰撞检查
 * @param vehicle_param 车辆几何参数（长、宽等）
 * @param state 车辆状态（位置、角度）
 * @param res 输出参数，true 表示发生碰撞
 * @return ErrorType
 *
 * **功能**：
 * 封装底层的碰撞检测算法。
 * 用于在轨迹优化或验证阶段快速检查某个状态是否安全。
 */
ErrorType SscPlannerAdapter::CheckIfCollision(
    const common::VehicleParam& vehicle_param, const State& state, bool* res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

/**
 * @brief 获取障碍物占据网格集合
 * @param obs_grids 输出参数，网格坐标集合
 * @return ErrorType
 *
 * **功能**：
 * 返回一组静态障碍物占据点，用于构建 Frenet 平面中的静态障碍输入。
 */
ErrorType SscPlannerAdapter::GetObstacleGrids(
    std::set<std::array<decimal_t, 2>>* obs_grids) {
  if (!is_valid_) return kWrongStatus;
  *obs_grids = map_->obstacle_grids();
  return kSuccess;
}

}  // namespace planning
