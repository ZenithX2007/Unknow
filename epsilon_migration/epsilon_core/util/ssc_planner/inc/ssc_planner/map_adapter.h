/**
 * @file map_adapter.h
 * @author GW
 * @brief SSC 地图适配器接口：把 SemanticMapManager 翻译为 SscPlannerMapItf
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `SscPlannerAdapter`，
 * 用于把 `SemanticMapManager` 暴露的环境快照、行为层输出与静态障碍信息
 * 适配成 `SscPlannerMapItf` 所要求的统一运动规划输入接口。
 *
 * **适配内容**：
 * 1. ego 状态与参考车道读取
 * 2. 障碍地图与障碍点集提取
 * 3. 上游行为层候选前向轨迹获取
 * 4. 当前离散横向行为读取
 * 5. 基础碰撞检测能力
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__

#include "common/basics/semantics.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "ssc_planner/map_interface.h"

namespace planning {

/**
 * @brief SSC 地图适配器
 *
 * **功能**：
 * 将 `SemanticMapManager` 暴露的环境快照和行为结果，
 * 转换成 `SscPlanner` 所依赖的统一地图接口。
 *
 * **适配内容**：
 * - ego 状态与参考车道
 * - 静态障碍地图与障碍点
 * - 上游行为层前向轨迹
 * - 当前离散行为标签
 * - 基础碰撞检测
 *
 * **设计意义**：
 * - 让 `SscPlanner` 不直接依赖具体语义地图实现
 * - 便于替换地图后端或单独测试 planner
 */
class SscPlannerAdapter : public SscPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;

  // 当前适配器是否已绑定有效地图。
  bool IsValid() override;
  // 当前语义地图时间戳。
  decimal_t GetTimeStamp() override;
  // ego 完整车辆对象。
  ErrorType GetEgoVehicle(Vehicle* vehicle) override;
  // ego 当前状态。
  ErrorType GetEgoState(State* state) override;
  // ego 参考车道。
  ErrorType GetEgoReferenceLane(Lane* lane) override;
  // 局部参考车道。
  ErrorType GetLocalReferenceLane(Lane* lane) override;
  // 通过 lane id 获取车道。
  ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) override;
  // 获取静态障碍二维占据图。
  ErrorType GetObstacleMap(GridMap2D* grid_map) override;
  // 基于状态执行碰撞检查。
  ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                             const State& state, bool* res) override;
  // 获取 ego 候选前向轨迹。
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) override;
  // 获取当前 ego 离散横向行为。
  ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) override;
  // 获取 ego 候选前向轨迹及周边车预测。
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs)
      override;
  // 获取静态障碍点集。
  ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) override;

  // 绑定当前规划周期的语义地图快照。
  ErrorType set_map(std::shared_ptr<IntegratedMap> map);

 private:
  // 当前绑定的语义地图快照。
  std::shared_ptr<IntegratedMap> map_;
  bool is_valid_ = false;
};

}  // namespace planning

#endif
