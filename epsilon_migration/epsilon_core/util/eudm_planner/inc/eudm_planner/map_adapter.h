#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_

/**
 * @file map_adapter.h
 * @author GW
 * @brief EUDM 地图适配器接口：把 SemanticMapManager 翻译为 EUDM 统一地图接口
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `EudmPlannerMapAdapter`，
 * 负责将 `SemanticMapManager` 中的环境信息转换成 `EudmPlannerMapItf`
 * 所定义的统一查询接口，供 `EudmPlanner` 与 `EudmManager` 使用。
 *
 * **适配内容**：
 * 1. ego / 周围车状态读取
 * 2. 车道拓扑与参考线构造
 * 3. 前后车 Frenet 关系查询
 * 4. 拓扑可达性与车道一致性判断
 * 5. 碰撞检测与全局车道网访问
 */

#include <iostream>
#include <set>

#include "common/basics/semantics.h"
#include "eudm_planner/map_interface.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace planning {

/**
 * @brief EUDM 地图适配器
 *
 * **功能**：
 * 将 `SemanticMapManager` 暴露的环境能力适配成 `EudmPlannerMapItf`
 * 所要求的统一接口，供 `EudmPlanner` 与 `EudmManager` 使用。
 *
 * **适配内容**：
 * - ego / 周边车状态查询
 * - 车道拓扑与参考线生成
 * - 前后车搜索与 Frenet 查询
 * - 碰撞检测
 *
 * **设计意义**：
 * - 让 planner 依赖抽象接口而不是具体地图实现
 * - 方便替换地图后端或在不同系统中复用 EUDM 逻辑
 */
class EudmPlannerMapAdapter : public EudmPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;
  // adapter 只做一件事:
  // 把 SemanticMapManager 暴露的环境能力翻译成 EUDM 需要的统一接口。
  bool IsValid() override;
  ErrorType GetEgoState(State *state) override;
  ErrorType GetEgoId(int *id) override;
  ErrorType GetEgoVehicle(common::Vehicle *vehicle) override;
  ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                   int *lane_id) override;
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) override;
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) override;
  bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) override;
  ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) override;
  ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) override;
  ErrorType GetChildLaneIds(const int lane_id,
                            std::vector<int> *child_ids) override;
  ErrorType GetFatherLaneIds(const int lane_id,
                             std::vector<int> *father_ids) override;
  ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) override;
  ErrorType GetLocalLaneSamplesByState(const State &state, const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) override;
  ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality, Lane *lane);
  ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) override;
  ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) override;
  ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) override;
  ErrorType GetWholeLaneNet(common::LaneNet *lane_net) override;
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b,
                                     bool *res) override;
  ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio) override;
  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) override;

  void set_map(std::shared_ptr<IntegratedMap> map_ptr);

  std::shared_ptr<IntegratedMap> map() { return map_; }

 private:
  // 当前规划周期绑定的语义地图快照。
  std::shared_ptr<IntegratedMap> map_;
  bool is_valid_ = false;
};

}  // namespace planning

#endif
