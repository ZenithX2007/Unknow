#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_INTERFACE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_INTERFACE_H_

/**
 * @file map_interface.h
 * @author GW
 * @brief EUDM 地图抽象接口：定义 planner 所需的环境查询能力
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `EudmPlannerMapItf` 抽象接口，
 * 统一描述 EUDM 行为规划层对地图、车道、周车和碰撞检测能力的依赖。
 *
 * **接口能力**：
 * 1. ego 状态与车道定位
 * 2. 车道拓扑与导航可达性判断
 * 3. 关键车/周围车/语义车提取
 * 4. 参考线构建与 Frenet 邻车查询
 * 5. 碰撞检测与前后车关系分析
 */

#include <iostream>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/**
 * @brief EUDM 地图接口抽象基类
 *
 * **设计目标**：
 * 让 `EudmPlanner` 依赖“查询能力”而不是具体地图实现。
 *
 * **接口分层**：
 * 1. ego 基础状态查询
 * 2. 车道定位与拓扑查询
 * 3. 周边车/语义车集合查询
 * 4. 参考线生成与 Frenet 关系查询
 * 5. 碰撞检测与前后车关系查询
 *
 * **工程意义**：
 * - 方便将 planner 接到不同地图后端
 * - 便于单元测试时注入 mock map
 */
class EudmPlannerMapItf {
 public:
  // EUDM 只依赖这一层抽象接口，而不直接依赖具体地图实现。
  // 因而 planner 可以专注于“需要哪些查询能力”，由 adapter 负责桥接。
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;

  // ego 基础信息读取：planner 的每轮仿真初始状态。
  virtual bool IsValid() = 0;
  virtual ErrorType GetEgoState(State *state) = 0;
  virtual ErrorType GetEgoId(int *id) = 0;
  virtual ErrorType GetEgoVehicle(common::Vehicle *vehicle) = 0;
  // 车道定位与拓扑查询：用于判断动作是否满足导航可达性。
  virtual ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                           int *lane_id) = 0;
  virtual ErrorType GetNearestLaneIdUsingState(
      const Vec3f &state, const std::vector<int> &navi_path, int *id,
      decimal_t *distance, decimal_t *arc_len) = 0;
  virtual ErrorType IsTopologicallyReachable(const int lane_id,
                                             const std::vector<int> &path,
                                             int *num_lane_changes,
                                             bool *res) = 0;
  virtual bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) = 0;
  // 周围车与语义车集合查询：EUDM 只使用关键交互车辆降低搜索规模。
  virtual ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) = 0;
  virtual ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) = 0;
  virtual ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) = 0;
  // 车道拓扑及参考线查询：把离散横向行为翻译为可仿真的 Lane。
  virtual ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) = 0;
  virtual ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) = 0;
  virtual ErrorType GetChildLaneIds(const int lane_id,
                                    std::vector<int> *child_ids) = 0;
  virtual ErrorType GetFatherLaneIds(const int lane_id,
                                     std::vector<int> *father_ids) = 0;
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) = 0;
  virtual ErrorType GetLocalLaneSamplesByState(
      const State &state, const int lane_id, const std::vector<int> &navi_path,
      const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
      vec_Vecf<2> *samples) = 0;
  virtual ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality,
      Lane *lane) = 0;
  // 全局道路网络、碰撞检测与跟驰关系查询：支撑 IDM、RSS 和换道 gap 评估。
  virtual ErrorType GetWholeLaneNet(common::LaneNet *lane_net) = 0;
  virtual ErrorType CheckCollisionUsingState(
      const common::VehicleParam &param_a, const common::State &state_a,
      const common::VehicleParam &param_b, const common::State &state_b,
      bool *res) = 0;
  virtual ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) = 0;
  virtual ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) = 0;
};

}  // namespace planning

#endif
