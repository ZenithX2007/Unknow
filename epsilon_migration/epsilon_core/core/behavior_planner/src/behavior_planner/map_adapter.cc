/**
 * @file map_adapter.cc
 * @author GW
 * @brief BehaviorPlanner 地图适配器实现：语义地图到行为规划查询接口的桥接层
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件实现 `BehaviorPlannerMapAdapter`，
 * 负责把 `SemanticMapManager` 的地图、车道与车辆语义查询接口
 * 翻译为 `BehaviorPlanner` 可直接使用的统一访问函数。
 */
#include "behavior_planner/map_adapter.h"

namespace planning {

// 检查当前适配器是否已经绑定了有效语义地图。
bool BehaviorPlannerMapAdapter::IsValid() { return is_valid_; }

// 读取 ego 当前状态。
ErrorType BehaviorPlannerMapAdapter::GetEgoState(State *state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

// 读取 ego id。
ErrorType BehaviorPlannerMapAdapter::GetEgoId(int *id) {
  if (!is_valid_) return kWrongStatus;
  *id = map_->ego_id();
  return kSuccess;
}

// 读取 ego 车辆完整状态。
ErrorType BehaviorPlannerMapAdapter::GetEgoVehicle(common::Vehicle *vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

// 根据当前位置和导航路径判断 ego 当前所属车道。
ErrorType BehaviorPlannerMapAdapter::GetEgoLaneIdByPosition(
    const std::vector<int> &navi_path, int *lane_id) {
  if (!is_valid_) {
    printf("[GetEgoLaneIdByPosition]Interface not valid.\n");
    return kWrongStatus;
  }

  int ego_lane_id = kInvalidLaneId;
  decimal_t distance_to_lane;
  decimal_t arc_len;

  Vec3f state_3dof(map_->ego_vehicle().state().vec_position(0),
                   map_->ego_vehicle().state().vec_position(1),
                   map_->ego_vehicle().state().angle);
  std::set<std::tuple<decimal_t, decimal_t, int>> dist_set;

  if (map_->GetNearestLaneIdUsingState(state_3dof, navi_path, &ego_lane_id,
                                       &distance_to_lane,
                                       &arc_len) != kSuccess) {
    printf("[GetEgoLaneIdByPosition]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  *lane_id = ego_lane_id;
  return kSuccess;
}

// 在导航路径约束下查询最近车道。
ErrorType BehaviorPlannerMapAdapter::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  std::set<std::tuple<decimal_t, decimal_t, int>> dist_set;
  if (map_->GetNearestLaneIdUsingState(state, navi_path, id, distance,
                                       arc_len) != kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// 判断给定车道在拓扑上是否可达。
ErrorType BehaviorPlannerMapAdapter::IsTopologicallyReachable(
    const int lane_id, const std::vector<int> &path, int *num_lane_changes,
    bool *res) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  if (map_->IsTopologicallyReachable(lane_id, path, num_lane_changes, res) !=
      kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// 查询右侧可换入车道。
ErrorType BehaviorPlannerMapAdapter::GetRightLaneId(const int lane_id,
                                                    int *r_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.r_change_avbl) {
      *r_lane_id = it->second.r_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

// 查询左侧可换入车道。
ErrorType BehaviorPlannerMapAdapter::GetLeftLaneId(const int lane_id,
                                                   int *l_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.l_change_avbl) {
      *l_lane_id = it->second.l_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

// 根据 lane_id 返回对应 Lane 几何对象。
ErrorType BehaviorPlannerMapAdapter::GetLaneByLaneId(const int lane_id,
                                                     Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
    if (!lane->IsValid()) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetChildLaneIds(
    const int lane_id, std::vector<int> *child_ids) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // ~ note this is an assign
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetFatherLaneIds(
    const int lane_id, std::vector<int> *father_ids) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    father_ids->assign(it->second.father_id.begin(),
                       it->second.father_id.end());
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetLocalLaneSamplesByState(
    const State &state, const int lane_id, const std::vector<int> &navi_path,
    const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
    vec_Vecf<2> *samples) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLocalLaneSamplesByState(state, lane_id, navi_path,
                                       max_reflane_dist, max_backward_dist,
                                       samples) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetRefLaneForStateByBehavior(
    const State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality, Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetRefLaneForStateByBehavior(state, navi_path, behavior,
                                         max_forward_len, max_back_len,
                                         is_high_quality, lane) != kSuccess) {
    return kWrongStatus;
  }
  if (!lane->IsValid()) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLeadingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                                    leading_vehicle,
                                    distance_residual_ratio) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetKeyVehicles(
    common::VehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  // TODO: (@denny.ding) add vehicle selection strategy here
  *key_vehicle_set = map_->surrounding_vehicles();
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetKeySemanticVehicles(
    common::SemanticVehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  // TODO: (@denny.ding) add vehicle selection strategy here
  *key_vehicle_set = map_->semantic_key_vehicles();
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetWholeLaneNet(
    common::LaneNet *lane_net) {
  if (!is_valid_) return kWrongStatus;
  *lane_net = map_->whole_lane_net();
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res) {
  if (!is_valid_) return kWrongStatus;
  if (map_->CheckCollisionUsingState(param_a, state_a, param_b, state_b, res) !=
      kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::CheckIfCollision(
    const common::VehicleParam &vehicle_param, const State &state, bool *res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetSpeedLimit(const State &state,
                                                   const Lane &lane,
                                                   decimal_t *speed_limit) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetSpeedLimit(state, lane, speed_limit) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetPredictedBehavior(
    const int vehicle_id, common::LateralBehavior *lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  if (vehicle_id == kInvalidAgentId) return kWrongStatus;
  auto semantic_vehicle_set = map_->semantic_surrounding_vehicles();
  *lat_behavior =
      semantic_vehicle_set.semantic_vehicles.at(vehicle_id).lat_behavior;
  return kSuccess;
}

ErrorType BehaviorPlannerMapAdapter::GetFixedNavigationPath(
    std::vector<int> *navi_path) {
  if (!is_valid_) return kWrongStatus;
  *navi_path = map_->fixed_navi_path();
  return kSuccess;
}

void BehaviorPlannerMapAdapter::set_map(
    std::shared_ptr<IntegratedMap> map_ptr) {
  map_ = map_ptr;
  is_valid_ = true;
}

}  // namespace planning
