/**
 * @file map_interface.h
 * @author GW
 * @brief SSC 地图抽象接口：定义运动规划器所需的环境查询能力
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `SscPlannerMapItf` 抽象接口，
 * 约束 SSC 运动规划器对上游环境和行为层结果的最小依赖集合。
 *
 * **接口能力分层**：
 * 1. ego 状态与参考车道读取
 * 2. 局部/指定车道查询
 * 3. 静态障碍地图与障碍点读取
 * 4. 候选前向轨迹与周车预测读取
 * 5. 离散行为标签与碰撞检测查询
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__

#include <array>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/**
 * @brief SSC 地图接口抽象基类
 *
 * **设计目标**：
 * 让 `SscPlanner` 依赖统一查询接口，而不直接绑定具体语义地图实现。
 *
 * **接口能力分层**：
 * 1. ego 状态与参考车道读取
 * 2. 局部车道/指定车道查询
 * 3. 静态障碍地图与障碍点读取
 * 4. 上游行为层候选前向轨迹读取
 * 5. 当前离散行为标签读取
 *
 * **工程意义**：
 * - 便于复用 `SscPlanner` 到不同系统
 * - 便于通过 adapter/mocks 进行联调与测试
 */
class SscPlannerMapItf {
 public:
  using ObstacleMapType = uint8_t;
  using State = common::State;
  using Lane = common::Lane;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using Behavior = common::SemanticBehavior;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  // 当前输入是否有效；无效时 SSC 不应继续使用旧地图快照。
  virtual bool IsValid() = 0;
  // 当前规划周期时间戳。
  virtual decimal_t GetTimeStamp() = 0;
  // ego 车辆完整信息。
  virtual ErrorType GetEgoVehicle(Vehicle* vehicle) = 0;
  // ego 当前状态。
  virtual ErrorType GetEgoState(State* state) = 0;
  // ego 对应的全局参考车道。
  virtual ErrorType GetEgoReferenceLane(Lane* lane) = 0;
  // 用于 Frenet 构建的局部参考车道。
  virtual ErrorType GetLocalReferenceLane(Lane* lane) = 0;
  // 通过 lane id 查询指定车道。
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) = 0;
  // 获取静态障碍二维占据图。
  virtual ErrorType GetObstacleMap(GridMap2D* grid_map) = 0;
  // 检查给定状态是否碰撞。
  virtual ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                                     const State& state, bool* res) = 0;
  // 获取 ego 候选前向轨迹，作为 SSC 走廊生成的 seed。
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) = 0;
  // 获取 ego 候选前向轨迹及周边车预测，后者用于时空动态障碍物。
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<Vehicle>>>* sur_trajs) = 0;
  // 获取当前 ego 离散横向行为，用于从多条优化轨迹中选择执行结果。
  virtual ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) = 0;
  // 获取静态障碍点集，后续会沿时间轴复制到 SSC 占据图。
  virtual ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) = 0;
};

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__
