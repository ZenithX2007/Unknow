#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_

/**
 * @file behavior_planner.h
 * @author GW
 * @brief MPDM/规则型行为规划器接口：基于多策略前向仿真的高层行为决策层
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `BehaviorPlanner`，
 * 它是 EPSILON 中较早的一代高层行为规划器接口，
 * 主要基于 MPDM（Multi-Policy Decision Making）/规则逻辑进行行为决策。
 *
 * **核心功能**：
 * 1. 读取地图接口、任务输入与自车状态
 * 2. 判断可用横向行为集合（保持/左换道/右换道）
 * 3. 对候选行为执行前向仿真
 * 4. 评估多条策略并选出 winner 行为
 * 5. 构造参考车道供下游运动规划或执行层使用
 *
 * **系统定位**：
 * `SemanticMapManager -> BehaviorPlanner -> SSC / control`
 */

#include <memory>
#include <string>

#include "behavior_planner/map_interface.h"
#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "route_planner/route_planner.h"

#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
namespace planning {

/**
 * @brief MPDM/规则型行为规划器
 *
 * 相比 EUDM，这个类更偏向“有限行为集合 + 前向仿真筛选”的工程实现，
 * 不显式维护 DCP-Tree，而是围绕 lane keep / lane change 等行为做候选评估。
 */
class BehaviorPlanner : public Planner {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  // 返回规划器名称。
  std::string Name() override;

  // 初始化 route planner 和行为缓存。
  ErrorType Init(const std::string config) override;

  // 执行一轮完整行为规划。
  ErrorType RunOnce() override;

  // 挂接上游地图接口。
  void set_map_interface(BehaviorPlannerMapItf* itf);

  /**
   * @brief 设置用户期望速度
   * @param desired_vel 用户给定的目标速度
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  /**
   * @brief 设置 L2 级人工换道指令
   * @param hmi_behavior HMI/joystick 指定的横向行为
   **/
  void set_hmi_behavior(const LateralBehavior& hmi_behavior);

  /**
   * @brief 设置自动驾驶等级
   */
  void set_autonomous_level(int level);

  void set_sim_resolution(const decimal_t sim_resolution);

  void set_sim_horizon(const decimal_t sim_horizon);

  void set_use_sim_state(bool use_sim_state);

  void set_aggressive_level(int level);

  // 运行 route planner，更新导航路径与可达车道信息。
  ErrorType RunRoutePlanner(const int nearest_lane_id);

  // 运行 MPDM 核心前向仿真与 winner 选择：
  // 枚举有限横向行为，对每个行为进行 rollout，再依据代价选出 winner。
  ErrorType RunMpdm();

  // 返回本轮 winner 行为结果。
  Behavior behavior() const;

  decimal_t user_desired_velocity() const;

  decimal_t reference_desired_velocity() const;

  int autonomous_level() const;

  vec_E<vec_E<common::Vehicle>> forward_trajs() const;

  std::vector<LateralBehavior> forward_behaviors() const;

 protected:
  ErrorType ConstructReferenceLane(const LateralBehavior& lat_behavior,
                                   Lane* lane);

  ErrorType ConstructLaneFromSamples(const vec_E<Vecf<2>>& samples, Lane* lane);

  ErrorType MultiBehaviorJudge(const decimal_t previous_desired_vel,
                               LateralBehavior* mpdm_behavior,
                               decimal_t* actual_desired_velocity);

  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids);
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  ErrorType MultiAgentSimForward(
      const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  // 周围车辆按当前语义行为推进，但不根据 ego 的新动作重新响应。
  ErrorType OpenloopSimForward(
      const common::SemanticVehicle& ego_semantic_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  // 对一个候选 ego 横向行为执行一次多车前向仿真。
  ErrorType SimulateEgoBehavior(
      const common::Vehicle& ego_vehicle, const LateralBehavior& ego_behavior,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  // 汇总所有候选轨迹的代价并选择最优行为。
  ErrorType EvaluateMultiPolicyTrajs(
      const std::vector<LateralBehavior>& valid_behaviors,
      const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
      const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>&
          valid_surround_trajs,
      LateralBehavior* winner_behavior,
      vec_E<common::Vehicle>* winner_forward_traj, decimal_t* winner_score,
      decimal_t* desired_vel);

  // 计算单条候选策略的综合分数和推荐速度。
  ErrorType EvaluateSinglePolicyTraj(
      const LateralBehavior& behaivor,
      const vec_E<common::Vehicle>& forward_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
      decimal_t* score, decimal_t* desired_vel);

  // 用车辆包络和时序轨迹评估候选策略的碰撞风险。
  ErrorType EvaluateSafetyCost(const vec_E<common::Vehicle>& traj_a,
                               const vec_E<common::Vehicle>& traj_b,
                               decimal_t* cost);

  ErrorType GetDesiredVelocityOfTrajectory(
      const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel);

  BehaviorPlannerMapItf* map_itf_{nullptr};
  Behavior behavior_;

  planning::RoutePlanner* p_route_planner_{nullptr};

  decimal_t user_desired_velocity_{5.0};
  decimal_t reference_desired_velocity_{5.0};
  int autonomous_level_{2};

  decimal_t sim_resolution_{0.4};
  decimal_t sim_horizon_{4.0};
  int aggressive_level_{3};
  planning::OnLaneForwardSimulation::Param sim_param_;

  bool use_sim_state_ = true;
  bool lock_to_hmi_ = false;
  LateralBehavior hmi_behavior_ = LateralBehavior::kLaneKeeping;

  // track the ego lane id
  int ego_lane_id_{kInvalidLaneId};
  int ego_id_;
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  // debug
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
};

}  // namespace planning

#endif
