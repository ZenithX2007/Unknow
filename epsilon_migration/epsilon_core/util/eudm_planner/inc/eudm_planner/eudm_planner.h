#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_

/**
 * @file eudm_planner.h
 * @author GW
 * @brief EUDM 行为规划器接口：基于 DCP-Tree 与闭环前向仿真的高层决策器
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `EudmPlanner` 及其核心数据结构，
 * 对应论文《Efficient Uncertainty-aware Decision-making for Automated Driving using Guided Branching》
 * 在工程系统中的接口层实现。
 *
 * **核心功能**（对应文档章节）：
 * 1. **离散动作空间构造**（文档：DCP-Tree / Guided Branching）
 *    - 调用 `DcpTree` 生成有限数量的高层语义动作序列
 *    - 将原始指数级动作树压缩为“每周期至多一次动作变化”的受限脚本
 *    - 为后续闭环评估提供可枚举的策略候选集
 *
 * 2. **闭环前向仿真建模**（文档：Closed-loop Forward Simulation）
 *    - 维护 ego 与 surround agent 在前向模拟中的独立状态包
 *    - 将离散 `DcpAction` 翻译为纵向/横向语义行为与底层驾驶参数
 *    - 在场景级、层级、步级三个粒度上推进多车交互
 *
 * 3. **策略代价评估与赢家选择**（文档：Policy Evaluation）
 *    - 分解效率、安全、导航三类成本
 *    - 结合 progress cost 与 tail cost 对整条策略打分
 *    - 输出 winner 动作序列及其 ego/周车前向轨迹
 *
 * 4. **与下游运动规划层衔接**
 *    - 将 winner 对应的语义行为与参考前向轨迹交给 SSC
 *    - 为 `EudmManager` 和 `EudmPlannerServer` 提供稳定接口
 *
 * **关键设计**：
 * - **DCP-Tree**：先在离散语义动作层做引导式分支，再进入连续仿真评估
 * - **闭环仿真**：ego 与周围车均在同一预测循环中演化，而非静态重放障碍物轨迹
 * - **分层上下文**：场景级决定长期横向模式，层级绑定当前动作，步级推进车辆状态
 * - **代价可解释性**：效率/安全/导航成本拆分，便于在线调参与调试
 *
 * **主要接口**：
 * - `Init()`：读取 EUDM 配置，初始化 DCP-Tree 与仿真参数
 * - `RunOnce()`：执行一轮完整行为规划
 * - `RunEudm()`：执行核心策略枚举、仿真与评估流程
 * - `behavior()`：返回 winner 序列归纳后的语义行为
 *
 * **参考文档**：paper/EUDM.md
 */

#include <algorithm>
#include <memory>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/mobil/mobil_model.h"
#include "common/state/state.h"
#include "eudm_config.pb.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/map_interface.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/**
 * @brief EUDM 行为规划器
 *
 * **定位**：
 * 这是 EPSILON 中承上启下的一层高层行为决策器。
 * 上游接收语义地图与任务约束，下游把 winner 对应的前向轨迹交给 SSC 运动规划层。
 */
class EudmPlanner : public Planner {
 public:
  // EUDM 是 EPSILON 中的高层行为规划器。
  //
  // 结合参考论文，可以把它理解成“把原始 POMDP 决策问题工程化简化”的那一层:
  // 1. 先用 DCP 树给出有限数量的语义动作序列
  // 2. 对每条动作序列做闭环多智能体前向模拟
  // 3. 用效率/安全/导航代价评估每条策略
  // 4. 选出赢家策略，再把其前向轨迹交给下游运动规划层(SSC)
  //
  // 所以这个类的核心不是“输出控制量”，而是“输出行为决策和参考前向轨迹”。
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;
  using Cfg = planning::eudm::Config;
  using LaneChangeInfo = planning::eudm::LaneChangeInfo;

  /**
   * @brief ego 横向动作在前向仿真中的执行模式
   *
   * 论文中的横向语义动作在工程实现里不会直接变成“瞬时跳变”的车道标签，
   * 而是被映射为一个跨若干时间步持续生效的执行模式。
   */
  enum class LatSimMode {
    kAlwaysLaneKeep = 0,
    kKeepThenChange,
    kAlwaysLaneChange,
    kChangeThenCancel
  };

  /**
   * @brief ego 在闭环前向仿真中的状态包
   *
   * 与普通车辆不同，自车需要同时维护：
   * 1. 当前层正在执行的横纵向语义动作
   * 2. 整条策略序列对应的长期横向意图
   * 3. 当前/目标/长期参考车道及其坐标变换器
   * 4. 换道间隙、取消换道等策略上下文
   */
  struct ForwardSimEgoAgent {
    // 常量配置：与 ego 尺寸和横向到达判定相关。
    decimal_t lat_range;

    // 场景级配置：对整条动作序列都成立。
    OnLaneForwardSimulation::Param sim_param;

    LatSimMode seq_lat_mode;
    common::LateralBehavior lat_behavior_longterm{LateralBehavior::kUndefined};
    common::LateralBehavior seq_lat_behavior;
    bool is_cancel_behavior;
    decimal_t operation_at_seconds{0.0};

    // 层级配置：每进入一个新的 DcpAction layer 时刷新。
    common::LongitudinalBehavior lon_behavior{LongitudinalBehavior::kMaintain};
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};

    common::Lane current_lane;
    common::StateTransformer current_stf;
    common::Lane target_lane;
    common::StateTransformer target_stf;
    common::Lane longterm_lane;
    common::StateTransformer longterm_stf;

    // 可选的目标 gap 编号。layer 确定 gap 身份，step 推进后 gap 几何关系会持续变化。
    Vec2i target_gap_ids;

    // 步级状态：每个仿真时间步都会推进一次。
    common::Vehicle vehicle;
  };

  /**
   * @brief 周围车辆在闭环前向仿真中的状态包
   *
   * 周围车不维护完整策略树上下文，但需要保存：
   * - 当前车辆动力学状态
   * - 纵横向驾驶风格参数
   * - 当前参考车道及其状态变换器
   * - 横向行为概率分布
   */
  struct ForwardSimAgent {
    int id = kInvalidAgentId;
    common::Vehicle vehicle;

    // 纵向/动力学仿真参数。
    OnLaneForwardSimulation::Param sim_param;

    // 横向行为概率及当前采样/推断出的横向语义行为。
    common::ProbDistOfLatBehaviors lat_probs;
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};

    common::Lane lane;
    common::StateTransformer stf;

    // 用于判断横向动作完成或车道归属的横向阈值。
    decimal_t lat_range;
  };

  /**
   * @brief 前向仿真中的周车集合容器
   *
   * 以 `vehicle_id -> ForwardSimAgent` 的映射形式组织，
   * 便于在闭环仿真与结果回收阶段按车辆 ID 访问。
   */
  struct ForwardSimAgentSet {
    std::unordered_map<int, ForwardSimAgent> forward_sim_agents;
  };

  /**
   * @brief 效率成本分量
   *
   * 对应“希望车辆以合理速度完成任务”的代价项，
   * 同时考虑 ego 自身速度偏差以及目标车道前车对期望速度的影响。
   */
  struct EfficiencyCost {
    decimal_t ego_to_desired_vel = 0.0;
    decimal_t leading_to_desired_vel = 0.0;
    decimal_t ave() const {
      return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
    }
  };

  /**
   * @brief 安全成本分量
   *
   * 主要聚焦 RSS 风险以及占道/危险换道带来的惩罚。
   */
  struct SafetyCost {
    decimal_t rss = 0.0;
    decimal_t occu_lane = 0.0;
    decimal_t ave() const { return (rss + occu_lane) / 2.0; }
  };

  /**
   * @brief 导航成本分量
   *
   * 用于表达导航任务、换道偏好或路线选择偏置，
   * 本质上是高层任务目标对策略筛选的附加约束。
   */
  struct NavigationCost {
    decimal_t lane_change_preference = 0.0;
    decimal_t ave() const { return lane_change_preference; }
  };

  /**
   * @brief 单个动作层的代价分解
   *
   * 每条候选策略在每个 layer 上都会得到一份 `CostStructure`，
   * 最终评分由逐层 `progress_cost` 与末端 `tail_cost` 汇总得到。
   */
  struct CostStructure {
    // 与该层微动作片段关联的有效样本上界。
    int valid_sample_index_ub;
    EfficiencyCost efficiency;
    SafetyCost safety;
    NavigationCost navigation;
    decimal_t weight = 1.0;
    decimal_t ave() const {
      return (efficiency.ave() + safety.ave() + navigation.ave()) * weight;
    }

    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructure& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(efficiency: "
         << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
         << cost.efficiency.leading_to_desired_vel << "), safety: ("
         << cost.safety.rss << "," << cost.safety.occu_lane
         << "), navigation: " << cost.navigation.lane_change_preference << ")";
      return os;
    }
  };

  /**
   * @brief 返回规划器名称
   */
  std::string Name() override;

  /**
   * @brief 初始化 EUDM 规划器
   * @param config 配置文件路径
   * @return 初始化状态
   *
   * 主要完成配置读取、DCP-Tree 初始化以及仿真参数装配。
   */
  ErrorType Init(const std::string config) override;

  /**
   * @brief 执行一轮完整高层行为规划
   * @return 规划状态
   *
   * 成功后会更新：
   * - `winner_action_seq_`
   * - `winner_id_ / winner_score_`
   * - `forward_trajs_ / surround_trajs_`
   * 这些结果会继续被下游 SSC 运动规划器消费。
   */
  ErrorType RunOnce() override;

  /**
   * @brief 绑定上游语义地图接口
   */
  void set_map_interface(EudmPlannerMapItf* itf);

  /**
   * @brief 设置行为层偏好目标速度
   * @param desired_vel 用户或任务给定的期望速度
   *
   * 该值不是硬约束，而是效率成本里的参考目标。
   */
  void set_desired_velocity(const decimal_t desired_vel);

  /**
   * @brief 设置换道约束信息
   * @param lc_info 实线、禁换道、占道不安全等换道上下文
   */
  void set_lane_change_info(const LaneChangeInfo& lc_info);

  /**
   * @brief 运行核心 EUDM 策略评估流程
   * @return 运行状态
   *
   * 不包含 `RunOnce()` 前面的输入状态准备部分，
   * 主要负责策略脚本生成、前向仿真、代价评估和 winner 选择。
   */
  ErrorType RunEudm();

  /**
   * @brief 返回赢家动作序列对应的语义行为摘要
   */
  Behavior behavior() const;

  std::vector<DcpAction> winner_action_seq() const {
    return winner_action_seq_;
  }

  decimal_t desired_velocity() const;

  vec_E<vec_E<common::Vehicle>> forward_trajs() const { return forward_trajs_; }

  // `winner_id_` 表示 `action_script()` 中哪一条候选序列获胜。
  int winner_id() const;

  decimal_t time_cost() const;

  std::vector<bool> sim_res() const {
    std::vector<bool> ret;
    for (auto& r : sim_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }

  std::vector<bool> risky_res() const {
    std::vector<bool> ret;
    for (auto& r : risky_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }
  std::vector<std::string> sim_info() const { return sim_info_; }
  std::vector<decimal_t> final_cost() const { return final_cost_; }
  std::vector<std::vector<CostStructure>> progress_cost() const {
    return progress_cost_;
  }
  std::vector<CostStructure> tail_cost() const { return tail_cost_; }
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs()
      const {
    return surround_trajs_;
  }
  common::State plan_state() { return ego_vehicle_.state(); }
  std::vector<std::vector<DcpAction>> action_script() {
    return dcp_tree_ptr_->action_script();
  }

  const Cfg& cfg() const { return cfg_; }

  EudmPlannerMapItf* map_itf() const;

  void UpdateDcpTree(const DcpAction& ongoing_action);

  /**
   * @brief 归纳动作序列的横向语义摘要
   *
   * 将离散 `DcpAction` 序列抽象成：
   * 1. 主要横向意图
   * 2. 横向操作的触发时间
   * 3. 是否属于“先换道再取消”的特殊模式
   */
  ErrorType ClassifyActionSeq(const std::vector<DcpAction>& action_seq,
                              decimal_t* operation_at_seconds,
                              common::LateralBehavior* lat_behavior,
                              bool* is_cancel_operation) const;

 private:
  // 读取 protobuf 配置。
  ErrorType ReadConfig(const std::string config_path);

  // 把论文中的“语义动作参数化驾驶风格”翻译成前向模拟器参数。
  // 这里会生成 IDM / 横向 pure pursuit / 车辆动力学限制等设置。
  ErrorType GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                        OnLaneForwardSimulation::Param* sim_param);

  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids) const;
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  ErrorType TranslateDcpActionToLonLatBehavior(const DcpAction& action,
                                               LateralBehavior* lat,
                                               LongitudinalBehavior* lon) const;

  ErrorType GetSurroundingForwardSimAgents(
      const common::SemanticVehicleSet& surrounding_semantic_vehicles,
      ForwardSimAgentSet* forward_sim_agents) const;

  // ==================== 闭环仿真主流程 ====================
  // 对一整条动作序列做闭环多智能体前向模拟。
  ErrorType SimulateActionSequence(
      const common::Vehicle& ego_vehicle,
      const ForwardSimAgentSet& surrounding_fsagents,
      const std::vector<DcpAction>& action_seq, const int& seq_id);

  ErrorType SimulateScenario(
      const common::Vehicle& ego_vehicle,
      const ForwardSimAgentSet& surrounding_fsagents,
      const std::vector<DcpAction>& action_seq, const int& seq_id,
      const int& sub_seq_id, std::vector<int>* sub_sim_res,
      std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
      std::vector<std::vector<CostStructure>>* sub_progress_cost,
      std::vector<CostStructure>* sub_tail_cost,
      vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
      std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
      std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
          sub_surround_trajs);

  // 对单个语义动作(action layer)做细粒度时间步仿真。
  ErrorType SimulateSingleAction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
      const ForwardSimAgentSet& surrounding_fsagents_this_layer,
      vec_E<common::Vehicle>* ego_traj_multisteps,
      std::unordered_map<int, vec_E<common::Vehicle>>*
          surround_trajs_multisteps);

  // ==================== 策略评估函数 ====================
  // 单层动作代价函数，综合效率/安全/导航。
  ErrorType CostFunction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
      const ForwardSimAgentSet& other_fsagent,
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool verbose, CostStructure* cost, bool* is_risky,
      std::set<int>* risky_ids);

  ErrorType StrictSafetyCheck(
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool* is_safe, int* collided_id);

  ErrorType EvaluateSafetyStatus(const vec_E<common::Vehicle>& traj_a,
                                 const vec_E<common::Vehicle>& traj_b,
                                 decimal_t* cost, bool* is_rss_safe,
                                 int* risky_id);
  ErrorType EvaluateSinglePolicyTrajs(
      const std::vector<CostStructure>& progress_cost,
      const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
      decimal_t* score);

  // 汇总多线程模拟结果，挑出赢家策略。
  ErrorType EvaluateMultiThreadSimResults(int* winner_id,
                                          decimal_t* winner_cost);

  // ==================== 仿真辅助函数 ====================
  // 场景级初始化: 根据整条动作序列确定长期横向模式和纵向风格。
  ErrorType UpdateSimSetupForScenario(const std::vector<DcpAction>& action_seq,
                                      ForwardSimEgoAgent* ego_fsagent) const;

  // 层级初始化: 根据当前层 action 刷新 ego 的当前/目标/长期车道与 gap 信息。
  ErrorType UpdateSimSetupForLayer(const DcpAction& action,
                                   const ForwardSimAgentSet& other_fsagent,
                                   ForwardSimEgoAgent* ego_fsagent) const;

  // 把 DcpAction 转成底层可执行的横纵向语义行为。
  ErrorType UpdateEgoBehaviorsUsingAction(
      const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const;

  bool CheckIfLateralActionFinished(const common::State& cur_state,
                                    const int& action_ref_lane_id,
                                    const LateralBehavior& lat_behavior,
                                    int* current_lane_id) const;

  ErrorType UpdateLateralActionSequence(
      const int cur_idx, std::vector<DcpAction>* action_seq) const;

  ErrorType PrepareMultiThreadContainers(const int n_sequence);

  ErrorType GetSimTimeSteps(const DcpAction& action,
                            std::vector<decimal_t>* dt_steps) const;

  ErrorType EgoAgentForwardSim(const ForwardSimEgoAgent& ego_fsagent,
                               const common::VehicleSet& all_sim_vehicles,
                               const decimal_t& sim_time_step,
                               common::State* state_out) const;

  ErrorType SurroundingAgentForwardSim(
      const ForwardSimAgent& fsagent,
      const common::VehicleSet& all_sim_vehicles,
      const decimal_t& sim_time_step, common::State* state_out) const;

  // ==================== 系统依赖 ====================
  // 上游语义地图接口。
  EudmPlannerMapItf* map_itf_{nullptr};
  // DCP 树: 论文中用于 guided action branching 的离散动作树。
  DcpTree* dcp_tree_ptr_;
  // ==================== 配置与场景设置 ====================
  Cfg cfg_;
  LaneChangeInfo lc_info_;
  decimal_t desired_velocity_{5.0};
  decimal_t sim_time_total_ = 0.0;
  // pre_deleted_seq_ids_ 用于预删除明显不需要评估的动作序列。
  std::set<int> pre_deleted_seq_ids_;
  int ego_lane_id_{kInvalidLaneId};
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  common::Lane rss_lane_;
  common::StateTransformer rss_stf_;
  common::RssChecker::RssConfig rss_config_;
  common::RssChecker::RssConfig rss_config_strict_as_front_;
  common::RssChecker::RssConfig rss_config_strict_as_rear_;

  // ego 与周围车可使用不同的 forward simulation 参数。
  OnLaneForwardSimulation::Param ego_sim_param_;
  OnLaneForwardSimulation::Param agent_sim_param_;

  // ==================== 当前输入状态 ====================
  decimal_t time_stamp_;
  int ego_id_;
  common::Vehicle ego_vehicle_;

  // ==================== 规划结果缓存 ====================
  // 赢家策略索引与得分。
  int winner_id_ = 0;
  decimal_t winner_score_ = 0.0;
  // 赢家的离散动作序列。
  std::vector<DcpAction> winner_action_seq_;
  // 每条动作序列是否仿真成功、是否存在风险。
  std::vector<int> sim_res_;
  std::vector<int> risky_res_;
  // 字符串形式的调试信息。
  std::vector<std::string> sim_info_;
  // 每条策略最终评分。
  std::vector<decimal_t> final_cost_;
  // 每条策略在每个 layer 上的累计代价明细。
  std::vector<std::vector<CostStructure>> progress_cost_;
  std::vector<CostStructure> tail_cost_;
  // 赢家及所有候选的 ego 前向轨迹，这些会被下游运动规划层使用。
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_;
  // 每条策略对应的周围车预测轨迹。
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
  // 本轮 EUDM 总耗时。
  decimal_t time_cost_ = 0.0;
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
