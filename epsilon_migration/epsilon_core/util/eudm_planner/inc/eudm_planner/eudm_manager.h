#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_

/**
 * @file eudm_manager.h
 * @author GW
 * @brief EUDM 在线管理器接口：封装跨周期上下文、任务状态机与 planner 调度
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 */

#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
namespace planning {

/**
 * @brief EUDM 在线管理器
 *
 * **设计动机**：
 * `EudmPlanner` 近似是“单周期、弱状态”的决策器，
 * 但真实在线系统需要维护跨周期上下文、任务约束、HMI 信号与结果快照。
 * 因此引入 `EudmManager` 作为工程调度层。
 *
 * **主要职责**：
 * 1. 维护 ongoing action 的重规划上下文
 * 2. 管理任务级 lane change state machine
 * 3. 对 planner 原始 winner 做上下文约束下的重选
 * 4. 保存与导出规划快照
 * 5. 生成下游使用的 `SemanticBehavior`
 */
class EudmManager {
 public:
  // manager 是 EUDM 的在线调度层。
  // planner 本体尽量保持“单轮规划、少状态”；manager 则负责维护跨周期上下文、
  // HMI/任务输入、重选逻辑以及输出快照。
  using DcpLatAction = planning::DcpTree::DcpLatAction;
  using DcpLonAction = planning::DcpTree::DcpLonAction;
  using DcpAction = planning::DcpTree::DcpAction;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using CostStructure = planning::EudmPlanner::CostStructure;

  /**
   * @brief 换道触发来源
   * - `kStick`：来自用户/HMI stick 显式触发
   * - `kActive`：来自系统主动换道提案
   */
  enum class LaneChangeTriggerType { kStick = 0, kActive };

  struct ReplanningContext {
    // 用于在相邻规划周期之间延续已开始执行的动作语义。
    bool is_valid = false;
    decimal_t seq_start_time;
    std::vector<DcpAction> action_seq;
  };

  struct ActivateLaneChangeRequest {
    // 主动换道候选在多帧上的累计观测。
    decimal_t trigger_time;
    decimal_t desired_operation_time;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeProposal {
    // 经多帧一致性验证后生成的一次性主动换道建议。
    bool valid = false;
    decimal_t trigger_time = 0.0;
    decimal_t operation_at_seconds = 0.0;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeContext {
    // 任务级换道状态机。
    bool completed = true;
    bool trigger_when_appropriate = false;
    decimal_t trigger_time = 0.0;
    decimal_t desired_operation_time = 0.0;
    int ego_lane_id = 0;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
    LaneChangeTriggerType type;
  };

  struct Snapshot {
    // 一次完整规划周期的结果快照，供后处理、可视化和下一轮上下文使用。
    bool valid = false;
    int original_winner_id;
    int processed_winner_id;
    common::State plan_state;
    std::vector<std::vector<DcpAction>> action_script;
    std::vector<bool> sim_res;
    std::vector<bool> risky_res;
    std::vector<std::string> sim_info;
    std::vector<decimal_t> final_cost;
    std::vector<std::vector<CostStructure>> progress_cost;
    std::vector<CostStructure> tail_cost;
    vec_E<vec_E<common::Vehicle>> forward_trajs;
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs;
    common::Lane ref_lane;

    double plan_stamp = 0.0;
    double time_cost = 0.0;
  };

  EudmManager() {}

  // 初始化 manager 及内部 planner。
  void Init(const std::string& config_path, const decimal_t work_rate);

  // 任务级运行入口。
  ErrorType Run(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  // 清空上下文。
  void Reset();

  // 把最近一次规划结果打包成下游使用的 SemanticBehavior。
  void ConstructBehavior(common::SemanticBehavior* behavior);

  EudmPlanner& planner();

  int original_winner_id() const { return last_snapshot_.original_winner_id; }
  int processed_winner_id() const { return last_snapshot_.processed_winner_id; }
  std::shared_ptr<semantic_map_manager::SemanticMapManager> map() {
    return map_adapter_.map();
  }

 private:
  // 求最近的未来动作层边界时刻。
  decimal_t GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                          const decimal_t& delta);

  // 判断当前是不是一个合适的换道触发时机。
  bool IsTriggerAppropriate(const LateralBehavior& lat);

  // 在调用 planner 前准备 map、ongoing action、参考速度与换道约束。
  ErrorType Prepare(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  // 结合用户期望速度与道路曲率，生成更现实的参考速度。
  ErrorType EvaluateReferenceVelocity(const planning::eudm::Task& task,
                                      decimal_t* ref_vel);

  // 从历史上下文恢复当前时刻仍应执行的动作。
  bool GetReplanDesiredAction(const decimal_t current_time,
                              DcpAction* desired_action);

  // 保存当前 planner 的完整快照。
  void SaveSnapshot(Snapshot* snapshot);

  // 结合任务上下文对 planner 原始 winner 做二次筛选。
  ErrorType ReselectByContext(const decimal_t stamp, const Snapshot& snapshot,
                              int* new_seq_id);

  // 依据 task/HMI 信号推进换道状态机。
  void UpdateLaneChangeContextByTask(const decimal_t stamp,
                                     const planning::eudm::Task& task);

  // 从最近若干轮规划结果中生成主动换道提案。
  ErrorType GenerateLaneChangeProposal(const decimal_t& stamp,
                                       const planning::eudm::Task& task);

  // 底层行为规划器本体。
  EudmPlanner bp_;
  // 语义地图适配器。
  EudmPlannerMapAdapter map_adapter_;
  decimal_t work_rate_{20.0};

  // ego 当前车道 id。
  int ego_lane_id_;
  // 重规划上下文。
  ReplanningContext context_;
  // 最近一次规划快照。
  Snapshot last_snapshot_;
  // 最近一次任务输入。
  planning::eudm::Task last_task_;
  // 当前换道任务状态机。
  LaneChangeContext lc_context_;
  // 最近一次主动换道提案。
  LaneChangeProposal last_lc_proposal_;
  // 主动换道提案的累计缓冲。
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_;
};

}  // namespace planning

#endif
