#include "eudm_planner/eudm_manager.h"

#include <cstdlib>
#include <sys/stat.h>

#include <glog/logging.h>

namespace planning {

/**
 * @file eudm_manager.cc
 * @author GW
 * @brief EUDM 在线管理器实现：维护跨周期上下文、任务级换道状态机与结果后处理
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * `EudmPlanner` 本体主要负责“单周期内”的策略生成与评估，
 * 而本文件中的 `EudmManager` 负责补齐在线系统真正需要的跨周期逻辑：
 * 1. 维护 ongoing action 的重规划上下文
 * 2. 管理 HMI / task 级换道触发状态机
 * 3. 对 planner 原始 winner 做任务语义约束下的二次筛选
 * 4. 保存规划快照，供后续行为构造与下一轮重规划使用
 *
 * **在 EUDM 框架中的位置**：
 * - `EudmPlanner`：论文核心求解器，偏“无状态”
 * - `EudmManager`：工程集成层，偏“有状态”
 *
 * **对应文档**：
 * - `paper/EUDM.md` 第 3 节：系统概述
 * - `paper/EUDM.md` 第 4 节：决策与策略序列执行
 */

/**
 * @brief 初始化 EUDM manager
 * @param config_path 配置文件路径
 * @param work_rate 在线运行频率
 *
 * **功能**：
 * 1. 初始化 glog 输出目录
 * 2. 初始化内部 `EudmPlanner`
 * 3. 绑定 map adapter
 * 4. 记录 manager 工作频率
 *
 * **设计意图**：
 * - manager 处于系统集成层，因此除了 planner 初始化外，还承担日志环境准备
 * - 便于在线回放与问题排查
 */
void EudmManager::Init(const std::string& config_path,
                       const decimal_t work_rate) {
  // 除了初始化 planner，本函数还会准备 glog 输出目录，
  // 便于分析 EUDM 在线运行日志。
  google::InitGoogleLogging("eudm");
  std::string log_dir = "/tmp/.eudm_log/";
  if (const char* home = std::getenv("HOME")) {
    log_dir = std::string(home) + "/.eudm_log/";
  }
  mkdir(log_dir.c_str(), 0755);
  google::SetLogDestination(google::GLOG_INFO,
                            (log_dir + "eudm_info_").c_str());
  google::SetLogDestination(google::GLOG_WARNING,
                            (log_dir + "eudm_warning_").c_str());
  google::SetLogDestination(google::GLOG_ERROR,
                            (log_dir + "eudm_error_").c_str());
  google::SetLogDestination(google::GLOG_FATAL,
                            (log_dir + "eudm_fatal_").c_str());

  bp_.Init(config_path);
  bp_.set_map_interface(&map_adapter_);
  work_rate_ = work_rate;
  if (bp_.cfg().function().active_lc_enable()) {
    LOG(ERROR) << "[HMI]HMI enabled with active lane change ON.";
  } else {
    LOG(ERROR) << "[HMI]HMI enabled with active lane change OFF.";
  }
}

/**
 * @brief 计算未来最近的动作层边界
 * @param stamp 当前时间戳
 * @param delta 额外的向前偏移量
 * @return 返回未来最近一个决策层起点
 *
 * **功能**：
 * 将任意时刻对齐到 DCP-Tree 动作层的时间边界上。
 *
 * **用途**：
 * - 保证 stick/active lane change 的触发时机与动作层严格对齐
 * - 避免在层中间插入新的横向触发，破坏动作序列语义
 *
 * **对应文档**：
 * - `paper/EUDM.md` 第 4.2 节：DCP-Tree 按层展开动作序列
 */
decimal_t EudmManager::GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                                     const decimal_t& delta) {
  // 把任意时刻对齐到未来最近的“动作层边界”，
  // 这样换道触发时机与 DCP 层结构能严格对齐。
  decimal_t past_decision_point =
      std::floor((stamp + delta) / bp_.cfg().sim().duration().layer()) *
      bp_.cfg().sim().duration().layer();
  return past_decision_point + bp_.cfg().sim().duration().layer();
}

/**
 * @brief 判断当前是否适合触发换道
 * @param lat 候选横向行为
 * @return 适合触发则返回 true
 *
 * **功能**：
 * 基于上一轮规划快照判断“现在是否是一个好的换道触发时机”。
 *
 * **当前实现说明**：
 * - 目前通过编译分支直接返回 `true`
 * - 下方保留了一套更严格的历史序列一致性检查逻辑，作为工程扩展接口
 *
 * **保留逻辑的意图**：
 * - 统计最近 winner 及其他有效序列中，是否有足够多候选在未来若干层内支持该换道
 * - 只有当换道在策略空间中足够“稳定一致”时，才认为值得触发
 */
bool EudmManager::IsTriggerAppropriate(const LateralBehavior& lat) {
#if 1
  return true;
#endif
  // check whether appropriate to conduct lat behavior right now,
  // if not, recommend a velocity instead.
  if (!last_snapshot_.valid) return false;
  // KXXXX
  // KKXXX
  // KKKLL
  // KKKKL
  const int kMinActionCheckIdx = 1;
  const int kMaxActionCheckIdx = 3;
  const int kMinMatchSeqs = 3;
  int num_action_seqs = last_snapshot_.action_script.size();
  int num_match_seqs = 0;
  for (int i = 0; i < num_action_seqs; i++) {
    if (!last_snapshot_.sim_res[i] || last_snapshot_.risky_res[i]) continue;
    auto action_seq = last_snapshot_.action_script[i];
    int num_actions = action_seq.size();
    for (int j = kMinActionCheckIdx; j <= kMaxActionCheckIdx; j++) {
      if (lat == LateralBehavior::kLaneChangeLeft &&
          action_seq[j].lat == DcpLatAction::kLaneChangeLeft &&
          (action_seq[j].lon == DcpLonAction::kAccelerate ||
           action_seq[j].lon == DcpLonAction::kMaintain)) {
        num_match_seqs++;
        break;
      } else if (lat == LateralBehavior::kLaneChangeRight &&
                 action_seq[j].lat == DcpLatAction::kLaneChangeRight &&
                 (action_seq[j].lon == DcpLonAction::kAccelerate ||
                  action_seq[j].lon == DcpLonAction::kMaintain)) {
        num_match_seqs++;
        break;
      }
    }
  }
  if (num_match_seqs < kMinMatchSeqs) return false;
  return true;
}

/**
 * @brief 为当前规划周期准备输入上下文
 * @param stamp 当前时间戳
 * @param map_ptr 当前语义地图
 * @param task 当前任务输入
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 在调用 `EudmPlanner::RunOnce()` 之前，把跨周期上下文与任务约束灌入 planner。
 *
 * **准备内容**：
 * 1. 更新 map adapter
 * 2. 从历史上下文恢复本轮应延续的 `desired_action`
 * 3. 更新当前 ego 所在车道 id
 * 4. 推进换道状态机 `lc_context_`
 * 5. 按 task 与道路几何重新评估参考速度
 * 6. 向 planner 注入 lane change forbid / recommend 信息
 *
 * **物理意义**：
 * - planner 只负责“给定当前上下文时怎么选策略”
 * - manager 则负责“当前上下文到底是什么”
 *
 * **对应文档**：
 * - 第 3 节：系统中决策模块与上游任务/HMI 的关系
 */
ErrorType EudmManager::Prepare(
    const decimal_t stamp,
    const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
    const planning::eudm::Task& task) {
  map_adapter_.set_map(map_ptr);

  // 先从历史上下文恢复本轮 ongoing action。
  DcpAction desired_action;
  if (!GetReplanDesiredAction(stamp, &desired_action)) {
    desired_action.lat = DcpLatAction::kLaneKeeping;
    desired_action.lon = DcpLonAction::kMaintain;
    decimal_t fdp_stamp = GetNearestFutureDecisionPoint(stamp, 0.0);
    desired_action.t = fdp_stamp - stamp;
  }

  if (map_adapter_.map()->GetEgoNearestLaneId(&ego_lane_id_) != kSuccess) {
    return kWrongStatus;
  }

  UpdateLaneChangeContextByTask(stamp, task);
  if (lc_context_.completed) {
    // 当前没有在执行中的换道任务时，横向默认回到车道保持。
    desired_action.lat = DcpLatAction::kLaneKeeping;
  }

  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]Replan context <valid, stamp, seq>:<"
              << context_.is_valid << "," << std::fixed << std::setprecision(3)
              << context_.seq_start_time << ",";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << ">";
    LOG(WARNING) << line_info.str();
  }
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]LC context <completed, twa, tt, dt, l_id, "
                 "lat, type>:<"
              << lc_context_.completed << ","
              << lc_context_.trigger_when_appropriate << "," << std::fixed
              << std::setprecision(3) << lc_context_.trigger_time << ","
              << lc_context_.desired_operation_time << ","
              << lc_context_.ego_lane_id << ","
              << common::SemanticsUtils::RetLatBehaviorName(lc_context_.lat)
              << "," << static_cast<int>(lc_context_.type) << ">";
    LOG(WARNING) << line_info.str();
  }

  bp_.UpdateDcpTree(desired_action);
  // 参考速度既受用户设定影响，也会被道路曲率二次裁剪。
  decimal_t ref_vel;
  EvaluateReferenceVelocity(task, &ref_vel);
  LOG(WARNING) << "[Eudm][Manager]<task vel, ref_vel>:<"
               << task.user_desired_vel << "," << ref_vel << ">";
  bp_.set_desired_velocity(ref_vel);

  auto lc_info = task.lc_info;
  // 用任务级上下文增强 lane-change 信息，例如注入推荐换道信号。
  if (!lc_context_.completed) {
    if (stamp >= lc_context_.desired_operation_time) {
      if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
        // LOG(WARNING) << std::fixed << std::setprecision(5)
        //              << "[HMI]Recommending left at " << stamp
        //              << " with desired time: "
        //              << lc_context_.desired_operation_time;
        lc_info.recommend_lc_left = true;
      } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight) {
        lc_info.recommend_lc_right = true;
      }
    }
  }
  bp_.set_lane_change_info(lc_info);

  LOG(WARNING) << "[Eudm][Manager]desired <lon,lat,t>:<"
               << DcpTree::RetLonActionName(desired_action.lon).c_str() << ","
               << DcpTree::RetLatActionName(desired_action.lat) << ","
               << desired_action.t << "> ego lane id:" << ego_lane_id_;

  return kSuccess;
}

/**
 * @brief 生成主动换道提案
 * @param stamp 当前时间戳
 * @param task 当前任务输入
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 从最近若干轮 EUDM winner 中，提炼出一个“稳定一致”的主动换道 proposal。
 *
 * **判定流程**：
 * 1. 先检查主动换道功能是否开启、当前是否在自动驾驶模式、是否已有进行中的换道
 * 2. 再检查冷却时间、速度范围、禁变道信号等先验条件
 * 3. 从上一轮 winner 序列中提取主横向意图与操作时机
 * 4. 将当前请求加入 `preliminary_active_requests_`
 * 5. 只有连续若干帧都保持同车道、同方向、相近操作时刻，才升级为正式 proposal
 *
 * **设计原因**：
 * - 防止单帧 winner 抖动导致主动换道信号过于敏感
 * - 将“planner 倾向”转化为“系统可采纳的主动建议”
 *
 * **对应文档**：
 * - 第 3 节：上层任务与决策输出的集成
 * - 第 4 节：策略序列具有长期语义，需要跨周期整合
 */
ErrorType EudmManager::GenerateLaneChangeProposal(
    const decimal_t& stamp, const planning::eudm::Task& task) {
  // 如果最近若干轮规划都稳定指向同一个换道意图，
  // manager 会把它沉淀成一个主动换道 proposal。
  if (!bp_.cfg().function().active_lc_enable()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to disabled lc:"
                 << stamp;
    return kSuccess;
  }

  if (!task.is_under_ctrl) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not under ctrl:"
                 << stamp;
    return kSuccess;
  }

  if (!lc_context_.completed) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not completed lc:"
                 << stamp;
    return kSuccess;
  }

  // if stick not reset, will not try active lane change
  if (lc_context_.completed && task.user_perferred_behavior != 0) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to stick not rest:"
                 << stamp;
    return kSuccess;
  }

  if (stamp - last_lc_proposal_.trigger_time < 0.0) {
    last_lc_proposal_.valid = false;
    last_lc_proposal_.trigger_time = stamp;
    last_lc_proposal_.ego_lane_id = ego_lane_id_;
    last_lc_proposal_.lat = LateralBehavior::kLaneKeeping;
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to illegal stamp:"
                 << stamp;
    return kSuccess;
  }

  if (stamp - last_lc_proposal_.trigger_time <
      bp_.cfg().function().active_lc().cold_duration()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to cold down:" << stamp
                 << " < " << last_lc_proposal_.trigger_time << " + "
                 << bp_.cfg().function().active_lc().cold_duration();
    return kSuccess;
  }

  if (last_snapshot_.plan_state.velocity <
          bp_.cfg().function().active_lc().activate_speed_lower_bound() ||
      last_snapshot_.plan_state.velocity >
          bp_.cfg().function().active_lc().activate_speed_upper_bound()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to illegal spd:"
                 << last_snapshot_.plan_state.velocity << " at time " << stamp;
    return kSuccess;
  }

  if (bp_.cfg()
          .function()
          .active_lc()
          .enable_clear_accumulation_by_forbid_signal() &&
      not preliminary_active_requests_.empty()) {
    auto last_request = preliminary_active_requests_.back();
    if (last_request.lat == LateralBehavior::kLaneChangeLeft &&
        task.lc_info.forbid_lane_change_left) {
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                   << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (last_request.lat == LateralBehavior::kLaneChangeRight &&
        task.lc_info.forbid_lane_change_right) {
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                   << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
  }

  common::LateralBehavior lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  // 从上一轮 winner 动作序列中提炼主横向意图和操作时机。
  bp_.ClassifyActionSeq(
      last_snapshot_.action_script[last_snapshot_.original_winner_id],
      &operation_at_seconds, &lat_behavior, &is_cancel_behavior);
  if (lat_behavior == LateralBehavior::kLaneKeeping || is_cancel_behavior) {
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not ideal behavior:"
                 << stamp;
    preliminary_active_requests_.clear();
    return kSuccess;
  }

  ActivateLaneChangeRequest this_request;
  this_request.trigger_time = stamp;
  this_request.desired_operation_time = stamp + operation_at_seconds;
  this_request.ego_lane_id = ego_lane_id_;
  this_request.lat = lat_behavior;
  if (preliminary_active_requests_.empty()) {
    // 第一帧只先入队，等待后续帧验证稳定性。
    preliminary_active_requests_.push_back(this_request);
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                 << " Init requesting "
                 << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                 << " at " << this_request.desired_operation_time
                 << " with lane id " << this_request.ego_lane_id;
  } else {
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                 << " Consequent requesting "
                 << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                 << " at " << this_request.desired_operation_time
                 << " with lane id " << this_request.ego_lane_id;
    auto last_request = preliminary_active_requests_.back();
    if (last_request.ego_lane_id != this_request.ego_lane_id) {
      LOG(WARNING)
          << "[Eudm][ActiveLc]Invalid this request due to lane id inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (last_request.lat != this_request.lat) {
      LOG(WARNING) << "[Eudm][ActiveLc]Invalid this request due to behavior "
                      "inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (fabs(last_request.desired_operation_time -
             this_request.desired_operation_time) >
        bp_.cfg().function().active_lc().consistent_operate_time_min_gap()) {
      LOG(WARNING)
          << "[Eudm][ActiveLc]Invalid this request due to time inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    preliminary_active_requests_.push_back(this_request);
    LOG(WARNING) << "[Eudm][ActiveLc]valid this request. Queue size "
                 << preliminary_active_requests_.size() << " and operate at "
                 << operation_at_seconds;
  }

  if (preliminary_active_requests_.size() >=
      bp_.cfg().function().active_lc().consistent_min_num_frame()) {
    // 连续若干帧一致后，才正式生成 proposal。
    if (operation_at_seconds <
        bp_.cfg().function().active_lc().activate_max_duration_in_seconds() +
            kEPS) {
      last_lc_proposal_.valid = true;
      last_lc_proposal_.trigger_time = stamp;
      last_lc_proposal_.operation_at_seconds =
          operation_at_seconds > bp_.cfg()
                                     .function()
                                     .active_lc()
                                     .active_min_operation_in_seconds()
              ? operation_at_seconds
              : GetNearestFutureDecisionPoint(
                    stamp, bp_.cfg()
                               .function()
                               .active_lc()
                               .active_min_operation_in_seconds()) -
                    stamp;
      last_lc_proposal_.ego_lane_id = ego_lane_id_;
      last_lc_proposal_.lat = lat_behavior;
      preliminary_active_requests_.clear();
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[HMI]Gen proposal with trigger time "
                   << last_lc_proposal_.trigger_time << " lane id "
                   << last_lc_proposal_.ego_lane_id << " behavior "
                   << static_cast<int>(last_lc_proposal_.lat) << " operate at "
                   << last_lc_proposal_.operation_at_seconds;
    } else {
      preliminary_active_requests_.clear();
      // LOG(WARNING) << "[HMI]Abandan Queue due to change time not legal.";
    }
  }

  return kSuccess;
}

/**
 * @brief 根据 task/HMI 输入推进换道上下文状态机
 * @param stamp 当前时间戳
 * @param task 当前任务输入
 *
 * **功能**：
 * 维护一个任务级 lane change state machine，统一处理：
 * 1. 自动驾驶启停
 * 2. 人工 stick 触发与取消
 * 3. “条件成熟再触发”的缓存请求
 * 4. active lane change proposal 的接受、过期与取消
 *
 * **状态语义**：
 * - `completed = true`：当前没有进行中的换道任务
 * - `trigger_when_appropriate = true`：已有换道意图，但等待合适时机
 * - `desired_operation_time`：计划真正开始横向动作的绝对时间
 *
 * **与 planner 的关系**：
 * - planner 决定哪条策略更优
 * - manager 决定当前是否允许/要求 planner 偏向某类横向行为
 *
 * **说明**：
 * - 这部分属于明显的工程状态机，不是论文公式主体
 * - 但它对在线系统的稳定性和 HMI 一致性非常关键
 */
void EudmManager::UpdateLaneChangeContextByTask(
    const decimal_t stamp, const planning::eudm::Task& task) {
  // 这里维护一个完整的任务级换道状态机，负责处理:
  // 1. 自动驾驶启停
  // 2. 用户 stick 触发/取消
  // 3. cached trigger 的延迟触发
  // 4. active lane change proposal 的接纳与取消
  if (!last_task_.is_under_ctrl && task.is_under_ctrl) {
    LOG(WARNING) << "[HMI]Autonomous mode activated!";
    lc_context_.completed = true;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (last_task_.is_under_ctrl && !task.is_under_ctrl) {
    LOG(WARNING) << "[HMI]Autonomous mode deactivated!";
    lc_context_.completed = true;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (task.user_perferred_behavior != last_task_.user_perferred_behavior) {
    LOG(WARNING) << "[HMI]stick state change from "
                 << last_task_.user_perferred_behavior << " to "
                 << task.user_perferred_behavior;
  }

  if ((task.lc_info.forbid_lane_change_left !=
       last_task_.lc_info.forbid_lane_change_left) ||
      (task.lc_info.forbid_lane_change_right !=
       last_task_.lc_info.forbid_lane_change_right)) {
    LOG(WARNING) << "[HMI]lane change forbid signal [left] "
                 << task.lc_info.forbid_lane_change_left << " [right] "
                 << task.lc_info.forbid_lane_change_right;
  }

  if (task.is_under_ctrl) {
    if (!lc_context_.completed) {
      if (!map_adapter_.IsLaneConsistent(lc_context_.ego_lane_id,
                                         ego_lane_id_)) {
        // in progress lane change and lane id change
        LOG(WARNING) << "[HMI]lane change completed due to different lane id "
                     << lc_context_.ego_lane_id << " to " << ego_lane_id_
                     << ". Cd alc.";
        lc_context_.completed = true;
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } else {
        if (task.user_perferred_behavior != 1 &&
            last_task_.user_perferred_behavior == 1) {
          // receive a lane cancel trigger
          LOG(WARNING) << "[HMI]lane change cancel by stick "
                       << last_task_.user_perferred_behavior << " to "
                       << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        } else if (task.user_perferred_behavior != -1 &&
                   last_task_.user_perferred_behavior == -1) {
          // receive a lane cancel trigger
          LOG(WARNING) << "[HMI]lane change cancel by stick "
                       << last_task_.user_perferred_behavior << " to "
                       << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        } else if (lc_context_.type == LaneChangeTriggerType::kActive) {
          if (bp_.cfg()
                  .function()
                  .active_lc()
                  .enable_auto_cancel_by_outdate_time() &&
              stamp > lc_context_.desired_operation_time +
                          bp_.cfg()
                              .function()
                              .active_lc()
                              .auto_cancel_if_late_for_seconds()) {
            if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG(WARNING)
                  << "[HMI]ACTIVE [Left] auto cancel due to outdated for "
                  << stamp - lc_context_.desired_operation_time
                  << " s. Cd alc.";
            } else {
              LOG(WARNING)
                  << "[HMI]ACTIVE [Right] auto cancel due to outdated for "
                  << stamp - lc_context_.desired_operation_time
                  << " s. Cd alc.";
            }
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_left &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
            LOG(WARNING) << "[HMI]ACTIVE [Left] canceled due to forbidden "
                            "signal. Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_right &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight) {
            LOG(WARNING)
                << "[HMI]ACTIVE [Right] canceled due to forbidden signal. "
                   "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
                     (task.user_perferred_behavior == 1 ||
                      task.user_perferred_behavior == 11)) {
            LOG(WARNING)
                << "[HMI]ACTIVE [left] canceled due to human opposite signal. "
                   "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                     (task.user_perferred_behavior == -1 ||
                      task.user_perferred_behavior == 12)) {
            LOG(WARNING) << "[HMI]ACTIVE canceled due to human active signal. "
                            "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          }
        }
      }
    } else {
      // 已完成状态下，优先处理新的 stick 指令，其次处理主动换道提案。
      if (task.user_perferred_behavior != 1 &&
          last_task_.user_perferred_behavior == 1 &&
          lc_context_.trigger_when_appropriate) {
        LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } else if (task.user_perferred_behavior != -1 &&
                 last_task_.user_perferred_behavior == -1 &&
                 lc_context_.trigger_when_appropriate) {
        LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      }

      if (task.user_perferred_behavior == 1 &&
          last_task_.user_perferred_behavior != 1) {
        // 收到右换道 stick 指令，且当前没有进行中的换道任务。
        if (task.lc_info.forbid_lane_change_right) {
          LOG(WARNING)
              << "[HMI]cannot stick [Right]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeRight;
        } else {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeRight)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI]stick [Right] triggered "
                         << last_task_.user_perferred_behavior << "->"
                         << task.user_perferred_behavior << " in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          } else {
            lc_context_.trigger_when_appropriate = true;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            LOG(WARNING)
                << std::fixed << std::setprecision(5)
                << "[HMI]stick [Right] triggered "
                << last_task_.user_perferred_behavior << "->"
                << task.user_perferred_behavior
                << " but not a good time. Will trigger when appropriate.";
          }
        }
      } else if (task.user_perferred_behavior == -1 &&
                 last_task_.user_perferred_behavior != -1) {
        if (task.lc_info.forbid_lane_change_left) {
          LOG(WARNING)
              << "[HMI]cannot stick [Left]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeLeft;
        } else {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeLeft)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI]stick [Left] triggered "
                         << last_task_.user_perferred_behavior << "->"
                         << task.user_perferred_behavior << " in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          } else {
            lc_context_.trigger_when_appropriate = true;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            LOG(WARNING)
                << std::fixed << std::setprecision(5)
                << "[HMI]stick [Left] triggered "
                << last_task_.user_perferred_behavior << "->"
                << task.user_perferred_behavior
                << " but not a good time. Will trigger when appropriate.";
          }
        }
      } else if (lc_context_.trigger_when_appropriate) {
        if (lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
            !task.lc_info.forbid_lane_change_left) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeLeft)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI][[cached]] stick [Left] appropriate in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          }
        } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                   !task.lc_info.forbid_lane_change_right) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeRight)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI][[cached]] stick [Right] triggered in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          }
        }
      } else {
        // 若没有新的 stick 请求，则尝试采用系统自己提出的主动换道提案。
        if (last_lc_proposal_.valid &&
            map_adapter_.IsLaneConsistent(last_lc_proposal_.ego_lane_id,
                                          ego_lane_id_) &&
            stamp > last_lc_proposal_.trigger_time &&
            last_lc_proposal_.lat != LateralBehavior::kLaneKeeping) {
          if ((last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft &&
               !task.lc_info.forbid_lane_change_left) ||
              (last_lc_proposal_.lat == LateralBehavior::kLaneChangeRight &&
               !task.lc_info.forbid_lane_change_right)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time =
                last_lc_proposal_.trigger_time +
                last_lc_proposal_.operation_at_seconds;
            lc_context_.ego_lane_id = last_lc_proposal_.ego_lane_id;
            lc_context_.lat = last_lc_proposal_.lat;
            lc_context_.type = LaneChangeTriggerType::kActive;
            last_lc_proposal_.trigger_time = stamp;
            if (last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG(WARNING) << std::fixed << std::setprecision(5)
                           << "[HMI][[Active]] [Left] triggered in "
                           << last_lc_proposal_.operation_at_seconds
                           << " s. Trigger time " << lc_context_.trigger_time
                           << " and absolute action time: "
                           << lc_context_.desired_operation_time << ". Cd alc.";
            } else {
              LOG(WARNING) << std::fixed << std::setprecision(5)
                           << "[HMI][[Active]] [Right] triggered in "
                           << last_lc_proposal_.operation_at_seconds
                           << " s. Trigger time " << lc_context_.trigger_time
                           << " and absolute action time: "
                           << lc_context_.desired_operation_time << ". Cd alc.";
            }
          }
        }
      }
    }
  }  // if under control

  // any proposal will not last for more than one cycle
  last_lc_proposal_.valid = false;
  last_task_ = task;
}  // namespace planning

/**
 * @brief 保存当前 planner 输出为完整快照
 * @param snapshot 输出：规划结果快照
 *
 * **功能**：
 * 把 `EudmPlanner` 本轮求解得到的所有关键结果打包保存，
 * 供后续重选、行为构造、可视化和下一轮重规划使用。
 *
 * **快照内容**：
 * - 原始 winner id
 * - 所有动作序列及其有效性/风险性
 * - 各序列代价、前向轨迹、行为序列和周边车轨迹
 * - 本轮规划状态、时间戳与耗时
 *
 * **设计意义**：
 * - 将“planner 单轮输出”从在线状态机中解耦出来
 * - 便于 manager 做二次处理而不重复访问 planner 内部状态
 */
void EudmManager::SaveSnapshot(Snapshot* snapshot) {
  // 保存本轮 planner 的完整结果，供后处理、可视化和下轮上下文使用。
  snapshot->valid = true;
  snapshot->plan_state = bp_.plan_state();
  snapshot->original_winner_id = bp_.winner_id();
  snapshot->processed_winner_id = bp_.winner_id();
  snapshot->action_script = bp_.action_script();
  snapshot->sim_res = bp_.sim_res();
  snapshot->risky_res = bp_.risky_res();
  snapshot->sim_info = bp_.sim_info();
  snapshot->final_cost = bp_.final_cost();
  snapshot->progress_cost = bp_.progress_cost();
  snapshot->tail_cost = bp_.tail_cost();
  snapshot->forward_trajs = bp_.forward_trajs();
  snapshot->forward_lat_behaviors = bp_.forward_lat_behaviors();
  snapshot->forward_lon_behaviors = bp_.forward_lon_behaviors();
  snapshot->surround_trajs = bp_.surround_trajs();

  snapshot->plan_stamp = map_adapter_.map()->time_stamp();
  snapshot->time_cost = bp_.time_cost();
}

/**
 * @brief 将最近一次规划结果构造成下游语义行为
 * @param behavior 输出：语义行为结果
 *
 * **功能**：
 * 从 `last_snapshot_` 中提取 manager 最终采纳的 winner，
 * 生成下游模块使用的 `SemanticBehavior`。
 *
 * **输出内容**：
 * - 当前横向/纵向行为
 * - 对应的自车前向轨迹
 * - 周边车预测轨迹
 * - 规划起始状态与参考车道
 *
 * **说明**：
 * - 这里使用的是 `processed_winner_id`
 * - 即已经经过 manager 上下文二次筛选后的最终行为，而非 planner 原始 winner
 */
void EudmManager::ConstructBehavior(common::SemanticBehavior* behavior) {
  if (not last_snapshot_.valid) return;
  // 对下游只暴露 manager 最终采纳的 processed winner。
  int selected_seq_id = last_snapshot_.processed_winner_id;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_final;
  surround_trajs_final.emplace_back(
      last_snapshot_.surround_trajs[selected_seq_id]);
  behavior->lat_behavior =
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front();
  behavior->lon_behavior =
      last_snapshot_.forward_lon_behaviors[selected_seq_id].front();
  behavior->forward_trajs = vec_E<vec_E<common::Vehicle>>{
      last_snapshot_.forward_trajs[selected_seq_id]};
  behavior->forward_behaviors = std::vector<LateralBehavior>{
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front()};
  behavior->surround_trajs = surround_trajs_final;
  behavior->state = last_snapshot_.plan_state;
  behavior->ref_lane = last_snapshot_.ref_lane;
}

/**
 * @brief 评估参考速度
 * @param task 当前任务输入
 * @param ref_vel 输出：参考速度
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 将用户期望速度与道路几何约束结合，得到更保守、可执行的参考速度。
 *
 * **计算逻辑**：
 * 1. 若当前参考车道无效，则直接退化为用户期望速度
 * 2. 否则沿前向参考车道扫描一定纵向范围
 * 3. 根据曲率上界计算舒适横向加速度约束下的最大允许速度
 * 4. 取扫描范围内最小值，并与用户期望速度共同裁剪
 *
 * **物理意义**：
 * - 曲率越大，车辆在舒适横向加速度限制下允许的速度越低
 * - 这一步是对用户意图的“几何可执行性修正”
 *
 * **对应关系**：
 * - 不直接来自 EUDM 论文公式主体
 * - 属于工程系统中“用户期望 -> 实际参考速度”的接口层处理
 */
ErrorType EudmManager::EvaluateReferenceVelocity(
    const planning::eudm::Task& task, decimal_t* ref_vel) {
  // 曲率越大，允许的舒适速度上限越低。
  // 因而这里会沿前向参考车道扫描曲率，给用户期望速度做二次裁剪。
  if (!last_snapshot_.ref_lane.IsValid()) {
    *ref_vel = task.user_desired_vel;
    return kSuccess;
  }
  common::StateTransformer stf(last_snapshot_.ref_lane);
  common::FrenetState current_fs;
  stf.GetFrenetStateFromState(last_snapshot_.plan_state, &current_fs);

  decimal_t c, cc;
  decimal_t v_max_by_curvature;
  decimal_t v_ref = kInf;

  decimal_t a_comfort = bp_.cfg().sim().ego().lon().limit().soft_brake();
  decimal_t t_forward = last_snapshot_.plan_state.velocity / a_comfort;
  decimal_t s_forward =
      std::min(std::max(20.0, t_forward * last_snapshot_.plan_state.velocity),
               last_snapshot_.ref_lane.end());
  decimal_t resolution = 0.2;

  for (decimal_t s = current_fs.vec_s[0]; s < current_fs.vec_s[0] + s_forward;
       s += resolution) {
    if (last_snapshot_.ref_lane.GetCurvatureByArcLength(s, &c, &cc) ==
        kSuccess) {
      v_max_by_curvature =
          sqrt(bp_.cfg().sim().ego().lat().limit().acc() / fabs(c));
      v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
    }
  }

  *ref_vel = std::floor(std::min(std::max(v_ref, 0.0), task.user_desired_vel));

  LOG(WARNING) << "[Eudm][Desired]User ref vel: " << task.user_desired_vel
               << ", final ref vel: " << *ref_vel;
  return kSuccess;
}

/**
 * @brief 按上下文对 planner 原始 winner 做二次筛选
 * @param stamp 当前时间戳
 * @param snapshot 本轮 planner 快照
 * @param new_seq_id 输出：重新选择后的序列 id
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 在所有有效动作序列中，选出最符合当前任务级换道上下文的一条。
 *
 * **筛选原则**：
 * 1. 若当前无进行中的换道任务，则偏向 `LaneKeeping`
 * 2. 若存在进行中的换道任务但尚未到触发时刻，仍允许保持
 * 3. 若已到触发时刻，则优先匹配 `lc_context_.lat` 对应的换道方向
 * 4. 在满足上下文约束的候选中，再取总代价最小者
 *
 * **设计意义**：
 * - planner 的最优代价不一定等于系统当前最合适的执行策略
 * - manager 通过这一层，把“任务语义一致性”放在“代价最优”之前
 */
ErrorType EudmManager::ReselectByContext(const decimal_t stamp,
                                         const Snapshot& snapshot,
                                         int* new_seq_id) {
  // planner 原始 winner 可能与当前任务上下文不完全匹配。
  // 这里会在有效动作序列里重新找一条更符合任务状态机的策略。
  // *new_seq_id = snapshot.original_winner_id;
  int selected_seq_id;
  int num_seqs = snapshot.action_script.size();
  bool find_match = false;
  decimal_t cost = kInf;

  for (int i = 0; i < num_seqs; i++) {
    if (!snapshot.sim_res[i]) continue;
    common::LateralBehavior lat_behavior;
    decimal_t operation_at_seconds;
    bool is_cancel_behavior;
    bp_.ClassifyActionSeq(snapshot.action_script[i], &operation_at_seconds,
                          &lat_behavior, &is_cancel_behavior);

    if ((lc_context_.completed &&
         lat_behavior == common::LateralBehavior::kLaneKeeping) ||
        (!lc_context_.completed && stamp < lc_context_.desired_operation_time &&
         lat_behavior == common::LateralBehavior::kLaneKeeping) ||
        (!lc_context_.completed &&
         stamp >= lc_context_.desired_operation_time &&
         (lat_behavior == lc_context_.lat ||
          lat_behavior == common::LateralBehavior::kLaneKeeping))) {
      find_match = true;
      if (snapshot.final_cost[i] < cost) {
        cost = snapshot.final_cost[i];
        selected_seq_id = i;
      }
    }
  }

  if (!find_match) {
    return kWrongStatus;
  }
  *new_seq_id = selected_seq_id;
  return kSuccess;
}

/**
 * @brief 执行一次完整的 manager 在线流程
 * @param stamp 当前时间戳
 * @param map_ptr 当前语义地图
 * @param task 当前任务输入
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **完整流程**：
 * 1. `Prepare()`：
 *    - 恢复 ongoing action、更新上下文和参考速度
 * 2. `bp_.RunOnce()`：
 *    - 调用 EUDM planner 完成单周期策略求解
 * 3. `SaveSnapshot()`：
 *    - 保存 planner 原始输出
 * 4. `ReselectByContext()`：
 *    - 基于任务级状态机重新选择最终 winner
 * 5. 重新拟合最终参考车道
 * 6. `GenerateLaneChangeProposal()`：
 *    - 从本轮结果中提炼主动换道建议
 * 7. 更新下一轮重规划上下文 `context_`
 *
 * **与 EudmPlanner::RunOnce 的关系**：
 * - `EudmPlanner::RunOnce()` 负责“求解”
 * - `EudmManager::Run()` 负责“在线系统级编排”
 *
 * **输出结果**：
 * - `last_snapshot_`：完整结果快照
 * - `context_`：下一轮重规划所需的动作继承上下文
 * - `last_lc_proposal_`：可能的主动换道提案
 */
ErrorType EudmManager::Run(
    const decimal_t stamp,
    const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
    const planning::eudm::Task& task) {
  // manager 的完整在线工作流:
  // I.   Prepare
  // II.  调用 planner.RunOnce()
  // III. 保存快照
  // IV.  依据任务上下文重选赢家
  // V.   更新参考车道、proposal 与下一轮重规划上下文
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]******************** RUN START: " << stamp
               << "******************";
  static TicToc eudm_timer;
  eudm_timer.tic();

  // * I : Prepare
  static TicToc prepare_timer;
  prepare_timer.tic();
  if (Prepare(stamp, map_ptr, task) != kSuccess) {
    return kWrongStatus;
  }
  auto t_prepare = prepare_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]Prepare time cost " << t_prepare << " ms";

  // * II : RunOnce
  static TicToc runonce_timer;
  runonce_timer.tic();
  if (bp_.RunOnce() != kSuccess) {
    LOG(WARNING) << "[Eudm][Fatal]BP runonce failed.";
    return kWrongStatus;
  }
  auto t_runonce = runonce_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]RunOnce time cost " << t_runonce << " ms";

  static TicToc sum_reselect_timer;
  sum_reselect_timer.tic();
  // * III: Summarize
  Snapshot snapshot;
  SaveSnapshot(&snapshot);
  // * IV: Reselect
  if (ReselectByContext(stamp, snapshot, &snapshot.processed_winner_id) !=
      kSuccess) {
    LOG(WARNING) << "[Eudm][Fatal]Reselect failed.";
    return kWrongStatus;
  }
  LOG(WARNING) << "[Eudm]original id " << snapshot.original_winner_id
               << " reselect : " << snapshot.processed_winner_id;
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Output]Reselected <if_risky:"
              << snapshot.risky_res[snapshot.processed_winner_id] << ">[";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    for (auto& v : snapshot.forward_trajs[snapshot.processed_winner_id]) {
      line_info << std::fixed << std::setprecision(5) << "<"
                << v.state().time_stamp - stamp << "," << v.state().velocity
                << "," << v.state().acceleration << "," << v.state().curvature
                << ">";
    }
    LOG(WARNING) << line_info.str();
  }
  auto t_sum_reselect = sum_reselect_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]Sum & Reselect time cost " << t_sum_reselect << " ms";

  static TicToc lane_timer;
  lane_timer.tic();
  if (map_adapter_.map()->GetRefLaneForStateByBehavior(
          snapshot.plan_state, std::vector<int>(),
          snapshot.forward_lat_behaviors[snapshot.processed_winner_id].front(),
          250.0, 20.0, true, &(snapshot.ref_lane)) != kSuccess) {
    return kWrongStatus;
  }

  last_snapshot_ = snapshot;
  GenerateLaneChangeProposal(stamp, task);
  // * V: Update
  // 更新下一轮要继承的动作上下文。
  context_.is_valid = true;
  context_.seq_start_time = stamp;
  context_.action_seq = snapshot.action_script[snapshot.processed_winner_id];
  auto t_lane = lane_timer.toc();
  LOG(WARNING) << "[Eudm]Fit reflane & update cost: " << t_lane << " ms";

  auto t_sum = t_prepare + t_runonce + t_sum_reselect + t_lane;
  auto t_eudmrun = eudm_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]Sum of time: " << t_sum
               << " ms, diff: " << t_eudmrun - t_sum << " ms";
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]******************** RUN FINISH: " << stamp << " +"
               << t_eudmrun << " ms ******************";
  return kSuccess;
}

/**
 * @brief 从历史上下文中恢复当前应延续的动作
 * @param current_time 当前时间戳
 * @param desired_action 输出：当前应执行的动作
 * @return 若成功恢复则返回 true
 *
 * **功能**：
 * 在重规划时，根据上一轮已选动作序列和序列起始时刻，
 * 找到“当前时间落在哪个动作段内”，并把该动作作为新一轮 DCP-Tree 的根动作。
 *
 * **作用**：
 * - 保证 planner 在相邻周期之间延续 ongoing action
 * - 避免每轮都从全新的 `LK + Maintain` 根节点重新开始，造成行为抖动
 *
 * **实现逻辑**：
 * - 计算 `current_time - context_.seq_start_time`
 * - 沿历史动作序列累计持续时间
 * - 找到首个覆盖当前时刻的动作段
 * - 将其剩余时长作为新的 `desired_action.t`
 */
bool EudmManager::GetReplanDesiredAction(const decimal_t current_time,
                                         DcpAction* desired_action) {
  // 从上一轮动作序列中找到“当前时刻仍应执行的那个动作”，
  // 并把它作为本轮 DCP 树的根动作。
  if (!context_.is_valid) return false;
  decimal_t time_since_last_plan = current_time - context_.seq_start_time;
  if (time_since_last_plan < -kEPS) return false;
  decimal_t t_aggre = 0.0;
  bool find_match_action = false;
  int action_seq_len = context_.action_seq.size();
  for (int i = 0; i < action_seq_len; ++i) {
    t_aggre += context_.action_seq[i].t;
    if (time_since_last_plan + kEPS < t_aggre) {
      *desired_action = context_.action_seq[i];
      desired_action->t = t_aggre - time_since_last_plan;
      find_match_action = true;
      break;
    }
  }
  if (!find_match_action) {
    return false;
  }
  return true;
}

/**
 * @brief 重置 manager 上下文
 *
 * **功能**：
 * 清空跨周期动作继承信息，使下一轮规划从无历史上下文重新开始。
 *
 * **说明**：
 * - 这是 manager 级别的 reset，不会重建 planner 配置
 * - 主要用于模式切换、系统重启或异常恢复后重新对齐
 */
void EudmManager::Reset() {
  // 清空跨周期上下文，让下一轮从无历史继承状态重新开始。
  context_.is_valid = false;
}

/**
 * @brief 获取内部 EUDM planner 引用
 * @return 内部 planner 引用
 *
 * **用途**：
 * 便于外部模块访问 planner 的配置、快照信息或调试接口。
 */
EudmPlanner& EudmManager::planner() { return bp_; }

}  // namespace planning
