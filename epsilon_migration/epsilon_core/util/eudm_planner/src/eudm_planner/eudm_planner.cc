#include "eudm_planner/eudm_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "common/rss/rss_checker.h"

namespace planning {

/**
 * @file eudm_planner.cc
 * @author GW
 * @brief EUDM（基于引导分支的高效不确定性感知决策）规划器实现
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件实现了 EUDM 规划器的核心逻辑，对应论文《Efficient Uncertainty-aware Decision-making
 * for Automated Driving Using Guided Branching》。
 *
 * **核心功能**：
 * 1. **DCP-Tree 构建**：利用领域特定闭环策略树（DCP-Tree）生成语义级动作序列。
 * 2. **前向仿真**：基于智能驾驶模型（IDM）和纯追踪控制器进行闭环前向仿真。
 * 3. **多线程并行评估**：并行评估每个策略序列，计算安全、效率和舒适性指标。
 * 4. **风险评估**：通过 RSS（责任敏感安全）模型进行严格的安全检查。
 *
 * **主要流程（对应 RunEudm）**：
 * 1. 获取周边车辆信息
 * 2. 从 DCP-Tree 获取动作序列脚本
 * 3. 并行执行前向仿真（SimulateActionSequence -> SimulateScenario）
 * 4. 汇总仿真结果并计算代价
 * 5. 选择最优策略（EvaluateMultiThreadSimResults）
 */
std::string EudmPlanner::Name() { return std::string("Eudm behavior planner"); }

/**
 * @brief 从配置文件读取 EUDM 配置参数
 * @param config_path 配置文件路径（protobuf 文本格式）
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 读取并解析 EUDM 规划器配置，加载到 `cfg_` 中。
 *
 * **配置内容概览**（对应文档第 3 节与第 5 节）：
 * 1. **仿真时域与 DCP-Tree 参数**：
 *    - `tree_height / layer / last_layer`：决定策略树高度与每层动作持续时间
 *    - 对应文档第 4.2 节：DCP-Tree 的动作域构建
 * 2. **自车与他车前向仿真参数**：
 *    - 纵向 IDM 参数、横向 pure pursuit 参数、动力学限幅
 *    - 对应文档第 5 节：实现细节与闭环仿真设置
 * 3. **安全配置**：
 *    - RSS 检查参数、严格碰撞检查膨胀参数
 *    - 对应文档第 3 节和第 5 节：风险评估与安全性实现
 * 4. **代价函数参数**：
 *    - 效率、安全、导航/一致性相关权重和单位代价
 *    - 对应文档第 5.5 节：奖励/代价评估
 */
ErrorType EudmPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading eudm planner config\n");
  using namespace google::protobuf;
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  TextFormat::Parse(&fstream, &cfg_);
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

/**
 * @brief 将 protobuf 仿真配置转换为前向仿真参数
 * @param cfg 单个智能体的前向仿真配置
 * @param sim_param 输出：前向仿真参数
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 将配置文件中的纵向、横向控制与动力学约束参数整理为
 * `OnLaneForwardSimulation::Param`，供闭环前向仿真使用。
 *
 * **参数映射**：
 * 1. **纵向控制**：
 *    - IDM 跟驰参数：最小间距、期望车头时距、指数项
 *    - 加减速与 jerk 限制
 * 2. **横向控制**：
 *    - pure pursuit 的增益与预瞄距离
 *    - 横向加速度、横向 jerk、曲率、转角与转角变化率限制
 * 3. **失败退化策略**：
 *    - `auto_decelerate_if_lat_failed`：横向控制失败时是否自动减速
 *
 * **对应文档**：
 * - 第 3 节：闭环前向仿真
 * - 第 5 节：实现细节
 */
ErrorType EudmPlanner::GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                                   OnLaneForwardSimulation::Param* sim_param) {
  sim_param->idm_param.kMinimumSpacing = cfg.lon().idm().min_spacing();
  sim_param->idm_param.kDesiredHeadwayTime = cfg.lon().idm().head_time();
  sim_param->idm_param.kAcceleration = cfg.lon().limit().acc();
  sim_param->idm_param.kComfortableBrakingDeceleration =
      cfg.lon().limit().soft_brake();
  sim_param->idm_param.kHardBrakingDeceleration =
      cfg.lon().limit().hard_brake();
  sim_param->idm_param.kExponent = cfg.lon().idm().exponent();
  sim_param->max_lon_acc_jerk = cfg.lon().limit().acc_jerk();
  sim_param->max_lon_brake_jerk = cfg.lon().limit().brake_jerk();
  sim_param->steer_control_gain = cfg.lat().pure_pursuit().gain();
  sim_param->steer_control_max_lookahead_dist =
      cfg.lat().pure_pursuit().max_lookahead_dist();
  sim_param->steer_control_min_lookahead_dist =
      cfg.lat().pure_pursuit().min_lookahead_dist();
  sim_param->max_lat_acceleration_abs = cfg.lat().limit().acc();
  sim_param->max_lat_jerk_abs = cfg.lat().limit().jerk();
  sim_param->max_curvature_abs = cfg.lat().limit().curvature();
  sim_param->max_steer_angle_abs = cfg.lat().limit().steer_angle();
  sim_param->max_steer_rate = cfg.lat().limit().steer_rate();
  sim_param->auto_decelerate_if_lat_failed = cfg.auto_dec_if_lat_failed();
  return kSuccess;
}

/**
 * @brief 初始化 EUDM 规划器
 * @param config 配置文件路径
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **初始化流程**（对应文档算法 1 的准备阶段）：
 * 1. 读取配置文件
 * 2. 构建 DCP-Tree
 * 3. 初始化自车与他车的闭环前向仿真参数
 * 4. 初始化常规 RSS 与严格 RSS 配置
 *
 * **关键设计**：
 * - **DCP-Tree**：
 *   根植于“每个规划周期内最多一次语义动作变化”的先验，
 *   将动作序列复杂度从朴素指数增长压缩到与树高近似线性相关
 * - **双层安全检查**：
 *   常规 RSS 用于代价评估中的风险惩罚，严格 RSS/碰撞检查用于提前裁剪不可行动作
 *
 * **对应文档**：
 * - 第 4.2 节：DCP-Tree
 * - 第 5 节：实现细节
 */
ErrorType EudmPlanner::Init(const std::string config) {
  ReadConfig(config);

  // **DCP-Tree 初始化**
  // 对应论文 4.2 节：领域特定闭环策略树
  // 树的高度（tree_height）决定了规划时域（例如 4 层 * 2s/层 = 8s）
  dcp_tree_ptr_ = new DcpTree(cfg_.sim().duration().tree_height(),
                              cfg_.sim().duration().layer(),
                              cfg_.sim().duration().last_layer());
  LOG(INFO) << "[Eudm]Init.";
  LOG(INFO) << "[Eudm]ActionScript size: "
            << dcp_tree_ptr_->action_script().size() << std::endl;

  GetSimParam(cfg_.sim().ego(), &ego_sim_param_);
  GetSimParam(cfg_.sim().agent(), &agent_sim_param_);

  rss_config_ = common::RssChecker::RssConfig(
      cfg_.safety().rss().response_time(),
      cfg_.safety().rss().longitudinal_acc_max(),
      cfg_.safety().rss().longitudinal_brake_min(),
      cfg_.safety().rss().longitudinal_brake_max(),
      cfg_.safety().rss().lateral_acc_max(),
      cfg_.safety().rss().lateral_brake_min(),
      cfg_.safety().rss().lateral_brake_max(),
      cfg_.safety().rss().lateral_miu());

  rss_config_strict_as_front_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_front().response_time(),
      cfg_.safety().rss_strict_as_front().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_acc_max(),
      cfg_.safety().rss_strict_as_front().lateral_brake_min(),
      cfg_.safety().rss_strict_as_front().lateral_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_miu());

  rss_config_strict_as_rear_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_rear().response_time(),
      cfg_.safety().rss_strict_as_rear().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_acc_max(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_min(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_miu());

  return kSuccess;
}

/**
 * @brief 将 DCP 动作翻译为系统内部的横纵向行为枚举
 * @param action DCP-Tree 中的语义动作
 * @param lat 输出：横向行为
 * @param lon 输出：纵向行为
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 将 DCP-Tree 中的动作节点映射到闭环仿真与后续评估使用的
 * `LateralBehavior / LongitudinalBehavior`。
 *
 * **语义对应**（对应文档第 4.2 节）：
 * - 横向：LK / LCL / LCR
 * - 纵向：Maintain / Accelerate / Decelerate
 *
 * **用途**：
 * - `UpdateSimSetupForLayer()`：设置单层仿真目标
 * - `UpdateEgoBehaviorsUsingAction()`：刷新自车当前行为
 */
ErrorType EudmPlanner::TranslateDcpActionToLonLatBehavior(
    const DcpAction& action, LateralBehavior* lat,
    LongitudinalBehavior* lon) const {
  switch (action.lat) {
    case DcpLatAction::kLaneKeeping: {
      *lat = LateralBehavior::kLaneKeeping;
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      *lat = LateralBehavior::kLaneChangeLeft;
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      *lat = LateralBehavior::kLaneChangeRight;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Lateral action translation error!";
      return kWrongStatus;
    }
  }

  switch (action.lon) {
    case DcpLonAction::kMaintain: {
      *lon = LongitudinalBehavior::kMaintain;
      break;
    }
    case DcpLonAction::kAccelerate: {
      *lon = LongitudinalBehavior::kAccelerate;
      break;
    }
    case DcpLonAction::kDecelerate: {
      *lon = LongitudinalBehavior::kDecelerate;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Longitudinal action translation error!";
      return kWrongStatus;
    }
  }

  return kSuccess;
}

/**
 * @brief 归纳动作序列的整体横向模式
 * @param action_seq 动作序列
 * @param operation_at_seconds 输出：首次横向主动操作发生的时间
 * @param lat_behavior 输出：序列的主要横向行为
 * @param is_cancel_operation 输出：是否属于“换道后取消”模式
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 从完整动作序列中提取“长期横向意图”，用于场景级仿真设置。
 *
 * **判定逻辑**：
 * 1. 找到首次出现的横向主动动作（左/右换道）
 * 2. 若之后再次回到 LK，则记为 `ChangeThenCancel`
 * 3. 若全程无换道，则视为 `AlwaysLaneKeep`
 *
 * **物理意义**：
 * - 用于区分“始终保持”、“先保持后变道”、“变道后取消”等典型模式
 * - 对应文档第 4.2 节中 DCP-Tree 路径的语义解释
 */
ErrorType EudmPlanner::ClassifyActionSeq(
    const std::vector<DcpAction>& action_seq, decimal_t* operation_at_seconds,
    common::LateralBehavior* lat_behavior, bool* is_cancel_operation) const {
  decimal_t duration = 0.0;
  decimal_t operation_at = 0.0;
  bool find_lat_active_behavior = false;
  *is_cancel_operation = false;
  for (const auto& action : action_seq) {
    if (!find_lat_active_behavior) {
      if (action.lat == DcpLatAction::kLaneChangeLeft) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeLeft;
        find_lat_active_behavior = true;
      }
      if (action.lat == DcpLatAction::kLaneChangeRight) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeRight;
        find_lat_active_behavior = true;
      }
    } else {
      if (action.lat == DcpLatAction::kLaneKeeping) {
        *is_cancel_operation = true;
      }
    }
    duration += action.t;
  }
  if (!find_lat_active_behavior) {
    *operation_at_seconds = duration + cfg_.sim().duration().layer();
    *lat_behavior = common::LateralBehavior::kLaneKeeping;
    *is_cancel_operation = false;
  }
  return kSuccess;
}

/**
 * @brief 准备多线程评估结果容器
 * @param n_sequence 动作序列数量
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 为每条 DCP-Tree 路径预分配结果缓存，支持策略级并行评估。
 *
 * **容器内容**：
 * - `sim_res_ / risky_res_ / sim_info_`：仿真状态、风险标志与调试信息
 * - `progress_cost_ / tail_cost_ / final_cost_`：逐层与总评估代价
 * - `forward_trajs_ / surround_trajs_`：自车与周边车前向仿真轨迹
 * - `forward_lat_behaviors_ / forward_lon_behaviors_`：各层行为记录
 *
 * **对应文档**：
 * - 第 3 节：每条策略序列分配独立线程并行评估
 */
ErrorType EudmPlanner::PrepareMultiThreadContainers(const int n_sequence) {
  LOG(INFO) << "[Eudm][Process]Prepare multi-threading - " << n_sequence
            << " threads.";

  sim_res_.clear();
  sim_res_.resize(n_sequence, 0);

  risky_res_.clear();
  risky_res_.resize(n_sequence, 0);

  sim_info_.clear();
  sim_info_.resize(n_sequence, std::string(""));

  final_cost_.clear();
  final_cost_.resize(n_sequence, 0.0);

  progress_cost_.clear();
  progress_cost_.resize(n_sequence);

  tail_cost_.clear();
  tail_cost_.resize(n_sequence);

  forward_trajs_.clear();
  forward_trajs_.resize(n_sequence);

  forward_lat_behaviors_.clear();
  forward_lat_behaviors_.resize(n_sequence);

  forward_lon_behaviors_.clear();
  forward_lon_behaviors_.resize(n_sequence);

  surround_trajs_.clear();
  surround_trajs_.resize(n_sequence);
  return kSuccess;
}

/**
 * @brief 构建周边车辆的前向仿真代理集合
 * @param surrounding_semantic_vehicles 周边语义车辆集合
 * @param fsagents 输出：前向仿真代理集合
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 将感知/语义层输出的周边车辆，转成闭环仿真可直接使用的代理对象。
 *
 * **构建内容**：
 * 1. 复制车辆状态与车辆参数
 * 2. 设置他车纵向 IDM 参数
 * 3. 注入他车横向行为概率与当前行为
 * 4. 基于所属车道构建 Frenet 状态转换器
 *
 * **实现假设**：
 * - 若他车加速度非负，则近似保持当前速度作为期望速度
 * - 若他车减速，则按总仿真时域外推一个保守期望速度
 *
 * **对应文档**：
 * - 第 3 节：智能体意图与闭环前向仿真
 * - 第 5 节：实现细节
 */
ErrorType EudmPlanner::GetSurroundingForwardSimAgents(
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    ForwardSimAgentSet* fsagents) const {
  for (const auto& psv : surrounding_semantic_vehicles.semantic_vehicles) {
    ForwardSimAgent fsagent;

    int id = psv.second.vehicle.id();
    fsagent.id = id;
    fsagent.vehicle = psv.second.vehicle;

    // * lon
    fsagent.sim_param = agent_sim_param_;

    common::State state = psv.second.vehicle.state();
    // ~ If other vehicles' acc > 0, we assume constant velocity
    if (state.acceleration >= 0) {
      fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
    } else {
      decimal_t est_vel =
          std::max(0.0, state.velocity + state.acceleration * sim_time_total_);
      fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
    }

    // * lat
    fsagent.lat_probs = psv.second.probs_lat_behaviors;
    fsagent.lat_behavior = psv.second.lat_behavior;

    fsagent.lane = psv.second.lane;
    fsagent.stf = common::StateTransformer(fsagent.lane);

    // * other
    fsagent.lat_range = cfg_.sim().agent().cooperative_lat_range();

    fsagents->forward_sim_agents.insert(std::make_pair(id, fsagent));
  }

  return kSuccess;
}

/**
 * @brief 执行 EUDM 规划的主循环（对应论文算法 1）
 * @return ErrorType
 *
 * **流程详解**：
 * 1. **获取感知数据**：获取周边车辆的关键语义信息。
 * 2. **DCP-Tree 动作序列**：获取所有待评估的策略序列（ActionScript）。
 * 3. **并行仿真**：为每个策略序列启动一个线程进行闭环前向仿真。
 *    - 调用 SimulateActionSequence -> SimulateScenario
 * 4. **结果汇总**：收集所有线程的仿真结果（成功/失败、代价、风险等）。
 * 5. **策略评估**：根据成本函数选择最优策略（EvaluateMultiThreadSimResults）。
 */
ErrorType EudmPlanner::RunEudm() {
  // * get relevant information
  // 获取周边车辆的语义信息（Key Semantic Vehicles），包括车辆ID、状态、所属车道等
  common::SemanticVehicleSet surrounding_semantic_vehicles;
  if (map_itf_->GetKeySemanticVehicles(&surrounding_semantic_vehicles) !=
      kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get key semantic vehicles. Exit";
    return kWrongStatus;
  }

  // 构建前向仿真代理集合（Surrounding Agents）
  // 将语义车辆转换为用于前向仿真的代理对象（ForwardSimAgent），包含IDM模型参数、意图概率等
  ForwardSimAgentSet surrounding_fsagents;
  GetSurroundingForwardSimAgents(surrounding_semantic_vehicles,
                                 &surrounding_fsagents);

  // 获取 DCP-Tree 的动作序列脚本
  // action_script 包含了所有从根节点到叶子节点的行为序列（Action Sequences）
  // 
  // [Paper] DCP-Tree Complexity:
  // The number of policy sequences grows linearly with tree height `h` (planning horizon):
  //   O((|A| - 1)(h - 2) + |A|)
  // where:
  //   |A| is the number of semantic actions (e.g., 3: LK, LCL, LCR)
  //   h is the tree height
  // This avoids the exponential complexity O(|A|^h) of traditional POMDPs.
  auto action_script = dcp_tree_ptr_->action_script();
  int n_sequence = action_script.size();

  // * prepare for multi-threading
  // 准备多线程容器，用于存储每个序列的仿真结果
  std::vector<std::thread> thread_set(n_sequence);
  PrepareMultiThreadContainers(n_sequence);

  // * threading
  // TODO(@lu.zhang) Use thread pool?
  // 启动多线程进行并行仿真
  // 为每个动作序列（action_script[i]）启动一个线程调用 SimulateActionSequence
  TicToc timer;
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i] =
        std::thread(&EudmPlanner::SimulateActionSequence, this, ego_vehicle_,
                    surrounding_fsagents, action_script[i], i);
  }
  // 等待所有线程完成（Join）
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i].join();
  }

  LOG(INFO) << "[Eudm][Process]Multi-thread forward simulation finished!";

  // * finish multi-threading, summary simulation results
  bool sim_success = false;
  int num_valid_behaviors = 0;
  for (int i = 0; i < static_cast<int>(sim_res_.size()); ++i) {
    if (sim_res_[i] == 1) {
      sim_success = true;
      num_valid_behaviors++;
    }
  }

  for (int i = 0; i < n_sequence; ++i) {
    std::ostringstream line_info;
    line_info << "[Eudm][Result]" << i << " [";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << final_cost_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    if (sim_res_[i]) {
      line_info << "[Eudm][Result][e;s;n;w:";
      for (const auto& c : progress_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << c.efficiency.ego_to_desired_vel << "_"
                  << c.efficiency.leading_to_desired_vel << ";" << c.safety.rss
                  << "_" << c.safety.occu_lane << ";"
                  << c.navigation.lane_change_preference << ";" << c.weight;
        line_info << "|";
      }
      line_info << "]";
    }
    LOG(WARNING) << line_info.str();
  }
  LOG(WARNING) << "[Eudm][Result]Sim status: " << sim_success << " with "
               << num_valid_behaviors << " behaviors.";
  if (!sim_success) {
    LOG(ERROR) << "[Eudm][Fatal]Fail to find any valid behavior. Exit";
    return kWrongStatus;
  }

  // * evaluate
  // [Paper] Optimal Policy Selection:
  // Find policy π* that maximizes expected reward:
  //   π* = argmax_π E[ Σ γ^t R(s_t, a_t) | b_0, π ]
  // Here, we select the sequence with the minimum total cost (equivalent to max reward).
  if (EvaluateMultiThreadSimResults(&winner_id_, &winner_score_) != kSuccess) {
    LOG(ERROR)
        << "[Eudm][Fatal]fail to evaluate multi-thread sim results. Exit";
    return kWrongStatus;
  }
  return kSuccess;
}

/**
 * @brief 根据单个 DCP 动作刷新自车当前行为
 * @param action 当前 DCP 动作
 * @param ego_fsagent 输出：更新后的自车仿真代理
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 将当前层的语义动作同步到自车仿真代理，供单层仿真直接使用。
 *
 * **用途**：
 * - 作为 `TranslateDcpActionToLonLatBehavior()` 的轻量封装
 * - 用于保证仿真代理中的横向/纵向模式与动作树节点一致
 */
ErrorType EudmPlanner::UpdateEgoBehaviorsUsingAction(
    const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const {
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    printf("[Eudm]Translate action error\n");
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;
  return kSuccess;
}

/**
 * @brief 为特定场景更新仿真设置
 * @param action_seq 动作序列
 * @param ego_fsagent 自车仿真代理
 * @return ErrorType
 *
 * **功能**：
 * 在进入闭环场景仿真前，先根据整条动作序列设置“场景级”的长期意图。
 *
 * **设置内容**：
 * 1. **横向长期模式**：
 *    - `AlwaysLaneKeep`
 *    - `KeepThenChange`
 *    - `AlwaysLaneChange`
 *    - `ChangeThenCancel`
 * 2. **首次操作时刻**：
 *    - 记录第一次主动换道动作开始的时间
 * 3. **纵向激进度**：
 *    - 根据动作序列早期的纵向命令调整 IDM 的期望速度、最小间距和车头时距
 *
 * **物理意义**：
 * - 这一步不是在每个积分步重复计算，而是为整条策略序列设定基调
 * - 对应论文中“每条策略序列作为一个完整候选方案被评估”的思想
 *
 * **实现细节**：
 * - 当前实现使用 `action_seq[1].lon` 作为纵向 aggressiveness 的主要依据
 * - 若为加速动作，会适度提高期望速度并降低跟驰保守度
 */
ErrorType EudmPlanner::UpdateSimSetupForScenario(
    const std::vector<DcpAction>& action_seq,
    ForwardSimEgoAgent* ego_fsagent) const {
  // * Get the type of lateral action sequence
  common::LateralBehavior seq_lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  ClassifyActionSeq(action_seq, &operation_at_seconds, &seq_lat_behavior,
                    &is_cancel_behavior);
  ego_fsagent->operation_at_seconds = operation_at_seconds;
  ego_fsagent->is_cancel_behavior = is_cancel_behavior;
  ego_fsagent->seq_lat_behavior = seq_lat_behavior;

  // * get action sequence type
  if (is_cancel_behavior) {
    ego_fsagent->lat_behavior_longterm = LateralBehavior::kLaneKeeping;
    ego_fsagent->seq_lat_mode = LatSimMode::kChangeThenCancel;
  } else {
    if (seq_lat_behavior == LateralBehavior::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneKeep;
    } else if (action_seq.front().lat == DcpLatAction::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kKeepThenChange;
    } else {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneChange;
    }
    ego_fsagent->lat_behavior_longterm = seq_lat_behavior;
  }

  // * lon
  decimal_t desired_vel = std::floor(ego_fsagent->vehicle.state().velocity);
  simulator::IntelligentDriverModel::Param idm_param_tmp;
  idm_param_tmp = ego_sim_param_.idm_param;

  switch (action_seq[1].lon) {
    case DcpLonAction::kAccelerate: {
      idm_param_tmp.kDesiredVelocity = std::min(
          desired_vel + cfg_.sim().acc_cmd_vel_gap(), desired_velocity_);
      idm_param_tmp.kMinimumSpacing *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      idm_param_tmp.kDesiredHeadwayTime *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      break;
    }
    case DcpLonAction::kDecelerate: {
      idm_param_tmp.kDesiredVelocity =
          std::min(std::max(desired_vel - cfg_.sim().dec_cmd_vel_gap(), 0.0),
                   desired_velocity_);
      break;
    }
    case DcpLonAction::kMaintain: {
      idm_param_tmp.kDesiredVelocity = std::min(desired_vel, desired_velocity_);
      break;
    }
    default: {
      printf("[Eudm]Error - Lon action not valid\n");
      assert(false);
    }
  }
  ego_fsagent->sim_param = ego_sim_param_;
  ego_fsagent->sim_param.idm_param = idm_param_tmp;
  ego_fsagent->lat_range = cfg_.sim().ego().cooperative_lat_range();

  return kSuccess;
}

/**
 * @brief 更新单层（Step）的仿真设置
 * @param action 当前层的动作
 * @param other_fsagent 其他车辆代理
 * @param ego_fsagent 自车代理
 * @return ErrorType
 *
 * **功能**：
 * 在每一层动作开始时，重新构造该层固定不变的上下文信息。
 *
 * **主要步骤**：
 * 1. **动作翻译**：
 *    - 将当前层 DCP 动作翻译为横向/纵向行为
 * 2. **参考车道获取**：
 *    - 当前车道 `current_lane`
 *    - 当前层目标车道 `target_lane`
 *    - 场景级长期目标车道 `longterm_lane`
 * 3. **目标空隙搜索**：
 *    - 若为换道，搜索目标车道前后车并记录 `target_gap_ids`
 * 4. **层级严格 RSS 预检查**：
 *    - 若目标空隙显然不满足安全距离，则直接判定本层动作无效
 *
 * **为什么按层更新**：
 * - 每一层持续时间较长（例如 2s），但层内上下文可视为近似固定
 * - 这样可以在不显著增加计算量的情况下保留足够的场景响应性
 *
 * **对应文档**：
 * - 第 3 节：闭环前向仿真
 * - 第 4 节：策略序列逐层展开评估
 */
ErrorType EudmPlanner::UpdateSimSetupForLayer(
    const DcpAction& action, const ForwardSimAgentSet& other_fsagent,
    ForwardSimEgoAgent* ego_fsagent) const {
  // * action -> behavior
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;

  // * related lanes
  // 获取当前状态
  auto state = ego_fsagent->vehicle.state();
  // 计算前向搜索的参考线长度：根据当前速度动态调整，限制在 [forward_len_min, forward_len_max] 之间
  decimal_t forward_lane_len =
      std::min(std::max(state.velocity * cfg_.sim().ref_line().len_vel_coeff(),
                        cfg_.sim().ref_line().forward_len_min()),
               cfg_.sim().ref_line().forward_len_max());

  // 1. 获取当前车道（Current Lane）
  // 基于 KeepLane 行为获取当前所在车道
  common::Lane lane_current;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), LateralBehavior::kLaneKeeping,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_current) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->current_lane = lane_current;
  ego_fsagent->current_stf = common::StateTransformer(lane_current);

  // 2. 获取目标车道（Target Lane）
  // 基于当前的 lat_behavior（如左换道/右换道）获取目标车道
  common::Lane lane_target;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_target) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->target_lane = lane_target;
  ego_fsagent->target_stf = common::StateTransformer(lane_target);

  // 3. 获取长期目标车道（Long-term Lane）
  // 基于动作序列的整体意图获取长期车道
  common::Lane lane_longterm;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior_longterm,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_longterm) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->longterm_lane = lane_longterm;
  ego_fsagent->longterm_stf = common::StateTransformer(lane_longterm);

  // * Gap finding for lane-changing behaviors
  // 如果是换道行为，寻找目标车道上的前后车辆（Gap Finding）
  if (ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping) {
    common::VehicleSet other_vehicles;
    for (const auto& pv : other_fsagent.forward_sim_agents) {
      other_vehicles.vehicles.insert(
          std::make_pair(pv.first, pv.second.vehicle));
    }

    bool has_front_vehicle = false, has_rear_vehicle = false;
    common::Vehicle front_vehicle, rear_vehicle;
    common::FrenetState front_fs, rear_fs;
    // 在目标车道上搜索前车和后车
    map_itf_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ego_fsagent->target_lane, state, other_vehicles, &has_front_vehicle,
        &front_vehicle, &front_fs, &has_rear_vehicle, &rear_vehicle, &rear_fs);

    // 记录目标空隙的前后车 ID
    ego_fsagent->target_gap_ids(0) =
        has_front_vehicle ? front_vehicle.id() : -1;
    ego_fsagent->target_gap_ids(1) = has_rear_vehicle ? rear_vehicle.id() : -1;

    if (cfg_.safety().rss_for_layers_enable()) {
      // * Strict RSS check here
      // * Disable the action that is apparently not valid
      // 进行严格的 RSS 检查：如果换道后的间距不满足 RSS 安全距离，则提前认为该动作无效
      common::FrenetState ego_fs;
      if (kSuccess != ego_fsagent->target_stf.GetFrenetStateFromState(
                          ego_fsagent->vehicle.state(), &ego_fs)) {
        return kWrongStatus;
      }
      // 计算自车前后保险杠的 s 坐标
      decimal_t s_ego_fbumper = ego_fs.vec_s[0] +
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();
      decimal_t s_ego_rbumper = ego_fs.vec_s[0] -
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();

      // check safety with front vehicle on target lane
      // 检查与目标车道前车的 RSS 安全
      if (has_front_vehicle) {
        decimal_t s_front_rbumper = front_fs.vec_s[0] -
                                    front_vehicle.param().length() / 2.0 +
                                    front_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            front_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Front,
            rss_config_strict_as_rear_, &rss_dist);

        if (s_front_rbumper - s_ego_fbumper < rss_dist) {
          // violate strict RSS
          // 违反 RSS 安全距离，返回失败
          return kWrongStatus;
        }
      }

      // check safety with rear vehicle on target lane
      // 检查与目标车道后车的 RSS 安全
      if (has_rear_vehicle) {
        decimal_t s_rear_fbumper = rear_fs.vec_s[0] +
                                   rear_vehicle.param().length() / 2.0 +
                                   rear_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            rear_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Rear,
            rss_config_strict_as_front_, &rss_dist);

        if (s_ego_rbumper - s_rear_fbumper < rss_dist) {
          // violate strict RSS
          // 违反 RSS 安全距离，返回失败
          return kWrongStatus;
        }
      }
    }
  }

  return kSuccess;
}

/**
 * @brief 仿真单个场景（对应论文闭环前向仿真）
 * @return ErrorType
 *
 * **核心逻辑**：
 * 对一条“自车策略序列 + 一个场景假设”执行完整的闭环评估。
 *
 * **流程**：
 * 1. `UpdateSimSetupForScenario()`：
 *    - 基于整条动作序列设置长期横向模式与纵向 aggressiveness
 * 2. 对每一层动作循环：
 *    - `UpdateSimSetupForLayer()` 更新该层车道和目标空隙
 *    - `SimulateSingleAction()` 在更细时间步上积分传播
 *    - `StrictSafetyCheck()` 执行硬安全裁剪
 *    - 若换道提前完成，`UpdateLateralActionSequence()` 动态修正后续动作
 *    - `CostFunction()` 计算该层代价并施加折扣
 * 3. 汇总为该场景下的完整自车/周边车轨迹与逐层代价
 *
 * **对应文档**：
 * - 算法 1 第 8 行：`SimulateForward`
 * - 第 3 节：闭环前向仿真考虑交互
 *
 * **说明**：
 * - 当前实现的 `sub_seq_id` 可视为为未来 CFB 扩展预留的场景编号
 * - 即使当前默认只有 1 个场景，这一层接口仍保留了“策略-场景”二级结构
 */
ErrorType EudmPlanner::SimulateScenario(
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
        sub_surround_trajs) {
  // * declare variables which will be used to track traces from multiple layers
  // 声明用于跟踪多层（多步）仿真的轨迹变量
  // ego_traj_multilayers: 记录自车在整个仿真时长内的完整轨迹
  vec_E<common::Vehicle> ego_traj_multilayers{ego_vehicle};

  // surround_trajs_multilayers: 记录周边代理的完整轨迹
  std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multilayers;
  for (const auto& p_fsa : surrounding_fsagents.forward_sim_agents) {
    surround_trajs_multilayers.insert(std::pair<int, vec_E<common::Vehicle>>(
        p_fsa.first, vec_E<common::Vehicle>({p_fsa.second.vehicle})));
  }

  // 记录每层的行为和代价
  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructure> cost_multilayers;
  std::set<int> risky_ids_multilayers;

  // * Setup ego longitudinal sim config
  // 设置自车的初始仿真配置（如是否换道取消、激进程度等）
  ForwardSimEgoAgent ego_fsagent_this_layer;
  ego_fsagent_this_layer.vehicle = ego_vehicle;
  UpdateSimSetupForScenario(action_seq, &ego_fsagent_this_layer);

  ForwardSimAgentSet surrounding_fsagents_this_layer = surrounding_fsagents;

  int action_ref_lane_id = ego_lane_id_;
  bool is_sub_seq_risky = false;
  std::vector<DcpAction> action_seq_sim = action_seq;

  // * For each action in action sequence
  // 遍历动作序列中的每一个动作（Layer）
  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    auto action_this_layer = action_seq_sim[i];

    // For each action, we can update context here.
    // such as lane, stf, desired_vel, social_force_masks
    // For each action, the context info will not change, so we can use it in
    // every step. A low-level reactive lane-changing controller can be
    // implemented without a lot of computation cost.
    // * update setup for this layer
    // 为当前层更新仿真设置
    if (kSuccess != UpdateSimSetupForLayer(action_this_layer,
                                           surrounding_fsagents_this_layer,
                                           &ego_fsagent_this_layer)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      return kWrongStatus;
    }

    // TODO(lu.zhang): MOBIL/RSS check here?
    // * Disable the action that is apparently not valid
    // if (ego_lat_behavior_this_layer != LateralBehavior::kLaneKeeping) {
    //   if (!CheckLaneChangingFeasibilityUsingMobil(
    //           ego_semantic_vehicle_this_layer,
    //           semantic_vehicle_set_this_layer)) {
    //     return kWrongStatus;
    //   }
    // }

    // * simulate this action (layer)
    // 执行单层前向仿真
    vec_E<common::Vehicle> ego_traj_multisteps;
    std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multisteps;
    if (SimulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                             surrounding_fsagents_this_layer,
                             &ego_traj_multisteps,
                             &surround_trajs_multisteps) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      return kWrongStatus;
    }

    // * update ForwardSimAgent
    ego_fsagent_this_layer.vehicle.set_state(
        ego_traj_multisteps.back().state());
    for (auto it = surrounding_fsagents_this_layer.forward_sim_agents.begin();
         it != surrounding_fsagents_this_layer.forward_sim_agents.end(); ++it) {
      it->second.vehicle.set_state(
          surround_trajs_multisteps.at(it->first).back().state());
    }

    // * enforce strict safety check
    // 执行严格的安全检查（碰撞检测和 RSS）
    bool is_strictly_safe = false;
    int collided_id = 0;
    TicToc timer;
    if (StrictSafetyCheck(ego_traj_multisteps, surround_trajs_multisteps,
                          &is_strictly_safe, &collided_id) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Check F)");
      return kWrongStatus;
    }
    // LOG(INFO) << "[RssTime]safety check time per action: " << timer.toc();

    if (!is_strictly_safe) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Strict F:") +
                                     std::to_string(collided_id) +
                                     std::string(")");
      return kWrongStatus;
    }

    // * If lateral action finished, update simulation action sequence
    // 检查横向动作是否完成（例如换道完成）
    // 如果完成，将后续动作序列中的换道动作更新为车道保持
    int current_lane_id;
    if (CheckIfLateralActionFinished(
            ego_fsagent_this_layer.vehicle.state(), action_ref_lane_id,
            ego_fsagent_this_layer.lat_behavior, &current_lane_id)) {
      action_ref_lane_id = current_lane_id;
      if (kSuccess != UpdateLateralActionSequence(i, &action_seq_sim)) {
        (*sub_sim_res)[sub_seq_id] = 0;
        (*sub_sim_info)[sub_seq_id] += std::string("(Update Lat F)");
        return kWrongStatus;
      }
      ego_fsagent_this_layer.lat_behavior_longterm =
          LateralBehavior::kLaneKeeping;
    }

    // * trace
    // 累积轨迹数据到多层轨迹容器中
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.push_back(
        ego_fsagent_this_layer.lat_behavior);
    ego_lon_behavior_multilayers.push_back(
        ego_fsagent_this_layer.lon_behavior);

    for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
      int id = v.first;
      surround_trajs_multilayers.at(id).insert(
          surround_trajs_multilayers.at(id).end(),
          surround_trajs_multisteps.at(id).begin(),
          surround_trajs_multisteps.at(id).end());
    }

    CostStructure cost;
    bool verbose = false;
    std::set<int> risky_ids;
    bool is_risky_action = false;
    // 计算代价函数（安全、效率、舒适性）
    CostFunction(action_this_layer, ego_fsagent_this_layer,
                 surrounding_fsagents_this_layer, ego_traj_multisteps,
                 surround_trajs_multisteps, verbose, &cost, &is_risky_action,
                 &risky_ids);
    if (is_risky_action) {
      is_sub_seq_risky = true;
      for (const auto& id : risky_ids) {
        risky_ids_multilayers.insert(id);
      }
    }
    cost.weight = cost.weight * pow(cfg_.cost().discount_factor(), i);
    cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_multilayers.push_back(cost);
  }

  (*sub_sim_res)[sub_seq_id] = 1;
  (*sub_risky_res)[sub_seq_id] = is_sub_seq_risky ? 1 : 0;
  if (is_sub_seq_risky) {
    std::string risky_id_list;
    for (auto it = risky_ids_multilayers.begin();
         it != risky_ids_multilayers.end(); it++) {
      risky_id_list += " " + std::to_string(*it);
    }
    (*sub_sim_info)[sub_seq_id] += std::string("(Risky)") + risky_id_list;
  }
  (*sub_progress_cost)[sub_seq_id] = cost_multilayers;
  (*sub_forward_trajs)[sub_seq_id] = ego_traj_multilayers;
  (*sub_forward_lat_behaviors)[sub_seq_id] = ego_lat_behavior_multilayers;
  (*sub_forward_lon_behaviors)[sub_seq_id] = ego_lon_behavior_multilayers;
  (*sub_surround_trajs)[sub_seq_id] = surround_trajs_multilayers;

  return kSuccess;
}

/**
 * @brief 仿真单个动作序列（Action Sequence）
 * @param ego_vehicle 自车信息
 * @param surrounding_fsagents 周边车辆代理
 * @param action_seq 动作序列
 * @param seq_id 序列 ID
 * @return ErrorType
 *
 * **功能**：
 * 这是策略级并行评估的入口，对应 DCP-Tree 中一条根到叶路径的求值。
 *
 * **CFB（条件聚焦分支）关系**：
 * - 按论文，单条自车策略序列应先经过 CFB 选出关键场景集合 `Ω`
 * - 随后对每个场景并行执行闭环仿真，再汇总结果
 * - 当前代码中 `n_sub_threads = 1`，表示默认只评估一个基础场景
 * - 但函数接口已经保留了未来扩展到多场景分支的结构
 *
 * **当前实现含义**：
 * - 策略级并行已实现
 * - 场景级分支目前是简化版本，属于对论文完整 CFB 的工程裁剪
 */
ErrorType EudmPlanner::SimulateActionSequence(
    const common::Vehicle& ego_vehicle,
    const ForwardSimAgentSet& surrounding_fsagents,
    const std::vector<DcpAction>& action_seq, const int& seq_id) {
  if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = std::string("(Pre-deleted)");
    return kWrongStatus;
  }

  // ~ For each ego sequence, we may further branch here, which will create
  // ~ multiple sub threads. Currently, we use n_sub_threads = 1
  // TODO(@lu.zhang) Preliminary safety assessment here
  int n_sub_threads = 1;

  std::vector<int> sub_sim_res(n_sub_threads);
  std::vector<int> sub_risky_res(n_sub_threads);
  std::vector<std::string> sub_sim_info(n_sub_threads);
  std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
  std::vector<CostStructure> sub_tail_cost(n_sub_threads);
  vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
      n_sub_threads);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
      n_sub_threads);
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(
      n_sub_threads);

  SimulateScenario(ego_vehicle, surrounding_fsagents, action_seq, seq_id, 0,
                   &sub_sim_res, &sub_risky_res, &sub_sim_info,
                   &sub_progress_cost, &sub_tail_cost, &sub_forward_trajs,
                   &sub_forward_lat_behaviors, &sub_forward_lon_behaviors,
                   &sub_surround_trajs);

  if (sub_sim_res.front() == 0) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = sub_sim_info.front();
    return kWrongStatus;
  }

  // ~ Here use the default scenario
  sim_res_[seq_id] = 1;
  risky_res_[seq_id] = sub_risky_res.front();
  sim_info_[seq_id] = sub_sim_info.front();
  progress_cost_[seq_id] = sub_progress_cost.front();
  tail_cost_[seq_id] = sub_tail_cost.front();
  forward_trajs_[seq_id] = sub_forward_trajs.front();
  forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors.front();
  forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors.front();
  surround_trajs_[seq_id] = sub_surround_trajs.front();

  return kSuccess;
}

/**
 * @brief 更新横向动作序列
 * @param cur_idx 当前动作索引
 * @param action_seq 动作序列指针
 * @return ErrorType
 *
 * **功能**：
 * 当换道动作在某一层提前完成时，动态修正后续动作序列，保持语义一致。
 *
 * **典型规则**：
 * - `LLLLL -> LLKKK`
 *   当前层左换道已完成，则后续继续左换道改为车道保持
 * - `LLKKK -> LLRRR`
 *   原本相对旧参考系的保持，在新车道参考系下可能变成“返回动作”
 * - 右换道情形同理对称处理
 *
 * **设计原因**：
 * - DCP-Tree 的动作持续时间是粗粒度的
 * - 真实闭环仿真中，换道可能早于该层结束就已完成
 * - 若不修正，后续动作的参考车道语义会失真
 *
 * **注意**：
 * - 这是一个工程上的一致性修正机制
 * - 某些“换完又立刻反向换回”的模式仍会被判为无效
 */
ErrorType EudmPlanner::UpdateLateralActionSequence(
    const int cur_idx, std::vector<DcpAction>* action_seq) const {
  if (cur_idx == static_cast<int>(action_seq->size()) - 1) {
    return kSuccess;
  }

  switch ((*action_seq)[cur_idx].lat) {
    case DcpLatAction::kLaneKeeping: {
      // * no need to update
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // * LLLLL -> LLKKK
          // 如果当前是 LCL 且已完成，后续的 LCL 应变为 LK（保持在目标车道）
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // * LLKKK -> LLRRR
          // 如果后续是 LK，则意味着要在目标车道保持。
          // 注意：如果原计划回到原车道（LCR），这里需要根据实际逻辑调整，
          // 当前简单处理为如果原本是 LK，现在变为 LCR 似乎有点奇怪？
          // 实际上这里的逻辑是：如果原计划是 LK（在原车道保持），
          // 但现在已经换到了左车道，那么在新车道上的 LK 就变成了“相对于原车道的右换道”才能回去？
          // 或者说，这里的逻辑是修正后续动作以匹配新的车道参考系。
          // 代码中的逻辑：LLKKK (原计划左换后保持) -> LLRRR (现在已经在左边了，要保持的话，相对于新车道是 LK，
          // 但相对于原参考线...?)
          // 实际上，ActionSeq 是基于当前时刻的 Ego 状态生成的吗？
          // 在 RunEudm 中，ActionScript 是固定的。
          // 这里的 update 是为了在仿真过程中动态调整后续动作。
          // 如果完成了 LCL，参考车道ID 变了（action_ref_lane_id updated）。
          // 这里的修改似乎是为了应对 “提前完成换道” 的情况。
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeRight;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // * LLRRR -> x
          return kWrongStatus;
        }
      }
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // * RRRRR -> RRKKK
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // * RRKKK -> RRLLL
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeLeft;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // * RRLLL -> x
          return kWrongStatus;
        }
      }
      break;
    }

    default: {
      std::cout << "[Eudm]Error - Invalid lateral behavior" << std::endl;
      assert(false);
    }
  }

  return kSuccess;
}

/**
 * @brief 判断当前横向动作是否已经完成
 * @param cur_state 当前车辆状态
 * @param action_ref_lane_id 当前动作所基于的参考车道 id
 * @param lat_behavior 当前横向行为
 * @param current_lane_id 输出：车辆当前最近车道 id
 * @return 若动作完成返回 true，否则返回 false
 *
 * **功能**：
 * 通过“当前实际所在车道”与“本次换道期望到达的候选车道集合”比较，
 * 判断横向动作是否已经落地完成。
 *
 * **判定逻辑**：
 * - 若当前行为是 `LaneKeeping`，直接返回未完成
 * - 若当前行为是换道，则：
 *   1. 查询当前最近车道 id
 *   2. 根据 `action_ref_lane_id + lat_behavior` 生成可能目标车道集合
 *   3. 若当前最近车道属于该集合，则认为换道完成
 *
 * **用途**：
 * - 触发 `UpdateLateralActionSequence()`
 * - 避免在已经进入目标车道后仍沿用旧的换道语义
 */
bool EudmPlanner::CheckIfLateralActionFinished(
    const common::State& cur_state, const int& action_ref_lane_id,
    const LateralBehavior& lat_behavior, int* current_lane_id) const {
  if (lat_behavior == LateralBehavior::kLaneKeeping) {
    return false;
  }

  decimal_t current_lane_dist;
  decimal_t arc_len;
  map_itf_->GetNearestLaneIdUsingState(cur_state.ToXYTheta(),
                                       std::vector<int>(), current_lane_id,
                                       &current_lane_dist, &arc_len);
  // std::cout << "[Eudm] current_lane_id: " << *current_lane_id << std::endl;
  // std::cout << "[Eudm] action_ref_lane_id: " << action_ref_lane_id <<
  // std::endl;

  std::vector<int> potential_lane_ids;
  GetPotentialLaneIds(action_ref_lane_id, lat_behavior, &potential_lane_ids);

  // ~ Lane change
  auto it = std::find(potential_lane_ids.begin(), potential_lane_ids.end(),
                      *current_lane_id);
  if (it == potential_lane_ids.end()) {
    return false;
  } else {
    return true;
  }
}

/**
 * @brief 执行一次完整的 EUDM 规划周期
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **主流程**（对应文档算法 1）：
 * 1. 获取自车状态、车道与 RSS 参考线
 * 2. 依据当前道路规则预裁剪明显不合理的动作序列
 * 3. 调用 `RunEudm()` 并行评估全部策略序列
 * 4. 记录最优策略编号、总代价与时间消耗
 *
 * **预裁剪内容**：
 * - 去除动作序列中相邻层直接“左后立刻右”或“右后立刻左”的模式
 * - 这与文档第 4.2 节中“单周期内最多一次动作变化”的设计思想一致
 *
 * **输出结果**：
 * - `winner_id_ / winner_score_`：最优策略序列及其总代价
 * - `forward_trajs_ / surround_trajs_`：供下游模块使用的离散仿真结果
 *
 * **对应文档**：
 * - 第 3 节：系统概述
 * - 第 4 节：通过引导分支进行决策
 */
ErrorType EudmPlanner::RunOnce() {
  TicToc timer_runonce;
  // * Get current nearest lane id
  if (!map_itf_) {
    LOG(ERROR) << "[Eudm]map interface not initialized. Exit";
    return kWrongStatus;
  }

  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Eudm]no ego vehicle found.";
    return kWrongStatus;
  }
  ego_id_ = ego_vehicle_.id();
  time_stamp_ = ego_vehicle_.state().time_stamp;

  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]------ Eudm Cycle Begins (stamp): " << time_stamp_
               << " ------- ";

  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(std::vector<int>(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    LOG(ERROR) << "[Eudm]Fatal (Exit) ego not on lane.";
    return kWrongStatus;
  }
  LOG(WARNING) << std::fixed << std::setprecision(3)
               << "[Eudm][Input]Ego plan state (x,y,theta,v,a,k):("
               << ego_vehicle_.state().vec_position[0] << ","
               << ego_vehicle_.state().vec_position[1] << ","
               << ego_vehicle_.state().angle << ","
               << ego_vehicle_.state().velocity << ","
               << ego_vehicle_.state().acceleration << ","
               << ego_vehicle_.state().curvature << ")"
               << " lane id:" << ego_lane_id_by_pos;
  LOG(WARNING) << "[Eudm][Setup]Desired vel:" << desired_velocity_
               << " sim_time total:" << sim_time_total_
               << " lc info[f_l,f_r,us_ol,us_or,solid_l,solid_r]:"
               << lc_info_.forbid_lane_change_left << ","
               << lc_info_.forbid_lane_change_right << ","
               << lc_info_.lane_change_left_unsafe_by_occu << ","
               << lc_info_.lane_change_right_unsafe_by_occu << ","
               << lc_info_.left_solid_lane << "," << lc_info_.right_solid_lane;
  ego_lane_id_ = ego_lane_id_by_pos;

  const decimal_t forward_rss_check_range = 130.0;
  const decimal_t backward_rss_check_range = 130.0;
  const decimal_t forward_lane_len = forward_rss_check_range;
  const decimal_t backward_lane_len = backward_rss_check_range;
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_.state(), std::vector<int>(),
          LateralBehavior::kLaneKeeping, forward_lane_len, backward_lane_len,
          false, &rss_lane_) != kSuccess) {
    LOG(ERROR) << "[Eudm]No Rss lane available. Rss disabled";
  }

  if (rss_lane_.IsValid()) {
    rss_stf_ = common::StateTransformer(rss_lane_);
  }

  pre_deleted_seq_ids_.clear();
  int n_sequence = dcp_tree_ptr_->action_script().size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    int num_actions = action_seq.size();
    for (int j = 1; j < num_actions; j++) {
      if ((action_seq[j - 1].lat == DcpLatAction::kLaneChangeLeft &&
           action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)) {
        pre_deleted_seq_ids_.insert(i);
      }
    }
  }

  TicToc timer;
  if (RunEudm() != kSuccess) {
    LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]****** Eudm Cycle FAILED (stamp): " << time_stamp_
               << " time cost " << timer.toc() << " ms.";
    return kWrongStatus;
  }
  auto action_script = dcp_tree_ptr_->action_script();
  std::ostringstream line_info;
  line_info << "[Eudm]SUCCESS id:" << winner_id_ << " [";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLonActionName(a.lon);
  }
  line_info << "|";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3) << winner_score_
            << " time cost: " << timer.toc() << " ms.";
  LOG(WARNING) << line_info.str();

  time_cost_ = timer_runonce.toc();
  return kSuccess;
}

/**
 * @brief 计算单条策略序列的总代价
 * @param progress_cost 各层代价
 * @param tail_cost 尾部代价
 * @param action_seq 对应动作序列
 * @param score 输出：总代价
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 将每一层的折扣后代价累加，得到整条策略序列的评估值。
 *
 * **与论文的关系**：
 * - 论文中以“期望折扣奖励最大化”描述策略选择
 * - 当前实现采用“总代价最小化”，本质上等价于对奖励取负后求最优
 */
ErrorType EudmPlanner::EvaluateSinglePolicyTrajs(
    const std::vector<CostStructure>& progress_cost,
    const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
    decimal_t* score) {
  decimal_t score_tmp = 0.0;
  for (const auto& c : progress_cost) {
    score_tmp += c.ave();
  }
  *score = score_tmp + tail_cost.ave();
  return kSuccess;
}

/**
 * @brief 从多线程仿真结果中选出最优策略
 * @param winner_id 输出：最优策略编号
 * @param winner_cost 输出：最优策略总代价
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 遍历所有有效策略序列，计算总代价，并选择总代价最小的候选。
 *
 * **选择准则**：
 * - `sim_res_[i] == 0` 的序列视为无效，直接跳过
 * - 对剩余序列调用 `EvaluateSinglePolicyTrajs()` 评估
 * - 取总代价最小者作为本周期最优策略
 *
 * **对应文档**：
 * - 算法 1 第 12 行：`SelectPolicy`
 * - 第 4.1 节公式 (2)：期望折扣奖励最优
 */
ErrorType EudmPlanner::EvaluateMultiThreadSimResults(int* winner_id,
                                                     decimal_t* winner_cost) {
  decimal_t min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();
  for (int i = 0; i < num_sequences; ++i) {
    if (sim_res_[i] == 0) {
      continue;
    }
    decimal_t cost = 0.0;
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    EvaluateSinglePolicyTrajs(progress_cost_[i], tail_cost_[i], action_seq,
                              &cost);
    final_cost_[i] = cost;
    if (cost < min_cost) {
      min_cost = cost;
      best_id = i;
    }
  }
  *winner_cost = min_cost;
  *winner_id = best_id;
  return kSuccess;
}

/**
 * @brief 评估两条轨迹之间的 RSS 安全状态与风险代价
 * @param traj_a 自车轨迹
 * @param traj_b 他车轨迹
 * @param cost 输出：RSS 风险代价
 * @param is_rss_safe 输出：是否满足 RSS
 * @param risky_id 输出：风险车辆 id
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 沿时间采样两条离散轨迹，执行 RSS 检查，并将违规程度映射为安全代价。
 *
 * **代价设计**：
 * - `TooFast`：自车相对 RSS 上界过快，增加超速型风险代价
 * - `TooSlow`：自车相对 RSS 下界过慢，增加缺速型风险代价
 *
 * **用途**：
 * - `CostFunction()` 中的安全项
 * - 用于识别“风险动作”而非立即淘汰策略
 *
 * **对应文档**：
 * - 第 3 节：风险场景评估
 * - 第 5 节：代价函数中的安全项
 */
ErrorType EudmPlanner::EvaluateSafetyStatus(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost, bool* is_rss_safe, int* risky_id) {
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }
  if (!cfg_.safety().rss_check_enable() || !rss_lane_.IsValid()) {
    return kSuccess;
  }
  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;
  bool ret_is_rss_safe = true;
  const int check_per_state = 1;
  for (int i = 0; i < num_states; i += check_per_state) {
    bool is_rss_safe = true;
    common::RssChecker::LongitudinalViolateType type;
    decimal_t rss_vel_low, rss_vel_up;
    common::RssChecker::RssCheck(traj_a[i], traj_b[i], rss_stf_, rss_config_,
                                 &is_rss_safe, &type, &rss_vel_low,
                                 &rss_vel_up);
    if (!is_rss_safe) {
      ret_is_rss_safe = false;
      *risky_id = traj_b.size() ? traj_b[0].id() : 0;
      if (cfg_.cost().safety().rss_cost_enable()) {
        if (type == common::RssChecker::LongitudinalViolateType::TooFast) {
          cost_tmp +=
              cfg_.cost().safety().rss_over_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_over_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_up));
        } else if (type ==
                   common::RssChecker::LongitudinalViolateType::TooSlow) {
          cost_tmp +=
              cfg_.cost().safety().rss_lack_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_lack_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_low));
        }
      }
    }
  }
  *is_rss_safe = ret_is_rss_safe;
  *cost = cost_tmp;
  return kSuccess;
}

/**
 * @brief 执行严格安全检查
 * @param ego_traj 自车离散轨迹
 * @param surround_trajs 周边车辆离散轨迹
 * @param is_safe 输出：是否严格安全
 * @param collided_id 输出：若碰撞则返回对方车辆 id
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 对闭环前向仿真结果做硬安全校验，和代价中的“软风险惩罚”相区分。
 *
 * **检查逻辑**：
 * 1. 对每辆周边车，确认轨迹长度与自车一致
 * 2. 使用膨胀后的车辆包络进行逐时刻碰撞检测
 * 3. 一旦发生碰撞，立即判定该策略不可行
 *
 * **与论文的关系**：
 * - 对应 EUDM 中对候选策略进行闭环安全验证的实现部分
 * - 当前实现以几何碰撞为主，是“必须满足”的硬约束
 */
ErrorType EudmPlanner::StrictSafetyCheck(
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool* is_safe, int* collided_id) {
  if (!cfg_.safety().strict_check_enable()) {
    *is_safe = true;
    return kSuccess;
  }

  int num_points_ego = ego_traj.size();
  if (num_points_ego == 0) {
    *is_safe = true;
    return kSuccess;
  }
  // strict collision check
  for (auto it = surround_trajs.begin(); it != surround_trajs.end(); it++) {
    int num_points_other = it->second.size();
    if (num_points_other != num_points_ego) {
      *is_safe = false;
      LOG(ERROR) << "[Eudm]unsafe due to incomplete sim record for vehicle";
      return kSuccess;
    }
    for (int i = 0; i < num_points_ego; i++) {
      common::Vehicle inflated_a, inflated_b;
      common::SemanticsUtils::InflateVehicleBySize(
          ego_traj[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_a);
      common::SemanticsUtils::InflateVehicleBySize(
          it->second[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_b);
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                         inflated_b.param(), inflated_b.state(),
                                         &is_collision);
      if (is_collision) {
        *is_safe = false;
        *collided_id = it->second[i].id();
        return kSuccess;
      }
    }
  }
  *is_safe = true;
  return kSuccess;
}

/**
 * @brief 计算单层动作的代价结构
 * @param action 当前层动作
 * @param ego_fsagent 当前层自车仿真代理
 * @param other_fsagent 当前层周边车仿真代理集合
 * @param ego_traj 当前层自车轨迹
 * @param surround_trajs 当前层周边车轨迹
 * @param verbose 是否输出详细日志
 * @param cost 输出：该层代价结构
 * @param is_risky 输出：该层是否存在风险
 * @param risky_ids 输出：风险车辆集合
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **代价组成**（对应文档第 5 节奖励/代价实现）：
 * 1. **效率代价**：
 *    - 自车与期望速度偏差
 *    - 被前车阻挡时的效率损失
 * 2. **安全代价**：
 *    - RSS 违规风险
 *    - 违反占道/禁变道规则的惩罚
 * 3. **导航代价**：
 *    - 换道本身的操作代价
 *    - 推荐换道奖励
 *    - 延迟操作与取消操作惩罚
 *
 * **实现特点**：
 * - `cost.weight = action.t`：每层动作持续时间作为该层代价权重
 * - 外层会再乘折扣因子，形成类似论文中的折扣评估
 *
 * **对应文档**：
 * - 第 3 节：所有场景输入到成本评估模块
 * - 第 5.5 节：奖励函数设计
 */
ErrorType EudmPlanner::CostFunction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
    const ForwardSimAgentSet& other_fsagent,
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool verbose, CostStructure* cost, bool* is_risky,
    std::set<int>* risky_ids) {
  decimal_t duration = action.t;

  auto ego_lon_behavior_this_layer = ego_fsagent.lon_behavior;
  auto ego_lat_behavior_this_layer = ego_fsagent.lat_behavior;

  auto seq_lat_behavior = ego_fsagent.seq_lat_behavior;
  auto is_cancel_behavior = ego_fsagent.is_cancel_behavior;

  common::VehicleSet vehicle_set;
  for (const auto& v : other_fsagent.forward_sim_agents) {
    vehicle_set.vehicles.insert(std::make_pair(v.first, v.second.vehicle));
  }

  decimal_t ego_velocity = ego_fsagent.vehicle.state().velocity;
  // f = c1 * fabs(v_ego - v_user), if v_ego < v_user
  // f = c2 * fabs(v_ego - v_user - vth), if v_ego > v_user + vth
  // unit of this cost is velocity (finally multiplied by duration)
  // f = c1 * fabs(v_ego - v_user), if v_ego < v_user
  // f = c2 * fabs(v_ego - v_user - vth), if v_ego > v_user + vth
  // unit of this cost is velocity (finally multiplied by duration)
  // 1. 效率代价（Efficiency Cost）：与期望速度的偏差
  // [Paper] Efficiency Cost Formula:
  // f_eff = w_eff * |v_ego - v_desired|
  // Penalizes deviation from the desired velocity.
  CostStructure cost_tmp;
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_) {
    // 速度低于期望速度，惩罚 'Lack Speed'
    cost_tmp.efficiency.ego_to_desired_vel =
        cfg_.cost().effciency().ego_lack_speed_to_desired_unit_cost() *
        fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_);
  } else {
    // 速度高于期望速度（包括容忍阈值），惩罚 'Over Speed'
    if (ego_fsagent.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost().effciency().ego_desired_speed_tolerate_gap()) {
      cost_tmp.efficiency.ego_to_desired_vel =
          cfg_.cost().effciency().ego_over_speed_to_desired_unit_cost() *
          fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost().effciency().ego_desired_speed_tolerate_gap());
    }
  }

  // f = ratio * c1 * fabs(v_ego - v_user) , if v_ego < v_user && v_ego
  // > v_leading
  // unit of this cost is velocity (finally multiplied by duration)
  // 效率代价：被前车阻挡导致的减速
  common::Vehicle leading_vehicle;
  decimal_t distance_residual_ratio = 0.0;
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_fsagent.target_lane, ego_fsagent.vehicle.state(), vehicle_set,
          ego_fsagent.lat_range, &leading_vehicle,
          &distance_residual_ratio) == kSuccess) {
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_fsagent.vehicle.state().vec_position)
            .norm();
    // 如果自车和前车都低于期望速度，且距离较近
    if (ego_fsagent.vehicle.state().velocity < desired_velocity_ &&
        leading_vehicle.state().velocity < desired_velocity_ &&
        distance_to_leading_vehicle <
            cfg_.cost().effciency().leading_distance_th()) {
      decimal_t ego_blocked_by_leading_velocity =
          ego_fsagent.vehicle.state().velocity >
                  leading_vehicle.state().velocity
              ? ego_fsagent.vehicle.state().velocity -
                    leading_vehicle.state().velocity
              : 0.0;
      decimal_t leading_to_desired_velocity =
          leading_vehicle.state().velocity < desired_velocity_
              ? desired_velocity_ - leading_vehicle.state().velocity
              : 0.0;
      cost_tmp.efficiency.leading_to_desired_vel =
          std::max(cfg_.cost().effciency().min_distance_ratio(),
                   distance_residual_ratio) *
          (cfg_.cost().effciency().ego_speed_blocked_by_leading_unit_cost() *
               ego_blocked_by_leading_velocity +
           cfg_.cost()
                   .effciency()
                   .leading_speed_blocked_desired_vel_unit_cost() *
               leading_to_desired_velocity);
    }
  }

  // * safety
  // 2. 安全代价（Safety Cost）
  // [Paper] Safety Cost Formula:
  // f_safety = w_rss * \sum (RSS_violations)
  // Evaluates safety using Responsibility-Sensitive Safety (RSS) rules.
  for (const auto& surround_traj : surround_trajs) {
    decimal_t safety_cost = 0.0;
    bool is_safe = true;
    int risky_id = 0;
    // 评估 RSS 安全性，包括纵向和横向风险
    EvaluateSafetyStatus(ego_traj, surround_traj.second, &safety_cost, &is_safe,
                         &risky_id);
    if (!is_safe) {
      risky_ids->insert(risky_id);
      *is_risky = true;
    }
    cost_tmp.safety.rss += safety_cost;
  }

  if (cfg_.cost().safety().occu_lane_enable()) {
    if (lc_info_.forbid_lane_change_left &&
        seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    } else if (lc_info_.forbid_lane_change_right &&
               seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    }
  }

  // * navigation
  // 3. 导航代价（Navigation Cost）：偏好惩罚或奖励
  // [Paper] Navigation Cost Formula:
  // f_nav = C_nav * I(action)
  // where I(action) is an indicator function for lane changes and preferences.
  if (seq_lat_behavior == LateralBehavior::kLaneChangeLeft ||
      seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
    if (is_cancel_behavior) {
      // 换道取消（ChangeThenCancel）的代价
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          cfg_.cost().user().cancel_operation_unit_cost();
    } else {
      // 正常换道的代价（默认惩罚换道，鼓励少换道）
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          (seq_lat_behavior == LateralBehavior::kLaneChangeLeft
               ? cfg_.cost().navigation().lane_change_left_unit_cost()
               : cfg_.cost().navigation().lane_change_right_unit_cost());
      
      // 如果推荐换道（recommend_lc），则给予奖励（减少代价）
      if (lc_info_.recommend_lc_left &&
          seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_left_recommendation_reward();
        // 如果虽然推荐换道，但当前步骤没有换道动作，则增加“延迟操作”的代价
        if (action.lat != DcpLatAction::kLaneChangeLeft) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      } else if (lc_info_.recommend_lc_right &&
                 seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_right_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeRight) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      }
    }
  }
  cost_tmp.weight = duration;
  *cost = cost_tmp;
  return kSuccess;
}

/**
 * @brief 将动作持续时间离散为仿真时间步
 * @param action 当前动作
 * @param dt_steps 输出：离散时间步序列
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 将单层动作的持续时间 `action.t` 按固定分辨率拆分为多个积分步长。
 *
 * **设计原因**：
 * - DCP-Tree 的节点持续时间通常较长（如 2s）
 * - 闭环仿真需要更细的积分粒度（如 0.1s / 0.2s）才能稳定传播系统状态
 *
 * **对应文档**：
 * - 第 3 节：闭环前向仿真输出离散状态序列
 */
ErrorType EudmPlanner::GetSimTimeSteps(const DcpAction& action,
                                       std::vector<decimal_t>* dt_steps) const {
  decimal_t sim_time_resolution = cfg_.sim().duration().step();
  decimal_t sim_time_total = action.t;
  int n_1 = std::floor(sim_time_total / sim_time_resolution);
  decimal_t dt_remain = sim_time_total - n_1 * sim_time_resolution;
  std::vector<decimal_t> steps(n_1, sim_time_resolution);
  if (fabs(dt_remain) > kEPS) {
    steps.insert(steps.begin(), dt_remain);
  }
  *dt_steps = steps;

  return kSuccess;
}

/**
 * @brief 对单层动作执行闭环前向仿真
 * @param action 当前层动作
 * @param ego_fsagent_this_layer 当前层自车代理
 * @param surrounding_fsagents_this_layer 当前层周边车代理集合
 * @param ego_traj 输出：自车在该层内的离散轨迹
 * @param surround_trajs 输出：周边车在该层内的离散轨迹
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 这是 EUDM 闭环仿真的最小执行单元，对应“一层动作在多个积分步上的展开”。
 *
 * **流程**：
 * 1. 根据 `action.t` 生成积分时间步
 * 2. 每个时间步先传播自车，再传播周边车
 * 3. 缓存并回写所有车辆状态，形成闭环交互
 *
 * **闭环含义**：
 * - 当前步的所有传播都基于上一步更新后的车辆状态
 * - 因而能反映策略与周边车辆之间的交互效应
 *
 * **对应文档**：
 * - 第 3 节：闭环前向仿真
 * - 算法 1 第 8 行：`SimulateForward`
 */
ErrorType EudmPlanner::SimulateSingleAction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
    const ForwardSimAgentSet& surrounding_fsagents_this_layer,
    vec_E<common::Vehicle>* ego_traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // ~ Prepare containers
  // 准备轨迹容器：ego_traj 和 surround_trajs
  ego_traj->clear();
  surround_trajs->clear();
  for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
  }

  // ~ Simulation time steps
  // 将当前动作的总时间 action.t 离散化为多个微小时间步 dt_steps (e.g., 0.1s, 0.1s, ...)
  std::vector<decimal_t> dt_steps;
  GetSimTimeSteps(action, &dt_steps);

  ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;
  ForwardSimAgentSet surrounding_fsagents_this_step =
      surrounding_fsagents_this_layer;

  // 逐步积分（Integration Loop）
  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    decimal_t sim_time_step = dt_steps[i];

    common::State ego_state_cache_this_step;
    std::unordered_map<int, State>
        others_state_cache_this_step;  // id - state_output

    // 构建所有参与仿真的车辆集合（包含自车和他车）
    common::VehicleSet all_sim_vehicles;  // include ego vehicle
    all_sim_vehicles.vehicles.insert(std::make_pair(
        ego_fsagent_this_step.vehicle.id(), ego_fsagent_this_step.vehicle));
    for (const auto& v : surrounding_fsagents_this_step.forward_sim_agents) {
      all_sim_vehicles.vehicles.insert(
          std::make_pair(v.first, v.second.vehicle));
    }

    // * For ego agent
    // 1. 自车前向仿真（动力学/运动学更新）
    // 调用 EgoAgentForwardSim 计算自车在当前时间步的下一个状态
    {
      all_sim_vehicles.vehicles.at(ego_id_).set_id(kInvalidAgentId);

      common::State state_output;
      if (kSuccess != EgoAgentForwardSim(ego_fsagent_this_step,
                                         all_sim_vehicles, sim_time_step,
                                         &state_output)) {
        return kWrongStatus;
      }

      common::Vehicle v_tmp = ego_fsagent_this_step.vehicle;
      v_tmp.set_state(state_output);
      ego_traj->push_back(v_tmp);
      ego_state_cache_this_step = state_output;

      all_sim_vehicles.vehicles.at(ego_id_).set_id(ego_id_);
    }

    // * For surrounding agents
    // 2. 他车前向仿真
    // 遍历所有他车，调用 SurroundingAgentForwardSim 计算其下一个状态（通常基于 IDM 或匀速模型）
    {
      for (const auto& p_fsa :
           surrounding_fsagents_this_step.forward_sim_agents) {
        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(kInvalidAgentId);

        common::State state_output;
        if (kSuccess !=
            SurroundingAgentForwardSim(p_fsa.second, all_sim_vehicles,
                                       sim_time_step, &state_output)) {
          return kWrongStatus;
        }

        common::Vehicle v_tmp = p_fsa.second.vehicle;
        v_tmp.set_state(state_output);
        surround_trajs->at(p_fsa.first).push_back(v_tmp);
        others_state_cache_this_step.insert(
            std::make_pair(p_fsa.first, state_output));

        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(p_fsa.first);
      }
    }

    // * update sim state after steps
    //更新所有车辆的状态，为下一个时间步做准备
    ego_fsagent_this_step.vehicle.set_state(ego_state_cache_this_step);
    for (const auto& ps : others_state_cache_this_step) {
      surrounding_fsagents_this_step.forward_sim_agents.at(ps.first)
          .vehicle.set_state(ps.second);
    }
  }

  return kSuccess;
}

/**
 * @brief 对单个周边车辆执行一步前向仿真
 * @param fsagent 周边车仿真代理
 * @param all_sim_vehicles 当前时刻所有参与仿真的车辆
 * @param sim_time_step 当前积分步长
 * @param state_out 输出：传播后的状态
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 使用基于参考车道的跟驰模型传播单个周边车辆状态。
 *
 * **实现逻辑**：
 * - 先在该车所属车道上寻找前车
 * - 再调用 `OnLaneForwardSimulation::PropagateOnce()` 执行一步积分
 *
 * **对应文档**：
 * - 第 3 节：多智能体闭环前向仿真
 */
ErrorType EudmPlanner::SurroundingAgentForwardSim(
    const ForwardSimAgent& fsagent, const common::VehicleSet& all_sim_vehicles,
    const decimal_t& sim_time_step, common::State* state_out) const {
  common::Vehicle leading_vehicle;
  common::State state_output;
  decimal_t distance_residual_ratio = 0.0;
  // 获取前车（Leading Vehicle）以计算 IDM 模型输入
  map_itf_->GetLeadingVehicleOnLane(fsagent.lane, fsagent.vehicle.state(),
                                    all_sim_vehicles, fsagent.lat_range,
                                    &leading_vehicle, &distance_residual_ratio);
  // 执行一步 IDM 模型积分（PropagateOnce）
  if (planning::OnLaneForwardSimulation::PropagateOnce(
          fsagent.stf, fsagent.vehicle, leading_vehicle, sim_time_step,
          fsagent.sim_param, &state_output) != kSuccess) {
    return kWrongStatus;
  }
  *state_out = state_output;
  return kSuccess;
}

/**
 * @brief 对自车执行一步前向仿真
 * @param ego_fsagent 自车仿真代理
 * @param all_sim_vehicles 当前时刻所有参与仿真的车辆
 * @param sim_time_step 当前积分步长
 * @param state_out 输出：传播后的自车状态
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 按当前横向模式分别处理车道保持与换道情形，是 EUDM 闭环仿真的核心执行器。
 *
 * **两类模式**：
 * 1. **车道保持**：
 *    - 仅跟踪目标车道上的前车
 *    - 调用 `PropagateOnceAdvancedLK()`
 * 2. **换道**：
 *    - 同时考虑当前车道前车、目标车道前后车
 *    - 允许在必要时触发“主动避让”参数调整
 *
 * **主动避让（Evasive Behavior）**：
 * - 当前实现会在目标车道后车造成严格 RSS 风险时，
 *   临时提升期望速度/加速度，并可施加虚拟横向屏障偏移
 * - 这是对文档中“风险场景下闭环交互”的工程化近似实现
 *
 * **对应文档**：
 * - 第 3 节：闭环前向仿真考虑交互
 * - 第 4 节：风险场景下的策略评估
 */
ErrorType EudmPlanner::EgoAgentForwardSim(
    const ForwardSimEgoAgent& ego_fsagent,
    const common::VehicleSet& all_sim_vehicles, const decimal_t& sim_time_step,
    common::State* state_out) const {
  common::State state_output;

  if (ego_fsagent.lat_behavior == LateralBehavior::kLaneKeeping) {
    // * Lane keeping, only consider leading vehicle on ego lane
    // 1. 车道保持模式（Lane Keeping）
    // 仅考虑当前车道的前车
    common::Vehicle leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.target_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // ~ with leading vehicle
      // 检查与前车的碰撞
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          leading_vehicle.param(), leading_vehicle.state(), &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // TODO(lu.zhang): consider lateral social force to get lateral offset
    decimal_t lat_track_offset = 0.0;
    // 使用 AdvancedLK 模型进行积分（通常结合纯追踪和 IDM）
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLK(
            ego_fsagent.target_stf, ego_fsagent.vehicle, leading_vehicle,
            lat_track_offset, sim_time_step, ego_fsagent.sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    // * Lane changing, consider multiple vehicles
    // 2. 换道模式（Lane Changing）
    // 需要考虑当前车道前车、目标车道前车和目标车道后车
    common::Vehicle current_leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    // 获取当前车道前车（Current Leading）
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.current_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &current_leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // ~ with leading vehicle
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          current_leading_vehicle.param(), current_leading_vehicle.state(),
          &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // 获取目标车道的目标空隙前后车（Target Gap Vehicles）
    common::Vehicle gap_front_vehicle;
    if (ego_fsagent.target_gap_ids(0) != -1) {
      gap_front_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(0));
    }
    common::Vehicle gap_rear_vehicle;
    if (ego_fsagent.target_gap_ids(1) != -1) {
      gap_rear_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(1));
    }

    // TODO(lu.zhang): consider lateral social force to get lateral offset
    decimal_t lat_track_offset = 0.0;
    auto sim_param = ego_fsagent.sim_param;

    // 激进避让行为（Evasive Behavior）
    // 如果启用了避让且存在目标车道后车
    if (gap_rear_vehicle.id() != kInvalidAgentId &&
        cfg_.sim().ego().evasive().evasive_enable()) {
      common::FrenetState ego_on_tarlane_fs;
      common::FrenetState rear_on_tarlane_fs;
      if (ego_fsagent.target_stf.GetFrenetStateFromState(
              ego_fsagent.vehicle.state(), &ego_on_tarlane_fs) == kSuccess &&
          ego_fsagent.target_stf.GetFrenetStateFromState(
              gap_rear_vehicle.state(), &rear_on_tarlane_fs) == kSuccess) {
        // * rss check for evasive behavior
        // 进行 RSS 检查判断是否需要加速避让
        bool is_rss_safe = true;
        common::RssChecker::LongitudinalViolateType type;
        decimal_t rss_vel_low, rss_vel_up;
        common::RssChecker::RssCheck(ego_fsagent.vehicle, gap_rear_vehicle,
                                     ego_fsagent.target_stf,
                                     rss_config_strict_as_rear_, &is_rss_safe,
                                     &type, &rss_vel_low, &rss_vel_up);
        if (!is_rss_safe) {
          if (type == common::RssChecker::LongitudinalViolateType::TooSlow) {
            // 如果自车太慢，增加期望速度和加速度以拉开距离（主动避让）
            sim_param.idm_param.kDesiredVelocity = std::max(
                sim_param.idm_param.kDesiredVelocity,
                rss_vel_low + cfg_.sim().ego().evasive().lon_extraspeed());
            sim_param.idm_param.kDesiredHeadwayTime =
                cfg_.sim().ego().evasive().head_time();
            sim_param.idm_param.kAcceleration =
                cfg_.sim().ego().evasive().lon_acc();
            sim_param.max_lon_acc_jerk = cfg_.sim().ego().evasive().lon_jerk();
          }
        }
        // 虚拟屏障（Virtual Barrier）：在横向增加偏移，模拟“挤入”或被“挤出”的效果
        if (cfg_.sim().ego().evasive().virtual_barrier_enable()) {
          if (ego_on_tarlane_fs.vec_s[0] - rear_on_tarlane_fs.vec_s[0] <
              gap_rear_vehicle.param().length() +
                  cfg_.sim().ego().evasive().virtual_barrier_tic() *
                      gap_rear_vehicle.state().velocity) {
            lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
          }
          common::FrenetState front_on_tarlane_fs;
          if (gap_front_vehicle.id() != kInvalidAgentId &&
              ego_fsagent.target_stf.GetFrenetStateFromState(
                  gap_front_vehicle.state(), &front_on_tarlane_fs) ==
                  kSuccess) {
            if (front_on_tarlane_fs.vec_s[0] - ego_on_tarlane_fs.vec_s[0] <
                ego_fsagent.vehicle.param().length() +
                    cfg_.sim().ego().evasive().virtual_barrier_tic() *
                        ego_fsagent.vehicle.state().velocity) {
              lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
            }
          }
        }
      }
    }

    // 使用 AdvancedLC 模型进行积分
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLC(
            ego_fsagent.current_stf, ego_fsagent.target_stf,
            ego_fsagent.vehicle, current_leading_vehicle, gap_front_vehicle,
            gap_rear_vehicle, lat_track_offset, sim_time_step, sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  }

  *state_out = state_output;

  return kSuccess;
}

ErrorType EudmPlanner::JudgeBehaviorByLaneId(
    const int ego_lane_id_by_pos, LateralBehavior* behavior_by_lane_id) {
  if (ego_lane_id_by_pos == ego_lane_id_) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  auto it = std::find(potential_lk_lane_ids_.begin(),
                      potential_lk_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcl = std::find(potential_lcl_lane_ids_.begin(),
                          potential_lcl_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcr = std::find(potential_lcr_lane_ids_.begin(),
                          potential_lcr_lane_ids_.end(), ego_lane_id_by_pos);

  if (it != potential_lk_lane_ids_.end()) {
    // ~ if routing information is available, here
    // ~ we still need to check whether the change is consist with the
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  if (it_lcl != potential_lcl_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeLeft;
    return kSuccess;
  }

  if (it_lcr != potential_lcr_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeRight;
    return kSuccess;
  }

  *behavior_by_lane_id = common::LateralBehavior::kUndefined;
  return kSuccess;
}

ErrorType EudmPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
  //                     &potential_lk_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeLeft,
  //                     &potential_lcl_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_,
  // common::LateralBehavior::kLaneChangeRight,
  //                     &potential_lcr_lane_ids_);
  return kSuccess;
}

ErrorType EudmPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) const {
  candidate_lane_ids->clear();
  if (beh == common::LateralBehavior::kUndefined ||
      beh == common::LateralBehavior::kLaneKeeping) {
    map_itf_->GetChildLaneIds(source_lane_id, candidate_lane_ids);
  } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
    int l_lane_id;
    if (map_itf_->GetLeftLaneId(source_lane_id, &l_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(l_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(l_lane_id);
    }
  } else if (beh == common::LateralBehavior::kLaneChangeRight) {
    int r_lane_id;
    if (map_itf_->GetRightLaneId(source_lane_id, &r_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(r_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(r_lane_id);
    }
  } else {
    assert(false);
  }
  return kSuccess;
}

void EudmPlanner::set_map_interface(EudmPlannerMapItf* itf) { map_itf_ = itf; }

void EudmPlanner::set_desired_velocity(const decimal_t desired_vel) {
  desired_velocity_ = std::max(0.0, desired_vel);
}

void EudmPlanner::set_lane_change_info(const LaneChangeInfo& lc_info) {
  lc_info_ = lc_info;
}

decimal_t EudmPlanner::desired_velocity() const { return desired_velocity_; }

int EudmPlanner::winner_id() const { return winner_id_; }

decimal_t EudmPlanner::time_cost() const { return time_cost_; }

void EudmPlanner::UpdateDcpTree(const DcpAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->UpdateScript();
  sim_time_total_ = dcp_tree_ptr_->planning_horizon();
}

EudmPlannerMapItf* EudmPlanner::map_itf() const { return map_itf_; }

}  // namespace planning
