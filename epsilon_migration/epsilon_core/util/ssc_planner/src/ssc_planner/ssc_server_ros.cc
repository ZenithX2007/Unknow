/**
 * @file ssc_server.cc
 * @author GW
 * @brief SSC规划器的ROS服务端实现
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件实现了SscPlannerServer类，它是SSC规划器的ROS节点封装。
 * 主要负责：
 * 1. 接收和处理语义地图（SemanticMap）输入
 * 2. 维护规划循环（Planning Loop）
 * 3. 调用SscPlanner核心算法库进行轨迹生成
 * 4. 发布规划结果（轨迹、控制指令）和可视化信息
 *
 * **核心功能**：
 * - **非阻塞数据接收**：使用moodycamel::ReaderWriterQueue实现地图数据的无锁接收
 * - **重规划机制（Replanning）**：基于Receding Horizon Planning（RHP）策略
 *   - 在当前轨迹执行过程中，提前计算下一段轨迹
 *   - 拼接新旧轨迹，保证控制的连续性
 * - **奇异状态过滤**：处理低速下的运动学状态奇异问题
 * - **可视化**：提供丰富的Rviz可视化接口（SMM、SSC、轨迹等）
 */

#include "ssc_planner/ssc_server_ros.h"

namespace planning {

SscPlannerServer::SscPlannerServer(ros::NodeHandle nh, int ego_id)
    : nh_(nh), work_rate_(20.0), ego_id_(ego_id) {
  // 初始化非阻塞队列缓冲区
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  // 初始化语义地图可视化器
  p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
  // 初始化SSC可视化器
  p_ssc_vis_ = new SscVisualizer(nh, ego_id);
}

/**
 * @brief 构造函数（指定工作频率）
 * @param nh ROS节点句柄
 * @param work_rate 规划循环频率 (Hz)
 * @param ego_id 自车ID
 *
 * 重载构造函数，允许自定义规划频率。
 */
SscPlannerServer::SscPlannerServer(ros::NodeHandle nh, double work_rate,
                                   int ego_id)
    : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) {
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
  p_ssc_vis_ = new SscVisualizer(nh, ego_id);
}

/**
 * @brief 接收语义地图数据
 * @param smm 语义地图管理器对象
 *
 * 这是一个回调函数（通常由Behavior那一层调用），将最新的地图数据推入缓冲区。
 * 使用 try_enqueue 保证线程安全且非阻塞。
 */
void SscPlannerServer::PushSemanticMap(const SemanticMapManager& smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

/**
 * @brief 发布可视化数据和反馈信息
 *
 * 发布内容包括：
 * 1. 语义地图（SMM）可视化：车道、障碍物等
 * 2. SSC可视化：时空走廊、Bézier曲线控制点等
 * 3. 实时控制指令：将轨迹状态转换为车辆控制指令（速度、转向角）
 * 4. 轨迹可视化：当前执行的轨迹（MarkerArray）
 * 5. 干预信号：当规划失败时显示警告
 */
void SscPlannerServer::PublishData() {
  using common::VisualizationUtil;
  auto current_time = ros::Time::now().toSec();
  // 语义地图（SMM）可视化
  {
    // 可视化带有时间戳的语义地图数据（车道、障碍物等）
    p_smm_vis_->VisualizeDataWithStamp(ros::Time(current_time), last_smm_);
    // 发布TF变换（如果需要）
    p_smm_vis_->SendTfWithStamp(ros::Time(current_time), last_smm_);
  }

  // SSC可视化（时空走廊）
  {
    TicToc timer;
    // 可视化SSC相关数据（走廊几何、膨胀立方体、参考线等）
    p_ssc_vis_->VisualizeDataWithStamp(ros::Time(current_time), planner_);
    printf("[SscPlannerServer]ssc vis all time cost: %lf ms\n", timer.toc());
  }

  // 轨迹反馈（Trajectory Feedback）
  {
    if (executing_traj_ == nullptr || !executing_traj_->IsValid()) return;

    // 调试信息：输出是否使用仿真状态
    if (use_sim_state_) {
      printf("[SscPlannerServer]use_sim_state_: true.\n");
    } else {
      printf("[SscPlannerServer]use_sim_state_: false.\n");
    }

    if (use_sim_state_) {
      // 计算当前时间在轨迹中的位置
      decimal_t plan_horizon = 1.0 / work_rate_;
      int num_cycles =
          std::floor((current_time - executing_traj_->begin()) / plan_horizon);
      decimal_t ct = executing_traj_->begin() + num_cycles * plan_horizon;
      {
        common::State state;
        if (executing_traj_->GetState(ct, &state) == kSuccess) {
          // 过滤奇异状态（处理低速不稳定性）
          FilterSingularityState(ctrl_state_hist_, &state);
          ctrl_state_hist_.push_back(state);
          if (ctrl_state_hist_.size() > 100)
            ctrl_state_hist_.erase(ctrl_state_hist_.begin());
          
          // 生成并发布车辆控制指令（Vehicle Control Signal）
          vehicle_msgs::ControlSignal ctrl_msg;
          vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
              common::VehicleControlSignal(state), ros::Time(ct),
              std::string("map"), &ctrl_msg);
          ctrl_signal_pub_.publish(ctrl_msg);
        } else {
          printf(
              "[SscPlannerServer]cannot evaluate state at %lf with begin "
              "%lf.\n",
              ct, executing_traj_->begin());
        }
      }
    }

    // 轨迹可视化（Trajectory Visualization）
    {
      // 默认颜色为洋红色（Magenta），如果请求人工干预则变为黄色（Yellow）
      auto color = common::cmap["magenta"];
      if (require_intervention_signal_) color = common::cmap["yellow"];
      
      // 生成轨迹的MarkerArray
      visualization_msgs::MarkerArray traj_mk_arr;
      common::VisualizationUtil::GetMarkerArrayByTrajectory(
          *executing_traj_, 0.1, Vecf<3>(0.3, 0.3, 0.3), color, 0.5,
          &traj_mk_arr);
      
      // 如果需要人工干预，在起点上方显示红色警告文字 "Intervention Needed!"
      if (require_intervention_signal_) {
        visualization_msgs::Marker traj_status;
        common::State state_begin;
        executing_traj_->GetState(executing_traj_->begin(), &state_begin);
        Vec3f pos = Vec3f(state_begin.vec_position[0],
                          state_begin.vec_position[1], 5.0);
        common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
            pos, std::string("Intervention Needed!"), common::cmap["red"],
            Vec3f(5.0, 5.0, 5.0), 0, &traj_status);
        traj_mk_arr.markers.push_back(traj_status);
      }
      
      // 处理Marker ID，防止未清除的Ghost Marker残留在Rviz中
      int num_traj_mks = static_cast<int>(traj_mk_arr.markers.size());
      common::VisualizationUtil::FillHeaderIdInMarkerArray(
          ros::Time(current_time), std::string("map"), last_trajmk_cnt_,
          &traj_mk_arr);
      last_trajmk_cnt_ = num_traj_mks;
      
      // 发布轨迹可视化消息
      executing_traj_vis_pub_.publish(traj_mk_arr);
    }
  }
}

/**
 * @brief 过滤奇异状态
 * @param hist 历史状态序列
 * @param filter_state 当前需要检查的状态
 * @return ErrorType
 *
 * **问题背景**：
 * 在车辆低速或停车时（velocity -> 0），车辆的航向角（orientation）计算可能会出现数值不稳定或奇异。
 * 
 * **处理逻辑**：
 * 如果速度极小（< kBigEPS）且角度变化剧烈（超过物理限制），
 * 这一步会认定为测量或计算噪声，强制将角度保持为上一时刻的角度，
 * 防止控制指令出现抖动。
 */
ErrorType SscPlannerServer::FilterSingularityState(
    const vec_E<common::State>& hist, common::State* filter_state) {
  if (hist.empty()) {
    return kWrongStatus;
  }
  decimal_t duration = filter_state->time_stamp - hist.back().time_stamp;
  decimal_t wheel_base = 2.85;
  decimal_t max_steer = M_PI / 4.0;
  decimal_t singular_velocity = kBigEPS;
  decimal_t max_orientation_rate = tan(max_steer) / 2.85 * singular_velocity;
  decimal_t max_orientation_change = max_orientation_rate * duration;

  if (fabs(filter_state->velocity) < singular_velocity &&
      fabs(normalize_angle(filter_state->angle - hist.back().angle)) >
          max_orientation_change) {
    printf(
        "[SscPlannerServer]Detect singularity velocity %lf angle (%lf, %lf).\n",
        filter_state->velocity, hist.back().angle, filter_state->angle);
    filter_state->angle = hist.back().angle;
    printf("[SscPlannerServer]Filter angle to %lf.\n", hist.back().angle);
  }
  return kSuccess;
}

/**
 * @brief 初始化ROS服务端
 * @param config_path 配置文件路径
 *
 * 1. 初始化核心规划器（planner_.Init）
 * 2. 读取ROS参数（如 use_sim_state）
 * 3. 注册ROS发布者（控制指令、Map Marker、轨迹 Marker）
 */
void SscPlannerServer::Init(const std::string& config_path) {
  planner_.Init(config_path);

  std::string traj_topic = std::string("/vis/agent_") +
                           std::to_string(ego_id_) +
                           std::string("/ssc/exec_traj");
  nh_.param("use_sim_state", use_sim_state_, true);

  ctrl_signal_pub_ = nh_.advertise<vehicle_msgs::ControlSignal>("ctrl", 20);
  map_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("ssc_map", 1);
  executing_traj_vis_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(traj_topic, 1);
}

/**
 * @brief 启动规划服务
 *
 * 开启主线程（MainThread），以固定频率执行规划循环。
 */
void SscPlannerServer::Start() {
  if (is_replan_on_) {
    return;
  }
  planner_.set_map_interface(&map_adapter_);
  printf("[SscPlannerServer]Planner server started.\n");
  is_replan_on_ = true;

  std::thread(&SscPlannerServer::MainThread, this).detach();
}

/**
 * @brief 主规划线程
 *
 * 包含一个死循环，通过 sleep_until 保证精确的循环周期（默认20Hz）。
 * 每个周期调用 PlanCycleCallback()。
 */
void SscPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (true) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

/**
 * @brief 规划循环回调（核心逻辑）
 *
 * 流程：
 * 1. **获取最新地图**：从缓冲区取出最新的语义地图
 * 2. **更新适配器**：将地图传递给 planning core
 * 3. **发布可视化**：PublishData()
 * 4. **初始规划**：
 *    - 如果当前没有正在执行的轨迹（Mission Start），调用 RunOnce()
 *    - 初始化执行轨迹和历史状态
 * 5. **任务完成检查**：如果当前时间超过轨迹结束时间，结束任务
 * 6. **重规划（Replan）**：
 *    - 检查是否有 next_traj_（上一周期规划好的轨迹）
 *    - 如果有，将其拼接为当前的 executing_traj_
 *    - 调用 Replan() 启动新一轮的规划（为未来做准备）
 */
void SscPlannerServer::PlanCycleCallback() {
  if (!is_replan_on_) {
    return;
  }
  // printf("input buffer size: %d.\n", p_input_smm_buff_->size_approx());
  // is_map_updated_ = false;  // return when no new map
  while (p_input_smm_buff_->try_dequeue(last_smm_)) {
    is_map_updated_ = true;
  }

  if (!is_map_updated_) return;

  // 更新地图（Update map）
  // 将最新的SMM数据封装为shared_ptr并设置给地图适配器
  auto map_ptr =
      std::make_shared<semantic_map_manager::SemanticMapManager>(last_smm_);
  map_adapter_.set_map(map_ptr);

  PublishData();

  auto current_time = ros::Time::now().toSec();
  printf("[SscPlannerServer]>>>>>>>current time %lf.\n", current_time);
  if (executing_traj_ == nullptr || !executing_traj_->IsValid() ||
      !use_sim_state_) {
    // 初始规划（Initial planning）
    if (planner_.RunOnce() != kSuccess) {
      // printf("[SscPlannerServer]Initial planning failed.\n");
      return;
    }

    desired_state_hist_.clear();
    desired_state_hist_.push_back(last_smm_.ego_vehicle().state());
    ctrl_state_hist_.clear();
    ctrl_state_hist_.push_back(last_smm_.ego_vehicle().state());
    executing_traj_ = std::move(planner_.trajectory());
    global_init_stamp_ = executing_traj_->begin();
    printf(
        "[SscPlannerServer]init plan success with stamp: %lf and angle %lf.\n",
        global_init_stamp_, last_smm_.ego_vehicle().state().angle);
    return;
  }

  // 检查任务是否完成（Mission Complete Check）
  // 仅当开启了Use Sim State且当前时间超过了轨迹的结束时间
  if (current_time > executing_traj_->end()) {
    printf("[SscPlannerServer]Current time %lf out of [%lf, %lf].\n",
           current_time, executing_traj_->begin(), executing_traj_->end());
    printf("[SscPlannerServer]Mission complete.\n");
    // is_replan_on_ = false;  // 可选：停止重规划
    executing_traj_.release();
    next_traj_.release();
    return;
  }

  // 注意：如果想禁用重规划，可以注释掉此块
  // NOTE: comment this block if you want to disable replan
  {
    // 如果没有下一段轨迹，或者下一段轨迹无效，尝试重规划
    if (next_traj_ == nullptr || !next_traj_->IsValid()) {
      Replan();
      return;
    }

    // 如果下一段轨迹有效，进行轨迹拼接（Replanning Stitching）
    if (next_traj_->IsValid()) {
      executing_traj_ = std::move(next_traj_);
      // next_traj_ = common::Trajectory();
      Replan();
      return;
    }
  }
}

/**
 * @brief 重规划逻辑（Receding Horizon Planning）
 *
 * **核心思想**：
 * 在当前轨迹执行结束前，提前规划下一段轨迹，保证车辆持续运动。
 *
 * **关键步骤**：
 * 1. **确定规划起点时间（t）**：
 *    - 考虑系统延迟和计算耗时
 *    - 规划起点通常选在当前时间之后的某个时刻（lookahead）
 * 2. **获取规划起点状态**：
 *    - 从当前正在执行的轨迹（executing_traj_）中采样 t 时刻的状态
 *    - 作为新一轮规划的起始状态（Initial State）
 * 3. **执行规划**：
 *    - 设置 planner_ 的初始状态
 *    - 调用 planner_.RunOnce()
 * 4. **保存结果**：
 *    - 成功规划的轨迹保存为 next_traj_，将在下个周期接管控制
 */
void SscPlannerServer::Replan() {
  if (!is_replan_on_) return;
  if (executing_traj_ == nullptr || !executing_traj_->IsValid()) return;

  decimal_t plan_horizon = 1.0 / work_rate_;
  common::State desired_state;
  decimal_t cur_time = ros::Time::now().toSec();

  // 计算已经执行的周期数
  int num_cycles_exec = std::floor(
      (executing_traj_->begin() - global_init_stamp_) / plan_horizon);
  // 计算需要向前预测的周期数（Lookahead steps）
  int num_cycles_ahead =
      cur_time > executing_traj_->begin()
          ? std::floor((cur_time - global_init_stamp_) / plan_horizon) + 1
          : num_cycles_exec + 1;
  // 确定新规划的起始时间 t
  decimal_t t = global_init_stamp_ + plan_horizon * num_cycles_ahead;

  printf(
      "[SscPlannerServer]init stamp: %lf, plan horizon: %lf, num cycles %d.\n",
      global_init_stamp_, plan_horizon, num_cycles_ahead);
  printf(
      "[SscPlannerServer]Replan at cur time %lf with executing traj begin "
      "time: %lf to rounded t: %lf.\n",
      cur_time, executing_traj_->begin(), t);

  // 获取t时刻的期望状态（作为新规划的起点）
  if (executing_traj_->GetState(t, &desired_state) != kSuccess) {
    printf("[SscPlannerServer]Cannot get desired state at %lf.\n", t);
    return;
  }
  printf(
      "[SscPlannerServer]t %lf, desired state (x,y,v,a,theta):(%lf, %lf, %lf, "
      "%lf, %lf).\n",
      t, desired_state.vec_position[0], desired_state.vec_position[1],
      desired_state.velocity, desired_state.acceleration, desired_state.angle);

  // 过滤奇异状态并更新期望状态历史
  FilterSingularityState(desired_state_hist_, &desired_state);
  desired_state_hist_.push_back(desired_state);
  if (desired_state_hist_.size() > 100)
    desired_state_hist_.erase(desired_state_hist_.begin());

  // 设置核心规划器的初始状态
  planner_.set_initial_state(desired_state);

  // 执行一次规划
  time_profile_tool_.tic();
  if (planner_.RunOnce() != kSuccess) {
    printf("[SscPlannerServer]Ssc planner core failed in %lf ms.\n",
           time_profile_tool_.toc());
    require_intervention_signal_ = true;
    return;
  }

  require_intervention_signal_ = false;
  printf("[SscPlannerServer]Ssc planner succeed in %lf ms.\n",
         time_profile_tool_.toc());

  // 保存规划结果到 next_traj_
  next_traj_ = std::move(planner_.trajectory());
}

}  // namespace planning