/**
 * @file eudm_server_ros.cc
 * @author GW
 * @brief EUDM ROS 服务端实现：在线行为规划线程、任务输入与结果回写
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件实现 `EudmPlannerServer`，
 * 它把 `EudmManager` 封装成一个持续运行的 ROS 在线行为规划节点。
 *
 * **核心功能**：
 * 1. **地图输入缓冲**
 *    - 接收 `SemanticMapManager` 快照并压入无锁队列
 *    - 每个周期只消费最新一帧，自动丢弃过时地图
 *
 * 2. **固定频率后台重规划**
 *    - 在独立线程中按设定频率执行 `PlanCycleCallback()`
 *    - 调用 `EudmManager::Run()` 完成一次高层行为决策
 *
 * 3. **用户输入转任务约束**
 *    - 将 joystick/HMI 输入翻译为偏好行为、目标速度和换道限制
 *    - 在下一轮重规划中作为 `task_` 输入生效
 *
 * 4. **结果回写与系统集成**
 *    - 将 winner 行为回写到语义地图中的 ego behavior
 *    - 通过回调把结果继续传给下游模块，例如 SSC
 *    - 同时触发可视化发布
 *
 * **系统链路**：
 * `SemanticMapManager -> EudmPlannerServer -> EudmManager -> callback(SSC/agent)`
 *
 */

#include "eudm_planner/eudm_server_ros.h"

namespace planning {

/**
 * @brief 构造函数（默认 20Hz）
 * @param nh ROS 节点句柄
 * @param ego_id 自车 ID
 *
 * 主要初始化可视化器、输入缓冲区和默认任务状态。
 */
EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, int ego_id)
    : nh_(nh), work_rate_(20.0), ego_id_(ego_id) {
  // visualizer 直接从 manager/planner 中读取结果做展示。
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

/**
 * @brief 构造函数（指定工作频率）
 * @param nh ROS 节点句柄
 * @param work_rate 在线规划频率
 * @param ego_id 自车 ID
 */
EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, double work_rate,
                                     int ego_id)
    : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

/**
 * @brief 推送最新语义地图快照
 * @param smm 语义地图管理器快照
 *
 * 生产者侧只负责入队；真正消费由后台规划线程完成。
 */
void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

/**
 * @brief 发布行为规划可视化结果
 *
 * 可视化器直接从 `manager/planner` 内部缓存读取本轮结果。
 */
void EudmPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(ros::Time::now());
}

/**
 * @brief 初始化 EUDM ROS 服务端
 * @param bp_config_path 行为规划配置路径
 *
 * 初始化 manager、joystick 订阅与可视化器。
 */
void EudmPlannerServer::Init(const std::string &bp_config_path) {
  bp_manager_.Init(bp_config_path, work_rate_);
  joy_sub_ = nh_.subscribe("/joy", 10, &EudmPlannerServer::JoyCallback, this);
  nh_.param("use_sim_state", use_sim_state_, true);
  p_visualizer_->Init();
  p_visualizer_->set_use_sim_state(use_sim_state_);
}

/**
 * @brief joystick 输入回调
 * @param msg joystick 消息
 *
 * 将用户输入翻译为任务级约束，
 * 例如左右换道偏好、目标速度调整、换道禁止开关与接管状态。
 */
void EudmPlannerServer::JoyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  int msg_id;
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) return;
  // 按键映射:
  // [2] 左换道请求
  // [1] 右换道请求
  // [3] 期望速度 +1 m/s
  // [0] 期望速度 -1 m/s
  // [4]/[5] 切换左右换道禁止信号
  // [6] 切换自动控制状态
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;

  if (msg->buttons[2] == 1) {
    if (task_.user_perferred_behavior != -1) {
      task_.user_perferred_behavior = -1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[1] == 1) {
    if (task_.user_perferred_behavior != 1) {
      task_.user_perferred_behavior = 1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[3] == 1) {
    task_.user_desired_vel = task_.user_desired_vel + 1.0;
  } else if (msg->buttons[0] == 1) {
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0);
  } else if (msg->buttons[4] == 1) {
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left;
  } else if (msg->buttons[5] == 1) {
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right;
  } else if (msg->buttons[6] == 1) {
    task_.is_under_ctrl = !task_.is_under_ctrl;
  }
}

/**
 * @brief 启动后台规划线程
 */
void EudmPlannerServer::Start() {
  std::thread(&EudmPlannerServer::MainThread, this).detach();
  task_.is_under_ctrl = true;
}

/**
 * @brief 后台主线程
 *
 * 以固定周期运行在线重规划循环。
 */
void EudmPlannerServer::MainThread() {
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
 * @brief 单个在线规划周期回调
 *
 * 本函数完成：
 * 1. 从缓冲区提取最新语义地图
 * 2. 量化时间戳以稳定动作层边界
 * 3. 调用 `EudmManager::Run()` 执行行为规划
 * 4. 将结果回写到语义地图并触发外部回调
 * 5. 发布可视化
 */
void EudmPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  bool has_updated_map = false;
  while (p_input_smm_buff_->try_dequeue(smm_)) {
    // 一直出队到最新一帧，旧帧自动被抛弃。
    has_updated_map = true;
  }

  if (!has_updated_map) return;

  auto map_ptr =
      std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);

  decimal_t replan_duration = 1.0 / work_rate_;
  // 按规划频率对时间戳做量化，使动作层边界更稳定。
  double stamp =
      std::floor(smm_.time_stamp() / replan_duration) * replan_duration;

  if (bp_manager_.Run(stamp, map_ptr, task_) == kSuccess) {
    // 若规划成功，就把行为层结果写回语义地图中的 ego behavior。
    common::SemanticBehavior behavior;
    bp_manager_.ConstructBehavior(&behavior);
    smm_.set_ego_behavior(behavior);
  }

  if (has_callback_binded_) {
    // 为系统集成留一个“规划完成后的回调”钩子。
    private_callback_fn_(smm_);
  }

  PublishData();
}

/**
 * @brief 绑定规划完成后的外部回调
 * @param fn 回调函数
 *
 * 常用于把行为层输出继续传递给 SSC 或执行层。
 */
void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

/**
 * @brief 设置用户期望速度
 * @param desired_vel 用户目标速度
 */
void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

/**
 * @brief 获取当前用户期望速度
 * @return 当前任务中的目标速度
 */
decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
