#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vehicle_msgs/msg/arena_info.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/msg/predicted_trajectory_array.hpp"

#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/trajectory/frenet_traj.h"
#include "eudm_planner/eudm_manager.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "ssc_planner/map_adapter.h"
#include "ssc_planner/ssc_planner.h"

namespace {

double StampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double DurationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

std::string DefaultShareFile(const std::string & package_name, const std::string & relative_path)
{
  return ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double AngleDifference(double lhs, double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

common::State ToState(const vehicle_msgs::msg::State & msg)
{
  common::State state;
  state.time_stamp = StampSeconds(msg.header.stamp);
  state.vec_position[0] = msg.vec_position.x;
  state.vec_position[1] = msg.vec_position.y;
  state.angle = msg.angle;
  state.curvature = msg.curvature;
  state.velocity = msg.velocity;
  state.acceleration = msg.acceleration;
  state.steer = msg.steer;
  return state;
}

void FillStateMsg(
  const common::State & state, const rclcpp::Time & stamp, const std::string & frame_id,
  vehicle_msgs::msg::State * msg)
{
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->vec_position.x = state.vec_position[0];
  msg->vec_position.y = state.vec_position[1];
  msg->vec_position.z = 0.0;
  msg->angle = state.angle;
  msg->curvature = state.curvature;
  msg->velocity = state.velocity;
  msg->acceleration = state.acceleration;
  msg->steer = state.steer;
}

common::VehicleParam ToVehicleParam(
  const vehicle_msgs::msg::VehicleParam & msg, const common::VehicleParam & defaults)
{
  common::VehicleParam param = defaults;
  if (msg.width > 0.0) {
    param.set_width(msg.width);
  }
  if (msg.length > 0.0) {
    param.set_length(msg.length);
  }
  if (msg.wheel_base > 0.0) {
    param.set_wheel_base(msg.wheel_base);
  }
  if (msg.front_suspension > 0.0) {
    param.set_front_suspension(msg.front_suspension);
  }
  if (msg.rear_suspension > 0.0) {
    param.set_rear_suspension(msg.rear_suspension);
  }
  if (msg.max_steering_angle > 0.0) {
    param.set_max_steering_angle(msg.max_steering_angle);
  }
  if (msg.max_longitudinal_acc > 0.0) {
    param.set_max_longitudinal_acc(msg.max_longitudinal_acc);
  }
  if (msg.max_lateral_acc > 0.0) {
    param.set_max_lateral_acc(msg.max_lateral_acc);
  }
  if (msg.d_cr > 0.0) {
    param.set_d_cr(msg.d_cr);
  }
  return param;
}

common::Vehicle ToVehicle(
  const vehicle_msgs::msg::Vehicle & msg, const common::VehicleParam & defaults)
{
  common::Vehicle vehicle;
  vehicle.set_id(msg.id.data);
  vehicle.set_subclass(msg.subclass.data);
  vehicle.set_type(msg.type.data.empty() ? std::string("car") : msg.type.data);
  vehicle.set_param(ToVehicleParam(msg.param, defaults));
  vehicle.set_state(ToState(msg.state));
  return vehicle;
}

common::VehicleSet ToVehicleSet(
  const vehicle_msgs::msg::VehicleSet & msg, const common::VehicleParam & defaults)
{
  common::VehicleSet vehicle_set;
  for (const auto & vehicle_msg : msg.vehicles) {
    auto vehicle = ToVehicle(vehicle_msg, defaults);
    vehicle_set.vehicles.emplace(vehicle.id(), vehicle);
  }
  return vehicle_set;
}

common::LaneRaw ToLaneRaw(const vehicle_msgs::msg::Lane & msg)
{
  common::LaneRaw lane;
  lane.id = msg.id;
  lane.dir = msg.dir;
  lane.child_id = msg.child_id;
  lane.father_id = msg.father_id;
  lane.l_lane_id = msg.l_lane_id;
  lane.l_change_avbl = msg.l_change_avbl;
  lane.r_lane_id = msg.r_lane_id;
  lane.r_change_avbl = msg.r_change_avbl;
  lane.behavior = msg.behavior;
  lane.length = msg.length;
  lane.start_point = Vec2f(msg.start_point.x, msg.start_point.y);
  lane.final_point = Vec2f(msg.final_point.x, msg.final_point.y);
  for (const auto & point : msg.points) {
    lane.lane_points.push_back(Vec2f(point.x, point.y));
  }
  return lane;
}

common::LaneNet ToLaneNet(const vehicle_msgs::msg::LaneNet & msg)
{
  common::LaneNet lane_net;
  for (const auto & lane_msg : msg.lanes) {
    auto lane = ToLaneRaw(lane_msg);
    lane_net.lane_set.emplace(lane.id, lane);
  }
  return lane_net;
}

common::CircleObstacle ToCircleObstacle(const vehicle_msgs::msg::CircleObstacle & msg)
{
  common::CircleObstacle obstacle;
  obstacle.id = msg.id;
  obstacle.circle.center.x = msg.circle.center.x;
  obstacle.circle.center.y = msg.circle.center.y;
  obstacle.circle.center.z = msg.circle.center.z;
  obstacle.circle.radius = msg.circle.radius;
  return obstacle;
}

common::PolygonObstacle ToPolygonObstacle(const vehicle_msgs::msg::PolygonObstacle & msg)
{
  common::PolygonObstacle obstacle;
  obstacle.id = msg.id;
  for (const auto & point : msg.polygon.points) {
    common::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    obstacle.polygon.points.push_back(p);
  }
  return obstacle;
}

common::ObstacleSet ToObstacleSet(const vehicle_msgs::msg::ObstacleSet & msg)
{
  common::ObstacleSet obstacle_set;
  for (const auto & obstacle_msg : msg.obs_circle) {
    auto obstacle = ToCircleObstacle(obstacle_msg);
    obstacle_set.obs_circle.emplace(obstacle.id, obstacle);
  }
  for (const auto & obstacle_msg : msg.obs_polygon) {
    auto obstacle = ToPolygonObstacle(obstacle_msg);
    obstacle_set.obs_polygon.emplace(obstacle.id, obstacle);
  }
  return obstacle_set;
}

std::unordered_map<int, vec_E<common::Vehicle>> ToPredictedTrajectoryMap(
  const vehicle_msgs::msg::PredictedTrajectoryArray & msg,
  const common::VehicleParam & defaults, double planning_stamp, bool position_is_rear_axle,
  double min_probability, int ego_id)
{
  const auto base_stamp = StampSeconds(msg.header.stamp) > 0.0 ?
    StampSeconds(msg.header.stamp) :
    planning_stamp;
  std::unordered_map<int, vec_E<common::Vehicle>> trajectories;
  std::unordered_map<int, double> chosen_probabilities;

  for (const auto & trajectory_msg : msg.trajectories) {
    const auto vehicle_id = trajectory_msg.id.data;
    if (vehicle_id == ego_id) {
      continue;
    }
    if (trajectory_msg.probability < min_probability) {
      continue;
    }

    auto param = ToVehicleParam(trajectory_msg.param, defaults);
    auto type = trajectory_msg.type.data.empty() ? std::string("car") : trajectory_msg.type.data;
    vec_E<common::Vehicle> trajectory;
    trajectory.reserve(trajectory_msg.states.size());

    for (const auto & predicted_state_msg : trajectory_msg.states) {
      common::State state;
      state.time_stamp = base_stamp + DurationSeconds(predicted_state_msg.time_from_start);
      if (state.time_stamp + 1e-3 < planning_stamp) {
        continue;
      }

      state.angle = std::isfinite(predicted_state_msg.heading) ? predicted_state_msg.heading : 0.0;
      state.curvature =
        std::isfinite(predicted_state_msg.curvature) ? predicted_state_msg.curvature : 0.0;
      state.velocity =
        std::isfinite(predicted_state_msg.velocity) ? predicted_state_msg.velocity : 0.0;
      state.acceleration =
        std::isfinite(predicted_state_msg.acceleration) ? predicted_state_msg.acceleration : 0.0;
      state.steer = std::isfinite(predicted_state_msg.steer) ? predicted_state_msg.steer : 0.0;

      auto x = predicted_state_msg.position.x;
      auto y = predicted_state_msg.position.y;
      if (!position_is_rear_axle) {
        x -= param.d_cr() * std::cos(state.angle);
        y -= param.d_cr() * std::sin(state.angle);
      }
      state.vec_position[0] = x;
      state.vec_position[1] = y;

      common::Vehicle vehicle;
      vehicle.set_id(vehicle_id);
      vehicle.set_type(type);
      vehicle.set_param(param);
      vehicle.set_state(state);
      trajectory.push_back(vehicle);
    }

    if (!trajectory.empty() && (
        trajectories.count(vehicle_id) == 0 ||
        trajectory_msg.probability > chosen_probabilities[vehicle_id]))
    {
      std::sort(
        trajectory.begin(), trajectory.end(),
        [](const common::Vehicle & lhs, const common::Vehicle & rhs) {
          return lhs.state().time_stamp < rhs.state().time_stamp;
        });
      trajectories[vehicle_id] = trajectory;
      chosen_probabilities[vehicle_id] = trajectory_msg.probability;
    }
  }

  return trajectories;
}

std::string LatBehaviorName(common::LateralBehavior behavior)
{
  return common::SemanticsUtils::RetLatBehaviorName(behavior);
}

}  // namespace

class EpsilonIntegratedPlannerNode : public rclcpp::Node
{
public:
  EpsilonIntegratedPlannerNode()
  : Node("epsilon_integrated_planner_node")
  {
    DeclareParameters();
    ReadParameters();
    ConfigureCore();
    ConfigureRosInterfaces();
  }

private:
  void DeclareParameters()
  {
    declare_parameter<int>("ego_id", 0);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("agent_config_path", "");
    declare_parameter<std::string>("eudm_config_path", "");
    declare_parameter<std::string>("ssc_config_path", "");
    declare_parameter<double>("work_rate", 20.0);
    declare_parameter<double>("dynamic_scene_timeout", 1.5);
    declare_parameter<double>("desired_velocity", 1.5);
    declare_parameter<int>("preferred_lateral_behavior", 0);
    declare_parameter<bool>("is_under_control", true);
    declare_parameter<bool>("override_ego_with_odom", true);
    declare_parameter<bool>("odom_pose_is_rear_axle", false);
    declare_parameter<bool>("use_previous_trajectory_initial_state", true);
    declare_parameter<double>("trajectory_seed_max_position_error", 0.75);
    declare_parameter<double>("trajectory_seed_max_yaw_error", 0.5);
    declare_parameter<double>("control_lookahead", 0.1);
    declare_parameter<double>("wheel_base", 2.8);
    declare_parameter<double>("max_steer", 0.4);
    declare_parameter<double>("max_linear_velocity", 3.0);
    declare_parameter<double>("min_linear_velocity", -1.0);
    declare_parameter<double>("vehicle_width", 1.9);
    declare_parameter<double>("vehicle_length", 4.88);
    declare_parameter<double>("front_suspension", 0.93);
    declare_parameter<double>("rear_suspension", 1.10);
    declare_parameter<double>("d_cr", 1.34);
    declare_parameter<double>("max_longitudinal_acc", 1.0);
    declare_parameter<double>("max_lateral_acc", 1.2);
    declare_parameter<std::string>("arena_info_topic", "");
    declare_parameter<std::string>("arena_info_static_topic", "/epsilon/arena_info_static");
    declare_parameter<std::string>("arena_info_dynamic_topic", "/epsilon/arena_info_dynamic");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("control_signal_topic", "/epsilon/control_signal");
    declare_parameter<std::string>("cmd_vel_topic", "/epsilon/cmd_vel_raw");
    declare_parameter<std::string>("status_topic", "/epsilon/status");
    declare_parameter<bool>("enable_external_predictions", true);
    declare_parameter<std::string>("predicted_trajectories_topic", "/epsilon/predicted_trajectories");
    declare_parameter<double>("external_prediction_max_age", 2.0);
    declare_parameter<double>("external_prediction_min_probability", 0.0);
    declare_parameter<bool>("predicted_position_is_rear_axle", false);
  }

  void ReadParameters()
  {
    ego_id_ = get_parameter("ego_id").as_int();
    map_frame_ = get_parameter("map_frame").as_string();
    work_rate_ = get_parameter("work_rate").as_double();
    dynamic_scene_timeout_ = get_parameter("dynamic_scene_timeout").as_double();
    desired_velocity_ = get_parameter("desired_velocity").as_double();
    preferred_lateral_behavior_ = get_parameter("preferred_lateral_behavior").as_int();
    is_under_control_ = get_parameter("is_under_control").as_bool();
    override_ego_with_odom_ = get_parameter("override_ego_with_odom").as_bool();
    odom_pose_is_rear_axle_ = get_parameter("odom_pose_is_rear_axle").as_bool();
    use_previous_trajectory_initial_state_ =
      get_parameter("use_previous_trajectory_initial_state").as_bool();
    trajectory_seed_max_position_error_ =
      get_parameter("trajectory_seed_max_position_error").as_double();
    trajectory_seed_max_yaw_error_ = get_parameter("trajectory_seed_max_yaw_error").as_double();
    control_lookahead_ = get_parameter("control_lookahead").as_double();
    wheel_base_ = get_parameter("wheel_base").as_double();
    max_steer_ = get_parameter("max_steer").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
    min_linear_velocity_ = get_parameter("min_linear_velocity").as_double();

    default_vehicle_param_.set_width(get_parameter("vehicle_width").as_double());
    default_vehicle_param_.set_length(get_parameter("vehicle_length").as_double());
    default_vehicle_param_.set_wheel_base(wheel_base_);
    default_vehicle_param_.set_front_suspension(get_parameter("front_suspension").as_double());
    default_vehicle_param_.set_rear_suspension(get_parameter("rear_suspension").as_double());
    default_vehicle_param_.set_max_steering_angle(max_steer_);
    default_vehicle_param_.set_max_longitudinal_acc(
      get_parameter("max_longitudinal_acc").as_double());
    default_vehicle_param_.set_max_lateral_acc(get_parameter("max_lateral_acc").as_double());
    default_vehicle_param_.set_d_cr(get_parameter("d_cr").as_double());

    arena_info_topic_ = get_parameter("arena_info_topic").as_string();
    arena_info_static_topic_ = get_parameter("arena_info_static_topic").as_string();
    arena_info_dynamic_topic_ = get_parameter("arena_info_dynamic_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    control_signal_topic_ = get_parameter("control_signal_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    enable_external_predictions_ = get_parameter("enable_external_predictions").as_bool();
    predicted_trajectories_topic_ = get_parameter("predicted_trajectories_topic").as_string();
    external_prediction_max_age_ = get_parameter("external_prediction_max_age").as_double();
    external_prediction_min_probability_ =
      get_parameter("external_prediction_min_probability").as_double();
    predicted_position_is_rear_axle_ = get_parameter("predicted_position_is_rear_axle").as_bool();

    agent_config_path_ = get_parameter("agent_config_path").as_string();
    if (agent_config_path_.empty()) {
      agent_config_path_ = DefaultShareFile("epsilon_core", "config/agent_config.json");
    }
    eudm_config_path_ = get_parameter("eudm_config_path").as_string();
    if (eudm_config_path_.empty()) {
      eudm_config_path_ = DefaultShareFile("epsilon_core", "config/eudm_config.pb.txt");
    }
    ssc_config_path_ = get_parameter("ssc_config_path").as_string();
    if (ssc_config_path_.empty()) {
      ssc_config_path_ = DefaultShareFile("epsilon_core", "config/ssc_config.pb.txt");
    }
  }

  void ConfigureCore()
  {
    semantic_map_manager_ =
      std::make_unique<semantic_map_manager::SemanticMapManager>(ego_id_, agent_config_path_);
    data_renderer_ =
      std::make_unique<semantic_map_manager::DataRenderer>(semantic_map_manager_.get());

    eudm_manager_.Init(eudm_config_path_, work_rate_);
    ssc_planner_.Init(ssc_config_path_);
    ssc_planner_.set_map_interface(&ssc_adapter_);

    RCLCPP_INFO(
      get_logger(), "EPSILON core ready: ego_id=%d, agent_config=%s",
      ego_id_, agent_config_path_.c_str());
  }

  void ConfigureRosInterfaces()
  {
    const auto dynamic_qos = rclcpp::SensorDataQoS();
    const auto static_qos = rclcpp::QoS(1).transient_local().reliable();

    if (!arena_info_topic_.empty()) {
      arena_info_sub_ = create_subscription<vehicle_msgs::msg::ArenaInfo>(
        arena_info_topic_, dynamic_qos,
        std::bind(&EpsilonIntegratedPlannerNode::ArenaInfoCallback, this, std::placeholders::_1));
    }

    if (!arena_info_static_topic_.empty()) {
      arena_info_static_sub_ = create_subscription<vehicle_msgs::msg::ArenaInfoStatic>(
        arena_info_static_topic_, static_qos,
        std::bind(
          &EpsilonIntegratedPlannerNode::ArenaInfoStaticCallback, this, std::placeholders::_1));
    }

    if (!arena_info_dynamic_topic_.empty()) {
      arena_info_dynamic_sub_ = create_subscription<vehicle_msgs::msg::ArenaInfoDynamic>(
        arena_info_dynamic_topic_, dynamic_qos,
        std::bind(
          &EpsilonIntegratedPlannerNode::ArenaInfoDynamicCallback, this, std::placeholders::_1));
    }

    if (!odom_topic_.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, dynamic_qos,
        std::bind(&EpsilonIntegratedPlannerNode::OdomCallback, this, std::placeholders::_1));
    }

    if (enable_external_predictions_ && !predicted_trajectories_topic_.empty()) {
      predicted_trajectories_sub_ =
        create_subscription<vehicle_msgs::msg::PredictedTrajectoryArray>(
        predicted_trajectories_topic_, dynamic_qos,
        std::bind(
          &EpsilonIntegratedPlannerNode::PredictedTrajectoriesCallback, this,
          std::placeholders::_1));
    }

    control_signal_pub_ =
      create_publisher<vehicle_msgs::msg::ControlSignal>(control_signal_topic_, 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    planning_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(work_rate_, 1.0)),
      std::bind(&EpsilonIntegratedPlannerNode::PlanningTimerCallback, this));
  }

  void ArenaInfoStaticCallback(const vehicle_msgs::msg::ArenaInfoStatic::SharedPtr msg)
  {
    lane_net_ = ToLaneNet(msg->lane_net);
    obstacle_set_ = ToObstacleSet(msg->obstacle_set);
    has_static_scene_ = true;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
  }

  void ArenaInfoDynamicCallback(const vehicle_msgs::msg::ArenaInfoDynamic::SharedPtr msg)
  {
    latest_dynamic_scene_ = *msg;
    latest_scene_wall_time_ = std::chrono::steady_clock::now();
  }

  void ArenaInfoCallback(const vehicle_msgs::msg::ArenaInfo::SharedPtr msg)
  {
    lane_net_ = ToLaneNet(msg->lane_net);
    obstacle_set_ = ToObstacleSet(msg->obstacle_set);
    has_static_scene_ = true;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
    latest_combined_scene_ = *msg;
    latest_scene_wall_time_ = std::chrono::steady_clock::now();
  }

  void PredictedTrajectoriesCallback(
    const vehicle_msgs::msg::PredictedTrajectoryArray::SharedPtr msg)
  {
    latest_predicted_trajectories_ = *msg;
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    common::State state;
    state.time_stamp = ResolveStamp(StampSeconds(msg->header.stamp));
    const auto yaw = YawFromQuaternion(msg->pose.pose.orientation);
    auto x = msg->pose.pose.position.x;
    auto y = msg->pose.pose.position.y;
    if (!odom_pose_is_rear_axle_) {
      x -= default_vehicle_param_.d_cr() * std::cos(yaw);
      y -= default_vehicle_param_.d_cr() * std::sin(yaw);
    }

    state.vec_position[0] = x;
    state.vec_position[1] = y;
    state.angle = yaw;
    state.velocity = msg->twist.twist.linear.x;
    state.curvature =
      std::abs(state.velocity) > 1e-3 ? msg->twist.twist.angular.z / state.velocity : 0.0;
    state.steer = std::clamp(std::atan(wheel_base_ * state.curvature), -max_steer_, max_steer_);

    if (latest_ego_state_.has_value()) {
      const auto dt = state.time_stamp - latest_ego_state_->time_stamp;
      if (dt > 1e-3) {
        state.acceleration = (state.velocity - latest_ego_state_->velocity) / dt;
      }
    }
    latest_ego_state_ = state;
  }

  void PlanningTimerCallback()
  {
    if (latest_dynamic_scene_.has_value()) {
      const auto stamp = ResolveStamp(StampSeconds(latest_dynamic_scene_->header.stamp));
      if (stamp > last_processed_scene_stamp_ + 1e-6 ||
        (last_processed_scene_stamp_ > 0.0 && stamp + 1.0 < last_processed_scene_stamp_))
      {
        last_processed_scene_stamp_ = stamp;
        last_planning_cycle_success_ = false;
        RunPlanningCycle(stamp, ToVehicleSet(latest_dynamic_scene_->vehicle_set, default_vehicle_param_));
        return;
      }
    } else if (latest_combined_scene_.has_value()) {
      const auto stamp = ResolveStamp(StampSeconds(latest_combined_scene_->header.stamp));
      if (stamp > last_processed_scene_stamp_ + 1e-6 ||
        (last_processed_scene_stamp_ > 0.0 && stamp + 1.0 < last_processed_scene_stamp_))
      {
        lane_net_ = ToLaneNet(latest_combined_scene_->lane_net);
        obstacle_set_ = ToObstacleSet(latest_combined_scene_->obstacle_set);
        has_static_scene_ = true;
        last_processed_scene_stamp_ = stamp;
        last_planning_cycle_success_ = false;
        RunPlanningCycle(
          stamp, ToVehicleSet(latest_combined_scene_->vehicle_set, default_vehicle_param_));
        return;
      }
    }

    if (!last_planning_cycle_success_ || !last_trajectory_.get() ||
      !last_trajectory_->IsValid() || !last_behavior_.has_value() ||
      !latest_scene_wall_time_.has_value())
    {
      PublishSafeStop("degraded waiting for valid planning state");
      return;
    }

    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *latest_scene_wall_time_).count();
    if (dynamic_scene_timeout_ > 0.0 && age > dynamic_scene_timeout_) {
      PublishSafeStop("degraded dynamic scene is stale");
      return;
    }

    PublishTrajectoryControl(ResolveStamp(now().seconds()), *last_behavior_);
  }

  double ResolveStamp(double stamp) const
  {
    if (stamp > 0.0) {
      return stamp;
    }
    return now().seconds();
  }

  void MergeEgoState(common::VehicleSet * vehicle_set)
  {
    if (!latest_ego_state_.has_value()) {
      return;
    }
    if (!override_ego_with_odom_ && vehicle_set->vehicles.count(ego_id_) > 0) {
      return;
    }

    common::Vehicle ego_vehicle;
    auto existing = vehicle_set->vehicles.find(ego_id_);
    if (existing != vehicle_set->vehicles.end()) {
      ego_vehicle = existing->second;
    } else {
      ego_vehicle.set_id(ego_id_);
      ego_vehicle.set_subclass("ego");
      ego_vehicle.set_type("car");
      ego_vehicle.set_param(default_vehicle_param_);
    }
    ego_vehicle.set_state(*latest_ego_state_);
    vehicle_set->vehicles[ego_id_] = ego_vehicle;
  }

  void RunPlanningCycle(double stamp, common::VehicleSet vehicle_set)
  {
    RefreshRuntimeTaskParameters();

    if (!has_static_scene_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for ArenaInfoStatic or ArenaInfo");
      PublishSafeStop("degraded waiting for ArenaInfoStatic or ArenaInfo");
      return;
    }
    if (lane_net_.lane_set.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lane net is empty");
      PublishSafeStop("degraded lane net is empty");
      return;
    }

    MergeEgoState(&vehicle_set);
    if (vehicle_set.vehicles.count(ego_id_) == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ego vehicle id %d is missing", ego_id_);
      PublishSafeStop("degraded ego vehicle is missing");
      return;
    }

    if (data_renderer_->Render(stamp, lane_net_, vehicle_set, obstacle_set_) != kSuccess) {
      PublishSafeStop("degraded data renderer failed");
      return;
    }

    auto map_for_eudm =
      std::make_shared<semantic_map_manager::SemanticMapManager>(*semantic_map_manager_);
    const auto eudm_stamp = QuantizedStamp(stamp);
    auto task = MakeTask();
    if (eudm_manager_.Run(eudm_stamp, map_for_eudm, task) != kSuccess) {
      PublishSafeStop("degraded eudm failed");
      return;
    }

    common::SemanticBehavior behavior;
    eudm_manager_.ConstructBehavior(&behavior);
    ApplyExternalPredictions(stamp, &behavior);
    semantic_map_manager_->set_ego_behavior(behavior);

    auto map_for_ssc =
      std::make_shared<semantic_map_manager::SemanticMapManager>(*semantic_map_manager_);
    ssc_adapter_.set_map(map_for_ssc);
    MaybeSeedSscInitialState(stamp);

    if (ssc_planner_.RunOnce() != kSuccess) {
      PublishSafeStop("degraded ssc failed");
      return;
    }

    auto trajectory = ssc_planner_.trajectory();
    if (trajectory == nullptr || !trajectory->IsValid()) {
      PublishSafeStop("degraded ssc returned invalid trajectory");
      return;
    }

    last_trajectory_ = std::move(trajectory);
    last_behavior_ = behavior;
    last_planning_cycle_success_ = true;
    PublishTrajectoryControl(stamp, behavior);
  }

  void PublishTrajectoryControl(double stamp, const common::SemanticBehavior & behavior)
  {
    if (!last_trajectory_ || !last_trajectory_->IsValid()) {
      return;
    }

    common::State desired_state;
    const auto sample_time =
      std::clamp(stamp + control_lookahead_, last_trajectory_->begin(), last_trajectory_->end());
    if (last_trajectory_->GetState(sample_time, &desired_state) != kSuccess) {
      PublishSafeStop("degraded trajectory sampling failed");
      return;
    }
    PublishControl(desired_state, behavior);
  }

  void ApplyExternalPredictions(double stamp, common::SemanticBehavior * behavior)
  {
    last_external_prediction_count_ = 0;
    if (!enable_external_predictions_ || !latest_predicted_trajectories_.has_value()) {
      return;
    }

    const auto prediction_stamp = ResolveStamp(StampSeconds(latest_predicted_trajectories_->header.stamp));
    const auto prediction_age = stamp - prediction_stamp;
    if ((external_prediction_max_age_ > 0.0 && prediction_age > external_prediction_max_age_) ||
      prediction_age < -0.2)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "external predictions are stale: age=%.3fs max=%.3fs",
        prediction_age, external_prediction_max_age_);
      return;
    }

    auto external_trajs = ToPredictedTrajectoryMap(
      *latest_predicted_trajectories_, default_vehicle_param_, stamp,
      predicted_position_is_rear_axle_, external_prediction_min_probability_, ego_id_);
    if (external_trajs.empty()) {
      return;
    }

    last_external_prediction_count_ = static_cast<int>(external_trajs.size());
    if (behavior->surround_trajs.empty()) {
      behavior->surround_trajs.emplace_back(std::move(external_trajs));
      return;
    }

    for (auto & candidate_surround_trajs : behavior->surround_trajs) {
      for (const auto & entry : external_trajs) {
        candidate_surround_trajs[entry.first] = entry.second;
      }
    }
  }

  void RefreshRuntimeTaskParameters()
  {
    desired_velocity_ = get_parameter("desired_velocity").as_double();
    preferred_lateral_behavior_ = get_parameter("preferred_lateral_behavior").as_int();
    is_under_control_ = get_parameter("is_under_control").as_bool();
    control_lookahead_ = get_parameter("control_lookahead").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
    min_linear_velocity_ = get_parameter("min_linear_velocity").as_double();
    enable_external_predictions_ = get_parameter("enable_external_predictions").as_bool();
    external_prediction_max_age_ = get_parameter("external_prediction_max_age").as_double();
    external_prediction_min_probability_ =
      get_parameter("external_prediction_min_probability").as_double();
    predicted_position_is_rear_axle_ = get_parameter("predicted_position_is_rear_axle").as_bool();
  }

  planning::eudm::Task MakeTask() const
  {
    planning::eudm::Task task;
    task.is_under_ctrl = is_under_control_;
    task.user_desired_vel = desired_velocity_;
    task.user_perferred_behavior = preferred_lateral_behavior_;
    return task;
  }

  double QuantizedStamp(double stamp) const
  {
    const auto duration = 1.0 / std::max(work_rate_, 1.0);
    return std::floor(stamp / duration) * duration;
  }

  void MaybeSeedSscInitialState(double stamp)
  {
    if (!use_previous_trajectory_initial_state_) {
      return;
    }
    if (!latest_ego_state_.has_value()) {
      return;
    }
    if (last_trajectory_ == nullptr || !last_trajectory_->IsValid()) {
      return;
    }
    const auto sample_time =
      std::clamp(stamp + control_lookahead_, last_trajectory_->begin(), last_trajectory_->end());
    common::State initial_state;
    if (last_trajectory_->GetState(sample_time, &initial_state) == kSuccess) {
      const auto position_error = std::hypot(
        initial_state.vec_position[0] - latest_ego_state_->vec_position[0],
        initial_state.vec_position[1] - latest_ego_state_->vec_position[1]);
      const auto yaw_error = AngleDifference(initial_state.angle, latest_ego_state_->angle);
      if ((trajectory_seed_max_position_error_ > 0.0 &&
        position_error > trajectory_seed_max_position_error_) ||
        (trajectory_seed_max_yaw_error_ > 0.0 && yaw_error > trajectory_seed_max_yaw_error_))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "discarding stale SSC trajectory seed: position_error=%.3fm yaw_error=%.3frad",
          position_error, yaw_error);
        return;
      }
      ssc_planner_.set_initial_state(initial_state);
    }
  }

  void PublishControl(
    const common::State & desired_state, const common::SemanticBehavior & behavior)
  {
    const auto stamp = now();
    vehicle_msgs::msg::ControlSignal control_msg;
    control_msg.header.stamp = stamp;
    control_msg.header.frame_id = map_frame_;
    control_msg.acc = desired_state.acceleration;
    control_msg.steer_rate = 0.0;
    control_msg.is_openloop.data = true;
    FillStateMsg(desired_state, stamp, map_frame_, &control_msg.state);
    control_signal_pub_->publish(control_msg);

    geometry_msgs::msg::Twist twist_msg;
    const auto velocity =
      std::clamp(desired_state.velocity, min_linear_velocity_, max_linear_velocity_);
    const auto steer = std::clamp(desired_state.steer, -max_steer_, max_steer_);
    twist_msg.linear.x = velocity;
    twist_msg.angular.z = velocity * std::tan(steer) / std::max(wheel_base_, 1e-3);
    cmd_vel_pub_->publish(twist_msg);

    std::ostringstream status;
    status << "ok behavior=" << LatBehaviorName(behavior.lat_behavior)
           << " v=" << velocity << " steer=" << steer;
    if (last_external_prediction_count_ > 0) {
      status << " external_predictions=" << last_external_prediction_count_;
    }
    PublishStatus(status.str());
  }

  void PublishSafeStop(const std::string & text)
  {
    const auto stamp = now();
    vehicle_msgs::msg::ControlSignal control_msg;
    control_msg.header.stamp = stamp;
    control_msg.header.frame_id = map_frame_;
    control_msg.acc = 0.0;
    control_msg.steer_rate = 0.0;
    control_msg.is_openloop.data = true;

    common::State stop_state;
    if (latest_ego_state_.has_value()) {
      stop_state = *latest_ego_state_;
    }
    stop_state.velocity = 0.0;
    stop_state.acceleration = 0.0;
    stop_state.curvature = 0.0;
    stop_state.steer = 0.0;
    FillStateMsg(stop_state, stamp, map_frame_, &control_msg.state);
    control_signal_pub_->publish(control_msg);

    geometry_msgs::msg::Twist twist_msg;
    cmd_vel_pub_->publish(twist_msg);

    // A stop caused by missing/invalid planning data is not a healthy EPSILON
    // command.  Marking it as ok makes the command mux select this zero Twist
    // instead of falling back to a usable Nav2 command.
    PublishStatus(text);
  }

  void PublishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  int ego_id_{0};
  std::string map_frame_{"map"};
  std::string agent_config_path_;
  std::string eudm_config_path_;
  std::string ssc_config_path_;
  double work_rate_{20.0};
  double dynamic_scene_timeout_{1.5};
  double desired_velocity_{1.5};
  int preferred_lateral_behavior_{0};
  bool is_under_control_{true};
  bool override_ego_with_odom_{true};
  bool odom_pose_is_rear_axle_{false};
  bool use_previous_trajectory_initial_state_{true};
  double trajectory_seed_max_position_error_{0.75};
  double trajectory_seed_max_yaw_error_{0.5};
  double control_lookahead_{0.1};
  double wheel_base_{2.8};
  double max_steer_{0.4};
  double max_linear_velocity_{3.0};
  double min_linear_velocity_{-1.0};
  std::string arena_info_topic_;
  std::string arena_info_static_topic_;
  std::string arena_info_dynamic_topic_;
  std::string odom_topic_;
  std::string control_signal_topic_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  bool enable_external_predictions_{true};
  std::string predicted_trajectories_topic_{"/epsilon/predicted_trajectories"};
  double external_prediction_max_age_{2.0};
  double external_prediction_min_probability_{0.0};
  bool predicted_position_is_rear_axle_{false};
  int last_external_prediction_count_{0};

  common::VehicleParam default_vehicle_param_;
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;
  bool has_static_scene_{false};
  std::optional<common::State> latest_ego_state_;
  std::optional<vehicle_msgs::msg::ArenaInfoDynamic> latest_dynamic_scene_;
  std::optional<vehicle_msgs::msg::ArenaInfo> latest_combined_scene_;
  std::optional<vehicle_msgs::msg::PredictedTrajectoryArray> latest_predicted_trajectories_;
  std::optional<common::SemanticBehavior> last_behavior_;
  std::optional<std::chrono::steady_clock::time_point> latest_scene_wall_time_;
  double last_processed_scene_stamp_{0.0};
  bool last_planning_cycle_success_{false};

  std::unique_ptr<semantic_map_manager::SemanticMapManager> semantic_map_manager_;
  std::unique_ptr<semantic_map_manager::DataRenderer> data_renderer_;
  planning::EudmManager eudm_manager_;
  planning::SscPlannerAdapter ssc_adapter_;
  planning::SscPlanner ssc_planner_;
  std::unique_ptr<common::FrenetTrajectory> last_trajectory_;

  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfo>::SharedPtr arena_info_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr arena_info_static_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr arena_info_dynamic_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::PredictedTrajectoryArray>::SharedPtr
    predicted_trajectories_sub_;
  rclcpp::Publisher<vehicle_msgs::msg::ControlSignal>::SharedPtr control_signal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EpsilonIntegratedPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
