#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"

namespace {

double YawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double Distance2D(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

geometry_msgs::msg::Point OffsetPoint(
  const std::vector<geometry_msgs::msg::Point> & points, std::size_t index, double offset)
{
  const auto last = points.size() - 1;
  const auto prev = index == 0 ? index : index - 1;
  const auto next = index == last ? index : index + 1;
  auto tangent_x = points[next].x - points[prev].x;
  auto tangent_y = points[next].y - points[prev].y;
  const auto norm = std::hypot(tangent_x, tangent_y);
  geometry_msgs::msg::Point out = points[index];
  if (norm < 1e-6) {
    return out;
  }
  tangent_x /= norm;
  tangent_y /= norm;
  const auto left_x = -tangent_y;
  const auto left_y = tangent_x;
  out.x += left_x * offset;
  out.y += left_y * offset;
  return out;
}

double PathLength(const std::vector<geometry_msgs::msg::Point> & points)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += Distance2D(points[i - 1], points[i]);
  }
  return length;
}

vehicle_msgs::msg::Lane MakeLane(
  const std_msgs::msg::Header & header, int id,
  const std::vector<geometry_msgs::msg::Point> & points)
{
  vehicle_msgs::msg::Lane lane;
  lane.header = header;
  lane.id = id;
  lane.dir = 1;
  lane.l_lane_id = -1;
  lane.r_lane_id = -1;
  lane.l_change_avbl = false;
  lane.r_change_avbl = false;
  lane.behavior = "normal";
  lane.points = points;
  lane.length = static_cast<float>(PathLength(points));
  if (!points.empty()) {
    lane.start_point = points.front();
    lane.final_point = points.back();
  }
  return lane;
}

geometry_msgs::msg::Point OccupancyCellCenter(
  const nav_msgs::msg::OccupancyGrid & grid, unsigned int x, unsigned int y)
{
  const auto resolution = static_cast<double>(grid.info.resolution);
  const auto local_x = (static_cast<double>(x) + 0.5) * resolution;
  const auto local_y = (static_cast<double>(y) + 0.5) * resolution;
  const auto yaw = YawFromQuaternion(grid.info.origin.orientation);
  const auto cos_yaw = std::cos(yaw);
  const auto sin_yaw = std::sin(yaw);

  geometry_msgs::msg::Point point;
  point.x = grid.info.origin.position.x + cos_yaw * local_x - sin_yaw * local_y;
  point.y = grid.info.origin.position.y + sin_yaw * local_x + cos_yaw * local_y;
  point.z = 0.0;
  return point;
}

double StampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

class EpsilonSceneBridgeNode : public rclcpp::Node
{
public:
  EpsilonSceneBridgeNode()
  : Node("epsilon_scene_bridge_node")
  {
    DeclareParameters();
    ReadParameters();
    ConfigureRosInterfaces();
  }

private:
  void DeclareParameters()
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("path_topic", "/epsilon/reference_path");
    declare_parameter<std::string>("costmap_topic", "/projected_costmap");
    declare_parameter<std::string>("object_pose_topic", "/gen0_perception/trash_poses");
    declare_parameter<std::string>("actor_pose_topics", "");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("arena_info_static_topic", "/epsilon/arena_info_static");
    declare_parameter<std::string>("arena_info_dynamic_topic", "/epsilon/arena_info_dynamic");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("dynamic_publish_period", 0.1);
    declare_parameter<double>("path_timeout", 1.5);
    declare_parameter<int>("ego_id", 0);
    declare_parameter<double>("min_lane_point_spacing", 0.3);
    declare_parameter<bool>("centerline_is_road_center", false);
    declare_parameter<int>("virtual_lane_count", 1);
    declare_parameter<double>("lane_width", 3.2);
    declare_parameter<bool>("allow_opposite_lane_change", false);
    declare_parameter<int>("occupied_threshold", 65);
    declare_parameter<bool>("treat_unknown_as_obstacle", false);
    declare_parameter<int>("max_obstacle_cells", 2500);
    declare_parameter<double>("grid_obstacle_radius", 0.12);
    declare_parameter<double>("object_obstacle_radius", 0.35);
    declare_parameter<bool>("odom_pose_is_rear_axle", false);
    declare_parameter<bool>("shift_path_to_rear_axle", true);
    declare_parameter<double>("wheel_base", 2.8);
    declare_parameter<double>("max_steer", 0.4);
    declare_parameter<double>("vehicle_width", 1.9);
    declare_parameter<double>("vehicle_length", 4.88);
    declare_parameter<double>("front_suspension", 0.93);
    declare_parameter<double>("rear_suspension", 1.10);
    declare_parameter<double>("d_cr", 1.34);
    declare_parameter<double>("max_longitudinal_acc", 1.0);
    declare_parameter<double>("max_lateral_acc", 1.2);
  }

  void ReadParameters()
  {
    map_frame_ = get_parameter("map_frame").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    costmap_topic_ = get_parameter("costmap_topic").as_string();
    object_pose_topic_ = get_parameter("object_pose_topic").as_string();
    actor_pose_topics_ = StringListParameter("actor_pose_topics");
    odom_topic_ = get_parameter("odom_topic").as_string();
    arena_info_static_topic_ = get_parameter("arena_info_static_topic").as_string();
    arena_info_dynamic_topic_ = get_parameter("arena_info_dynamic_topic").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    dynamic_publish_period_ = get_parameter("dynamic_publish_period").as_double();
    path_timeout_ = get_parameter("path_timeout").as_double();
    ego_id_ = get_parameter("ego_id").as_int();
    min_lane_point_spacing_ = get_parameter("min_lane_point_spacing").as_double();
    centerline_is_road_center_ = get_parameter("centerline_is_road_center").as_bool();
    virtual_lane_count_ = get_parameter("virtual_lane_count").as_int();
    lane_width_ = get_parameter("lane_width").as_double();
    allow_opposite_lane_change_ = get_parameter("allow_opposite_lane_change").as_bool();
    occupied_threshold_ = get_parameter("occupied_threshold").as_int();
    treat_unknown_as_obstacle_ = get_parameter("treat_unknown_as_obstacle").as_bool();
    max_obstacle_cells_ = get_parameter("max_obstacle_cells").as_int();
    grid_obstacle_radius_ = get_parameter("grid_obstacle_radius").as_double();
    object_obstacle_radius_ = get_parameter("object_obstacle_radius").as_double();
    odom_pose_is_rear_axle_ = get_parameter("odom_pose_is_rear_axle").as_bool();
    shift_path_to_rear_axle_ = get_parameter("shift_path_to_rear_axle").as_bool();
    wheel_base_ = get_parameter("wheel_base").as_double();
    max_steer_ = get_parameter("max_steer").as_double();
    vehicle_width_ = get_parameter("vehicle_width").as_double();
    vehicle_length_ = get_parameter("vehicle_length").as_double();
    front_suspension_ = get_parameter("front_suspension").as_double();
    rear_suspension_ = get_parameter("rear_suspension").as_double();
    d_cr_ = get_parameter("d_cr").as_double();
    max_longitudinal_acc_ = get_parameter("max_longitudinal_acc").as_double();
    max_lateral_acc_ = get_parameter("max_lateral_acc").as_double();
  }

  void ConfigureRosInterfaces()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&EpsilonSceneBridgeNode::PathCallback, this, std::placeholders::_1));
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, map_qos,
      std::bind(&EpsilonSceneBridgeNode::CostmapCallback, this, std::placeholders::_1));
    object_pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      object_pose_topic_, sensor_qos,
      std::bind(&EpsilonSceneBridgeNode::ObjectPoseCallback, this, std::placeholders::_1));
    for (const auto & topic : actor_pose_topics_) {
      actor_pose_subs_.push_back(
        create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, sensor_qos,
          [this, topic](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            ActorPoseCallback(msg, topic);
          }));
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      std::bind(&EpsilonSceneBridgeNode::OdomCallback, this, std::placeholders::_1));

    static_pub_ =
      create_publisher<vehicle_msgs::msg::ArenaInfoStatic>(arena_info_static_topic_, map_qos);
    dynamic_pub_ =
      create_publisher<vehicle_msgs::msg::ArenaInfoDynamic>(
      arena_info_dynamic_topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, publish_period_)),
      std::bind(&EpsilonSceneBridgeNode::PublishStaticScene, this));
    dynamic_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.02, dynamic_publish_period_)),
      std::bind(&EpsilonSceneBridgeNode::PublishDynamicSceneTimer, this));

    RCLCPP_INFO(
      get_logger(), "EPSILON scene bridge: path=%s costmap=%s odom=%s actors=%zu",
      path_topic_.c_str(), costmap_topic_.c_str(), odom_topic_.c_str(), actor_pose_topics_.size());
  }

  void PathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_received_wall_time_ = std::chrono::steady_clock::now();
    std::vector<geometry_msgs::msg::Point> points;
    for (const auto & pose : msg->poses) {
      geometry_msgs::msg::Point point = pose.pose.position;
      point.z = 0.0;
      if (!points.empty() && Distance2D(points.back(), point) < min_lane_point_spacing_) {
        continue;
      }
      points.push_back(point);
    }
    if (points.size() < 2) {
      latest_path_points_.reset();
      PublishStaticScene();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "reference path needs at least two usable points");
      return;
    }

    // Nav2 paths are expressed at base_link while EPSILON's state is defined
    // at the rear axle. Keep the native EPSILON convention and shift only the
    // bridge-generated lane geometry when the input odometry is not rear-axle based.
    if (shift_path_to_rear_axle_ && !odom_pose_is_rear_axle_ && d_cr_ > 1e-6) {
      std::vector<geometry_msgs::msg::Point> rear_axle_points;
      rear_axle_points.reserve(points.size());
      const auto last = points.size() - 1;
      for (std::size_t i = 0; i < points.size(); ++i) {
        const auto prev = i == 0 ? i : i - 1;
        const auto next = i == last ? i : i + 1;
        const auto tangent_x = points[next].x - points[prev].x;
        const auto tangent_y = points[next].y - points[prev].y;
        const auto norm = std::hypot(tangent_x, tangent_y);
        auto shifted = points[i];
        if (norm > 1e-6) {
          shifted.x -= d_cr_ * tangent_x / norm;
          shifted.y -= d_cr_ * tangent_y / norm;
        }
        rear_axle_points.push_back(shifted);
      }
      points = std::move(rear_axle_points);
    }
    latest_path_points_ = points;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
    PublishStaticScene();
  }

  void CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_costmap_ = *msg;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
    PublishStaticScene();
  }

  void ObjectPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_object_poses_ = *msg;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
    PublishStaticScene();
  }

  void ActorPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const std::string & topic)
  {
    const auto actor_name = ActorNameFromTopic(topic);
    const auto previous_it = latest_actor_poses_.find(actor_name);
    std::optional<geometry_msgs::msg::PoseStamped> previous_sample;
    if (previous_it != latest_actor_poses_.end()) {
      previous_sample = previous_it->second;
    }
    const auto old_speed_it = actor_speeds_.find(actor_name);
    std::optional<double> old_speed;
    if (old_speed_it != actor_speeds_.end()) {
      old_speed = old_speed_it->second;
    }

    auto & sample = latest_actor_poses_[actor_name];
    sample = *msg;
    if (sample.header.stamp.sec == 0 && sample.header.stamp.nanosec == 0) {
      sample.header.stamp = now();
    }
    if (sample.header.frame_id.empty()) {
      sample.header.frame_id = map_frame_;
    }
    actor_headings_[actor_name] = YawFromQuaternion(sample.pose.orientation);
    if (previous_sample.has_value()) {
      const auto dt = StampSeconds(sample.header.stamp) - StampSeconds(previous_sample->header.stamp);
      if (dt > 1e-3) {
        const auto dx = sample.pose.position.x - previous_sample->pose.position.x;
        const auto dy = sample.pose.position.y - previous_sample->pose.position.y;
        const auto speed = std::hypot(dx, dy) / dt;
        if (speed > 1e-3) {
          actor_headings_[actor_name] = std::atan2(dy, dx);
        }
        if (old_speed.has_value()) {
          actor_accelerations_[actor_name] = (speed - *old_speed) / dt;
        }
        actor_speeds_[actor_name] = speed;
      }
    }
    dynamic_scene_dirty_ = true;
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void PublishDynamicSceneTimer()
  {
    if (!latest_odom_.has_value()) {
      return;
    }

    const auto stamp = StampSeconds(latest_odom_->header.stamp);
    if (!dynamic_scene_dirty_ && stamp > 0.0 && stamp <= last_dynamic_odom_stamp_ + 1e-6) {
      return;
    }
    if (stamp > 0.0) {
      last_dynamic_odom_stamp_ = stamp;
    }
    dynamic_scene_dirty_ = false;
    PublishDynamicScene(*latest_odom_);
  }

  void PublishDynamicScene(const nav_msgs::msg::Odometry & odom)
  {
    vehicle_msgs::msg::ArenaInfoDynamic dynamic_msg;
    dynamic_msg.header.stamp = odom.header.stamp;
    dynamic_msg.header.frame_id = map_frame_;
    dynamic_msg.vehicle_set.header = dynamic_msg.header;
    dynamic_msg.vehicle_set.vehicles.push_back(MakeEgoVehicle(odom));
    for (const auto & [actor_name, actor_pose] : latest_actor_poses_) {
      if (actor_name.empty()) {
        continue;
      }
      dynamic_msg.vehicle_set.vehicles.push_back(
        MakeActorVehicle(actor_name, actor_pose, odom.header, dynamic_msg.header.frame_id));
    }
    dynamic_pub_->publish(dynamic_msg);
  }

  vehicle_msgs::msg::Vehicle MakeEgoVehicle(const nav_msgs::msg::Odometry & odom) const
  {
    const auto yaw = YawFromQuaternion(odom.pose.pose.orientation);
    auto x = odom.pose.pose.position.x;
    auto y = odom.pose.pose.position.y;
    if (!odom_pose_is_rear_axle_) {
      x -= d_cr_ * std::cos(yaw);
      y -= d_cr_ * std::sin(yaw);
    }

    vehicle_msgs::msg::Vehicle vehicle;
    vehicle.header = odom.header;
    vehicle.header.frame_id = map_frame_;
    vehicle.id.data = ego_id_;
    vehicle.subclass.data = "ego";
    vehicle.type.data = "car";
    vehicle.param.width = static_cast<float>(vehicle_width_);
    vehicle.param.length = static_cast<float>(vehicle_length_);
    vehicle.param.wheel_base = static_cast<float>(wheel_base_);
    vehicle.param.front_suspension = static_cast<float>(front_suspension_);
    vehicle.param.rear_suspension = static_cast<float>(rear_suspension_);
    vehicle.param.max_steering_angle = static_cast<float>(max_steer_);
    vehicle.param.d_cr = static_cast<float>(d_cr_);
    vehicle.param.max_longitudinal_acc = static_cast<float>(max_longitudinal_acc_);
    vehicle.param.max_lateral_acc = static_cast<float>(max_lateral_acc_);
    vehicle.state.header = vehicle.header;
    vehicle.state.vec_position.x = x;
    vehicle.state.vec_position.y = y;
    vehicle.state.vec_position.z = 0.0;
    vehicle.state.angle = yaw;
    vehicle.state.velocity = odom.twist.twist.linear.x;
    vehicle.state.curvature =
      std::abs(vehicle.state.velocity) > 1e-3 ?
      odom.twist.twist.angular.z / vehicle.state.velocity : 0.0;
    vehicle.state.steer =
      std::clamp(std::atan(wheel_base_ * vehicle.state.curvature), -max_steer_, max_steer_);
    vehicle.state.acceleration = 0.0;
    return vehicle;
  }

  vehicle_msgs::msg::Vehicle MakeActorVehicle(
    const std::string & actor_name, const geometry_msgs::msg::PoseStamped & pose_sample,
    const std_msgs::msg::Header & header, const std::string & frame_id) const
  {
    vehicle_msgs::msg::Vehicle vehicle;
    vehicle.header = header;
    vehicle.header.frame_id = frame_id;
    vehicle.id.data = ActorIdFromName(actor_name);
    vehicle.subclass.data = actor_name;
    vehicle.type.data = "pedestrian";
    vehicle.param.width = 0.8F;
    vehicle.param.length = 0.8F;
    vehicle.param.wheel_base = 0.5F;
    vehicle.param.front_suspension = 0.25F;
    vehicle.param.rear_suspension = 0.25F;
    vehicle.param.max_steering_angle = 0.4F;
    vehicle.param.d_cr = 0.001F;
    vehicle.param.max_longitudinal_acc = 1.0F;
    vehicle.param.max_lateral_acc = 1.0F;
    vehicle.state.header = header;
    vehicle.state.header.frame_id = frame_id;
    vehicle.state.vec_position.x = pose_sample.pose.position.x;
    vehicle.state.vec_position.y = pose_sample.pose.position.y;
    vehicle.state.vec_position.z = 0.0;
    vehicle.state.angle = MapValueOr(actor_headings_, actor_name, YawFromQuaternion(pose_sample.pose.orientation));
    vehicle.state.velocity = MapValueOr(actor_speeds_, actor_name, 0.0);
    vehicle.state.curvature = 0.0;
    vehicle.state.steer = 0.0;
    vehicle.state.acceleration = MapValueOr(actor_accelerations_, actor_name, 0.0);
    return vehicle;
  }

  void PublishStaticScene()
  {
    if (latest_path_points_.has_value() && path_timeout_ > 0.0 &&
      latest_path_received_wall_time_.has_value())
    {
      const auto age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *latest_path_received_wall_time_).count();
      if (age > path_timeout_) {
        latest_path_points_.reset();
      }
    }

    if (!latest_path_points_.has_value()) {
      if (static_scene_active_) {
        PublishEmptyStaticScene();
        static_scene_active_ = false;
        RCLCPP_WARN(
          get_logger(), "reference path on %s expired; EPSILON static lane was invalidated",
          path_topic_.c_str());
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "waiting for reference path on %s",
        path_topic_.c_str());
      return;
    }

    vehicle_msgs::msg::ArenaInfoStatic msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.lane_net = BuildLaneNet(msg.header, *latest_path_points_);
    msg.obstacle_set = BuildObstacleSet(msg.header);
    static_pub_->publish(msg);
    static_scene_active_ = true;
    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "static scene published; waiting for odom on %s",
        odom_topic_.c_str());
    }
  }

  void PublishEmptyStaticScene()
  {
    vehicle_msgs::msg::ArenaInfoStatic msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.obstacle_set = BuildObstacleSet(msg.header);
    static_pub_->publish(msg);
  }

  vehicle_msgs::msg::LaneNet BuildLaneNet(
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & path_points) const
  {
    vehicle_msgs::msg::LaneNet lane_net;
    lane_net.header = header;

    if (!centerline_is_road_center_ || virtual_lane_count_ <= 1) {
      lane_net.lanes.push_back(MakeLane(header, 0, path_points));
      return lane_net;
    }

    const auto half_width = lane_width_ * 0.5;
    std::vector<geometry_msgs::msg::Point> right_lane_points;
    std::vector<geometry_msgs::msg::Point> left_lane_points;
    right_lane_points.reserve(path_points.size());
    left_lane_points.reserve(path_points.size());
    for (std::size_t i = 0; i < path_points.size(); ++i) {
      right_lane_points.push_back(OffsetPoint(path_points, i, -half_width));
      left_lane_points.push_back(OffsetPoint(path_points, i, half_width));
    }

    auto ego_direction_lane = MakeLane(header, 0, right_lane_points);
    auto opposite_lane_points = left_lane_points;
    std::reverse(opposite_lane_points.begin(), opposite_lane_points.end());
    auto opposite_lane = MakeLane(header, 1, opposite_lane_points);

    ego_direction_lane.l_lane_id = 1;
    ego_direction_lane.l_change_avbl = allow_opposite_lane_change_;
    opposite_lane.r_lane_id = 0;
    opposite_lane.r_change_avbl = allow_opposite_lane_change_;

    lane_net.lanes.push_back(ego_direction_lane);
    lane_net.lanes.push_back(opposite_lane);
    return lane_net;
  }

  vehicle_msgs::msg::ObstacleSet BuildObstacleSet(const std_msgs::msg::Header & header) const
  {
    vehicle_msgs::msg::ObstacleSet obstacle_set;
    obstacle_set.header = header;
    int obstacle_id = 0;

    if (latest_costmap_.has_value()) {
      AppendCostmapObstacles(*latest_costmap_, &obstacle_id, &obstacle_set);
    }
    if (latest_object_poses_.has_value()) {
      AppendPoseObstacles(*latest_object_poses_, &obstacle_id, &obstacle_set);
    }
    return obstacle_set;
  }

  void AppendCostmapObstacles(
    const nav_msgs::msg::OccupancyGrid & grid, int * obstacle_id,
    vehicle_msgs::msg::ObstacleSet * obstacle_set) const
  {
    if (grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0) {
      return;
    }

    int occupied_count = 0;
    for (const auto value : grid.data) {
      if (IsOccupied(value)) {
        ++occupied_count;
      }
    }
    const auto max_cells = std::max(1, max_obstacle_cells_);
    const auto stride = std::max(1, static_cast<int>(std::ceil(
      static_cast<double>(occupied_count) / static_cast<double>(max_cells))));

    int seen = 0;
    for (unsigned int y = 0; y < grid.info.height; ++y) {
      for (unsigned int x = 0; x < grid.info.width; ++x) {
        const auto idx = static_cast<std::size_t>(y) * grid.info.width + x;
        if (idx >= grid.data.size() || !IsOccupied(grid.data[idx])) {
          continue;
        }
        if ((seen++ % stride) != 0) {
          continue;
        }
        vehicle_msgs::msg::CircleObstacle obstacle;
        obstacle.header.stamp = grid.header.stamp;
        obstacle.header.frame_id = map_frame_;
        obstacle.id = (*obstacle_id)++;
        obstacle.circle.center = OccupancyCellCenter(grid, x, y);
        obstacle.circle.radius = static_cast<float>(
          std::max(grid_obstacle_radius_, static_cast<double>(grid.info.resolution) * 0.5));
        obstacle_set->obs_circle.push_back(obstacle);
        if (static_cast<int>(obstacle_set->obs_circle.size()) >= max_cells) {
          return;
        }
      }
    }
  }

  void AppendPoseObstacles(
    const geometry_msgs::msg::PoseArray & poses, int * obstacle_id,
    vehicle_msgs::msg::ObstacleSet * obstacle_set) const
  {
    for (const auto & pose : poses.poses) {
      vehicle_msgs::msg::CircleObstacle obstacle;
      obstacle.header = poses.header;
      obstacle.header.frame_id = map_frame_;
      obstacle.id = (*obstacle_id)++;
      obstacle.circle.center = pose.position;
      obstacle.circle.center.z = 0.0;
      obstacle.circle.radius = static_cast<float>(object_obstacle_radius_);
      obstacle_set->obs_circle.push_back(obstacle);
    }
  }

  bool IsOccupied(int value) const
  {
    if (value < 0) {
      return treat_unknown_as_obstacle_;
    }
    return value >= occupied_threshold_;
  }

  static std::string ActorNameFromTopic(const std::string & topic)
  {
    const auto slash = topic.find_last_of('/');
    if (slash == std::string::npos || slash == 0) {
      return topic;
    }
    const auto last = topic.substr(slash + 1);
    if (last == "pose" && slash > 0) {
      const auto prev = topic.find_last_of('/', slash - 1);
      if (prev != std::string::npos && prev + 1 < slash) {
        return topic.substr(prev + 1, slash - prev - 1);
      }
    }
    return last;
  }

  static int ActorIdFromName(const std::string & actor_name)
  {
    std::string digits;
    for (const auto ch : actor_name) {
      if (std::isdigit(static_cast<unsigned char>(ch))) {
        digits.push_back(ch);
      }
    }
    if (!digits.empty()) {
      try {
        return 1000 + std::stoi(digits);
      } catch (...) {
        return static_cast<int>(std::hash<std::string>{}(actor_name) & 0x7fffffff);
      }
    }
    return static_cast<int>(std::hash<std::string>{}(actor_name) & 0x7fffffff);
  }

  std::vector<std::string> StringListParameter(const std::string & name) const
  {
    const auto raw = get_parameter(name).as_string();
    std::vector<std::string> values;
    std::string current;
    for (const auto ch : raw) {
      if (ch == ',') {
        const auto trimmed = TrimString(current);
        if (!trimmed.empty()) {
          values.push_back(trimmed);
        }
        current.clear();
      } else {
        current.push_back(ch);
      }
    }
    const auto trimmed = TrimString(current);
    if (!trimmed.empty()) {
      values.push_back(trimmed);
    }
    return values;
  }

  static std::string TrimString(const std::string & value)
  {
    const auto begin = value.find_first_not_of(" \t\n\r");
    if (begin == std::string::npos) {
      return "";
    }
    const auto end = value.find_last_not_of(" \t\n\r");
    return value.substr(begin, end - begin + 1);
  }

  static double MapValueOr(
    const std::unordered_map<std::string, double> & values,
    const std::string & key, double fallback)
  {
    const auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
  }

  std::string map_frame_{"map"};
  std::string path_topic_;
  std::string costmap_topic_;
  std::string object_pose_topic_;
  std::vector<std::string> actor_pose_topics_;
  std::string odom_topic_;
  std::string arena_info_static_topic_;
  std::string arena_info_dynamic_topic_;
  double publish_period_{1.0};
  double dynamic_publish_period_{0.1};
  double path_timeout_{1.5};
  int ego_id_{0};
  double min_lane_point_spacing_{0.3};
  bool centerline_is_road_center_{false};
  int virtual_lane_count_{1};
  double lane_width_{3.2};
  bool allow_opposite_lane_change_{false};
  int occupied_threshold_{65};
  bool treat_unknown_as_obstacle_{false};
  int max_obstacle_cells_{2500};
  double grid_obstacle_radius_{0.12};
  double object_obstacle_radius_{0.35};
  bool odom_pose_is_rear_axle_{false};
  bool shift_path_to_rear_axle_{true};
  double wheel_base_{2.8};
  double max_steer_{0.4};
  double vehicle_width_{1.9};
  double vehicle_length_{4.88};
  double front_suspension_{0.93};
  double rear_suspension_{1.10};
  double d_cr_{1.34};
  double max_longitudinal_acc_{1.0};
  double max_lateral_acc_{1.2};

  std::optional<std::vector<geometry_msgs::msg::Point>> latest_path_points_;
  std::optional<std::chrono::steady_clock::time_point> latest_path_received_wall_time_;
  bool static_scene_active_{false};
  std::optional<nav_msgs::msg::OccupancyGrid> latest_costmap_;
  std::optional<geometry_msgs::msg::PoseArray> latest_object_poses_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> latest_actor_poses_;
  std::unordered_map<std::string, double> actor_headings_;
  std::unordered_map<std::string, double> actor_speeds_;
  std::unordered_map<std::string, double> actor_accelerations_;
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  bool has_odom_{false};
  double last_dynamic_odom_stamp_{0.0};
  bool dynamic_scene_dirty_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr object_pose_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> actor_pose_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr static_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr dynamic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr dynamic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EpsilonSceneBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
