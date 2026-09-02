#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>
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

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double AngleDifference(double a, double b)
{
  return std::fabs(NormalizeAngle(a - b));
}

struct SegmentProjection2D
{
  geometry_msgs::msg::Point point;
  double distance{std::numeric_limits<double>::infinity()};
  double t{0.0};
};

SegmentProjection2D ProjectPointToSegment2D(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end)
{
  SegmentProjection2D projection;
  const auto vx = end.x - start.x;
  const auto vy = end.y - start.y;
  const auto wx = point.x - start.x;
  const auto wy = point.y - start.y;
  const auto length_sq = vx * vx + vy * vy;
  if (length_sq <= 1e-12) {
    projection.point = start;
    projection.distance = Distance2D(point, start);
    return projection;
  }

  projection.t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
  projection.point.x = start.x + projection.t * vx;
  projection.point.y = start.y + projection.t * vy;
  projection.point.z = 0.0;
  projection.distance = Distance2D(point, projection.point);
  return projection;
}

double PointToSegmentDistance2D(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end)
{
  return ProjectPointToSegment2D(point, start, end).distance;
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
    declare_parameter<double>("path_timeout", 120.0);
    declare_parameter<int>("ego_id", 0);
    declare_parameter<double>("min_lane_point_spacing", 0.3);
    declare_parameter<bool>("centerline_is_road_center", true);
    declare_parameter<int>("virtual_lane_count", 3);
    declare_parameter<double>("lane_width", 3.2);
    declare_parameter<bool>("allow_opposite_lane_change", false);
    declare_parameter<int>("occupied_threshold", 65);
    declare_parameter<bool>("treat_unknown_as_obstacle", false);
    declare_parameter<int>("max_obstacle_cells", 2500);
    declare_parameter<double>("grid_obstacle_radius", 0.12);
    declare_parameter<double>("object_obstacle_radius", 0.35);
    declare_parameter<bool>("filter_dynamic_actors_by_lane", true);
    declare_parameter<double>("actor_lane_max_distance", 4.0);
    declare_parameter<bool>("align_path_to_ego", true);
    declare_parameter<double>("path_ego_max_heading_error", 1.35);
    declare_parameter<double>("path_ego_max_snap_distance", 8.0);
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
    filter_dynamic_actors_by_lane_ = get_parameter("filter_dynamic_actors_by_lane").as_bool();
    actor_lane_max_distance_ = get_parameter("actor_lane_max_distance").as_double();
    align_path_to_ego_ = get_parameter("align_path_to_ego").as_bool();
    path_ego_max_heading_error_ = get_parameter("path_ego_max_heading_error").as_double();
    path_ego_max_snap_distance_ = get_parameter("path_ego_max_snap_distance").as_double();
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
      path_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&EpsilonSceneBridgeNode::PathCallback, this, std::placeholders::_1));
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).transient_local().reliable(),
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
      PublishStaticScene();
      if (latest_path_points_.has_value()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ignoring degenerate reference path on %s: raw_poses=%zu usable_points=%zu; "
          "keeping previous lane",
          path_topic_.c_str(), msg->poses.size(), points.size());
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "reference path on %s needs at least two usable points: raw_poses=%zu usable_points=%zu",
          path_topic_.c_str(), msg->poses.size(), points.size());
      }
      return;
    }

    latest_path_received_wall_time_ = std::chrono::steady_clock::now();
    path_stale_warned_ = false;

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
    points = AlignPathToEgo(points);
    latest_path_points_ = points;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
    PublishStaticScene();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "accepted reference path on %s: raw_poses=%zu usable_points=%zu frame=%s",
      path_topic_.c_str(), msg->poses.size(), points.size(), map_frame_.c_str());
  }

  void CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_costmap_ = *msg;
    if (HasFrameMismatch(msg->header.frame_id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "costmap frame '%s' does not match EPSILON scene frame '%s'; keeping the "
        "reference path frame and ignoring mismatched costmap obstacles until frames match",
        msg->header.frame_id.c_str(), map_frame_.c_str());
    }
    PublishStaticScene();
  }

  void ObjectPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_object_poses_ = *msg;
    if (HasFrameMismatch(msg->header.frame_id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "object pose frame '%s' does not match EPSILON scene frame '%s'; keeping the "
        "reference path frame and ignoring mismatched object obstacles until frames match",
        msg->header.frame_id.c_str(), map_frame_.c_str());
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

  struct RearAxlePose
  {
    geometry_msgs::msg::Point position;
    double yaw{0.0};
  };

  RearAxlePose RearAxlePoseFromOdom(const nav_msgs::msg::Odometry & odom) const
  {
    RearAxlePose pose;
    pose.yaw = YawFromQuaternion(odom.pose.pose.orientation);
    pose.position = odom.pose.pose.position;
    pose.position.z = 0.0;
    if (!odom_pose_is_rear_axle_) {
      pose.position.x -= d_cr_ * std::cos(pose.yaw);
      pose.position.y -= d_cr_ * std::sin(pose.yaw);
    }
    return pose;
  }

  std::vector<geometry_msgs::msg::Point> AlignPathToEgo(
    const std::vector<geometry_msgs::msg::Point> & points)
  {
    if (!align_path_to_ego_ || points.size() < 2 || !latest_odom_.has_value()) {
      return points;
    }

    const auto ego = RearAxlePoseFromOdom(*latest_odom_);
    const auto max_heading_error =
      std::clamp(path_ego_max_heading_error_, 0.0, std::acos(-1.0));
    const auto max_snap_distance = std::max(0.0, path_ego_max_snap_distance_);

    bool found = false;
    std::size_t best_segment = 0;
    SegmentProjection2D best_projection;
    double best_yaw_error = std::numeric_limits<double>::infinity();
    double best_longitudinal = 0.0;
    double best_score = std::numeric_limits<double>::infinity();

    bool nearest_valid = false;
    std::size_t nearest_segment = 0;
    SegmentProjection2D nearest_projection;
    double nearest_yaw_error = std::numeric_limits<double>::infinity();

    const auto ego_heading_x = std::cos(ego.yaw);
    const auto ego_heading_y = std::sin(ego.yaw);
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const auto & start = points[i];
      const auto & end = points[i + 1];
      const auto segment_length = Distance2D(start, end);
      if (segment_length < 1e-6) {
        continue;
      }

      const auto projection = ProjectPointToSegment2D(ego.position, start, end);
      const auto segment_heading = std::atan2(end.y - start.y, end.x - start.x);
      const auto yaw_error = AngleDifference(segment_heading, ego.yaw);
      if (!nearest_valid || projection.distance < nearest_projection.distance) {
        nearest_valid = true;
        nearest_segment = i;
        nearest_projection = projection;
        nearest_yaw_error = yaw_error;
      }

      if (projection.distance > max_snap_distance || yaw_error > max_heading_error) {
        continue;
      }

      const auto dx = projection.point.x - ego.position.x;
      const auto dy = projection.point.y - ego.position.y;
      const auto longitudinal = dx * ego_heading_x + dy * ego_heading_y;
      if (longitudinal < -std::max(0.25, min_lane_point_spacing_)) {
        continue;
      }
      const auto behind_penalty = longitudinal < 0.0 ? std::fabs(longitudinal) * 2.0 : 0.0;
      const auto score = projection.distance + yaw_error * 0.5 + behind_penalty;
      if (!found || score < best_score) {
        found = true;
        best_segment = i;
        best_projection = projection;
        best_yaw_error = yaw_error;
        best_longitudinal = longitudinal;
        best_score = score;
      }
    }

    if (!found) {
      if (nearest_valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "reference path is not aligned with ego heading near the vehicle: "
          "nearest_segment=%zu distance=%.2fm yaw_error=%.2frad "
          "limits distance<=%.2fm yaw_error<=%.2frad; keeping original path",
          nearest_segment, nearest_projection.distance, nearest_yaw_error,
          max_snap_distance, max_heading_error);
      }
      return points;
    }

    std::vector<geometry_msgs::msg::Point> aligned_points;
    aligned_points.reserve(points.size() - best_segment + 3);
    aligned_points.push_back(ego.position);

    const auto projection_from_ego = Distance2D(ego.position, best_projection.point);
    if (projection_from_ego >= std::max(0.05, min_lane_point_spacing_ * 0.5)) {
      aligned_points.push_back(best_projection.point);
    }
    for (std::size_t i = best_segment + 1; i < points.size(); ++i) {
      if (!aligned_points.empty() &&
        Distance2D(aligned_points.back(), points[i]) < min_lane_point_spacing_)
      {
        continue;
      }
      aligned_points.push_back(points[i]);
    }

    if (aligned_points.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "reference path alignment produced a degenerate lane; keeping original path");
      return points;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "aligned reference path to ego heading: segment=%zu distance=%.2fm yaw_error=%.2frad "
      "longitudinal=%.2fm trimmed_prefix=%zu kept_points=%zu",
      best_segment, best_projection.distance, best_yaw_error, best_longitudinal,
      best_segment, aligned_points.size());
    return aligned_points;
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
    int filtered_actors = 0;
    for (const auto & [actor_name, actor_pose] : latest_actor_poses_) {
      if (actor_name.empty()) {
        continue;
      }
      auto actor_vehicle = MakeActorVehicle(
        actor_name, actor_pose, odom.header, dynamic_msg.header.frame_id);
      if (!ShouldPublishActor(actor_vehicle.state.vec_position)) {
        ++filtered_actors;
        continue;
      }
      dynamic_msg.vehicle_set.vehicles.push_back(std::move(actor_vehicle));
    }
    if (filtered_actors > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "filtered %d/%zu dynamic actors farther than %.2fm from the current LaneNet",
        filtered_actors, latest_actor_poses_.size(), actor_lane_max_distance_);
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
        if (!path_stale_warned_) {
          RCLCPP_WARN(
            get_logger(),
            "reference path on %s is stale for %.1fs; keeping the last lane until a new path arrives",
            path_topic_.c_str(), age);
          path_stale_warned_ = true;
        }
      } else {
        path_stale_warned_ = false;
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

    const auto lane_count = std::max(2, virtual_lane_count_);
    const auto lane_spacing = lane_count > 1 ?
      lane_width_ / static_cast<double>(lane_count - 1) : 0.0;
    const auto half_span = lane_width_ * 0.5;
    const bool allow_adjacent_lane_change = lane_count > 2 ? true : allow_opposite_lane_change_;

    for (int lane_index = 0; lane_index < lane_count; ++lane_index) {
      const auto offset = -half_span + lane_spacing * static_cast<double>(lane_index);
      std::vector<geometry_msgs::msg::Point> lane_points;
      lane_points.reserve(path_points.size());
      for (std::size_t i = 0; i < path_points.size(); ++i) {
        lane_points.push_back(OffsetPoint(path_points, i, offset));
      }

      auto lane = MakeLane(header, lane_index, lane_points);
      if (lane_index > 0) {
        lane.r_lane_id = lane_index - 1;
        lane.r_change_avbl = allow_adjacent_lane_change;
      }
      if (lane_index + 1 < lane_count) {
        lane.l_lane_id = lane_index + 1;
        lane.l_change_avbl = allow_adjacent_lane_change;
      }
      lane_net.lanes.push_back(std::move(lane));
    }
    return lane_net;
  }

  vehicle_msgs::msg::ObstacleSet BuildObstacleSet(const std_msgs::msg::Header & header) const
  {
    vehicle_msgs::msg::ObstacleSet obstacle_set;
    obstacle_set.header = header;
    int obstacle_id = 0;

    if (latest_costmap_.has_value()) {
      if (!HasFrameMismatch(latest_costmap_->header.frame_id)) {
        AppendCostmapObstacles(*latest_costmap_, &obstacle_id, &obstacle_set);
      }
    }
    if (latest_object_poses_.has_value()) {
      if (!HasFrameMismatch(latest_object_poses_->header.frame_id)) {
        AppendPoseObstacles(*latest_object_poses_, &obstacle_id, &obstacle_set);
      }
    }
    return obstacle_set;
  }

  bool HasFrameMismatch(const std::string & frame_id) const
  {
    return !frame_id.empty() && !map_frame_.empty() && frame_id != map_frame_;
  }

  bool ShouldPublishActor(const geometry_msgs::msg::Point & actor_position) const
  {
    if (!filter_dynamic_actors_by_lane_ || !latest_path_points_.has_value() ||
      latest_path_points_->size() < 2)
    {
      return true;
    }
    return NearestVirtualLaneDistance(actor_position) <= actor_lane_max_distance_;
  }

  double NearestVirtualLaneDistance(const geometry_msgs::msg::Point & point) const
  {
    if (!latest_path_points_.has_value() || latest_path_points_->size() < 2) {
      return 0.0;
    }

    const auto & path_points = *latest_path_points_;
    const auto lane_count = centerline_is_road_center_ && virtual_lane_count_ > 1 ?
      std::max(2, virtual_lane_count_) : 1;
    const auto lane_spacing = lane_count > 1 ?
      lane_width_ / static_cast<double>(lane_count - 1) : 0.0;
    const auto half_span = lane_count > 1 ? lane_width_ * 0.5 : 0.0;

    auto best_distance = std::numeric_limits<double>::infinity();
    for (int lane_index = 0; lane_index < lane_count; ++lane_index) {
      const auto offset = lane_count > 1 ?
        -half_span + lane_spacing * static_cast<double>(lane_index) : 0.0;
      for (std::size_t i = 1; i < path_points.size(); ++i) {
        const auto start = OffsetPoint(path_points, i - 1, offset);
        const auto end = OffsetPoint(path_points, i, offset);
        best_distance = std::min(best_distance, PointToSegmentDistance2D(point, start, end));
      }
    }
    return best_distance;
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
  double path_timeout_{120.0};
  int ego_id_{0};
  double min_lane_point_spacing_{0.3};
  bool centerline_is_road_center_{true};
  int virtual_lane_count_{3};
  double lane_width_{3.2};
  bool allow_opposite_lane_change_{false};
  int occupied_threshold_{65};
  bool treat_unknown_as_obstacle_{false};
  int max_obstacle_cells_{2500};
  double grid_obstacle_radius_{0.12};
  double object_obstacle_radius_{0.35};
  bool filter_dynamic_actors_by_lane_{true};
  double actor_lane_max_distance_{4.0};
  bool align_path_to_ego_{true};
  double path_ego_max_heading_error_{1.35};
  double path_ego_max_snap_distance_{8.0};
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
  bool path_stale_warned_{false};
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
