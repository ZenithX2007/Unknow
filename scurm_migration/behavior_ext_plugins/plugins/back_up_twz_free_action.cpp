// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_ext_plugins/back_up_twz_free_action.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "tf2/utils.h"

namespace
{
double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

namespace nav2_behaviors
{
  void BackUpTwzFree::onConfigure()
  {
    DriveOnHeading<nav2_msgs::action::BackUp>::onConfigure();

    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    const std::string prefix = this->behavior_name_ + ".";

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "robot_radius", rclcpp::ParameterValue(0.7));
    node->get_parameter(prefix + "robot_radius", robot_radius_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "max_radius", rclcpp::ParameterValue(1.5));
    node->get_parameter(prefix + "max_radius", max_radius_);

    if(max_radius_ < robot_radius_)
    {
      RCLCPP_WARN(node->get_logger(), "max_radius is smaller than robot_radius. Setting max_radius to robot_radius");
      max_radius_ = robot_radius_;
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "service_name", rclcpp::ParameterValue(std::string("/local_costmap/get_costmap")));
    node->get_parameter(prefix + "service_name", service_name_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "free_threshold", rclcpp::ParameterValue(5));
    node->get_parameter(prefix + "free_threshold", free_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "cost_threshold", rclcpp::ParameterValue(5.0));
    node->get_parameter(prefix + "cost_threshold", cost_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "visualization", rclcpp::ParameterValue(false));
    node->get_parameter(prefix + "visualization", visualization_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "holonomic_motion", rclcpp::ParameterValue(false));
    node->get_parameter(prefix + "holonomic_motion", holonomic_motion_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "max_angular_speed", rclcpp::ParameterValue(0.25));
    node->get_parameter(prefix + "max_angular_speed", max_angular_speed_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "min_turning_radius", rclcpp::ParameterValue(6.62));
    node->get_parameter(prefix + "min_turning_radius", min_turning_radius_);
    if (min_turning_radius_ < 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "min_turning_radius is negative. Disabling curvature limiting.");
      min_turning_radius_ = 0.0;
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "ackermann_heading_tolerance", rclcpp::ParameterValue(0.35));
    node->get_parameter(prefix + "ackermann_heading_tolerance", ackermann_heading_tolerance_);
    if (ackermann_heading_tolerance_ < 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "ackermann_heading_tolerance is negative. Setting it to 0.");
      ackermann_heading_tolerance_ = 0.0;
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      prefix + "min_free_space_direction", rclcpp::ParameterValue(0.25));
    node->get_parameter(prefix + "min_free_space_direction", min_free_space_direction_);
    if (min_free_space_direction_ < 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "min_free_space_direction is negative. Setting it to 0.");
      min_free_space_direction_ = 0.0;
    }
    
    costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("back_up_twz_free_markers", 1);

    RCLCPP_DEBUG(node->get_logger(), "back_up_twz_free_action plugin initialized.");
  }

  Status BackUpTwzFree::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
  {
    // send request to get costmap
    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    while (!costmap_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return Status::FAILED;
      }
      RCLCPP_WARN(node->get_logger(), "service not available, waiting again...");
    }

    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto result = costmap_client_->async_send_request(request);
    if(result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return Status::FAILED;
    }

    RCLCPP_DEBUG(node->get_logger(), "Got costmap");

    // get costmap
    auto costmap = result.get()->map;

    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }

    // move towards free space
    // get current pose
    auto pose_x = initial_pose_.pose.position.x;
    auto pose_y = initial_pose_.pose.position.y;
    auto yaw = tf2::getYaw(initial_pose_.pose.orientation);

    auto free_space_found = false;

    // expand circle until free space is found
    auto radius = robot_radius_;
    std::vector<geometry_msgs::msg::Point> free_points;
    while (!free_space_found && radius <= max_radius_)
    {
      // calculate sum of free space in circle
      int free_space_sum = 0;
      std::vector<geometry_msgs::msg::Point> candidate_points;
      for (auto i = 0; i < costmap.metadata.size_x; i++)
      {
        for (auto j = 0; j < costmap.metadata.size_y; j++)
        {
          auto costmap_index = i + j * costmap.metadata.size_x;
          auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
          auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
          auto distance_to_center = std::hypot(x - pose_x, y - pose_y);
          if (distance_to_center <= radius)
          {
            if (costmap.data[costmap_index] <= cost_threshold_)
            {
              free_space_sum++;
              candidate_points.push_back(geometry_msgs::msg::Point());
              candidate_points.back().x = x;
              candidate_points.back().y = y;
            }
          }
        }
      }

      if (free_space_sum > free_threshold_)
      {
        free_space_found = true;
        free_points = std::move(candidate_points);
        RCLCPP_WARN(node->get_logger(), "free space found at radius: %f", radius);
        break;
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "free space not found at radius: %f", radius);
        radius += 0.1;
      }
    }

    if (!free_space_found || free_points.empty())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "No free space found in local costmap within %.2f m; aborting BackUpTwzFree",
        max_radius_);
      return Status::FAILED;
    }

    // calculate avg position of free space
    auto avg_x = 0.0;
    auto avg_y = 0.0;
    for (auto i = 0; i < free_points.size(); i++)
    {
      avg_x += free_points[i].x;
      avg_y += free_points[i].y;
    }
    avg_x /= free_points.size();
    avg_y /= free_points.size();
    RCLCPP_WARN(node->get_logger(), "avg_x: %f, avg_y: %f", avg_x, avg_y);

    const double free_space_direction = std::hypot(avg_x - pose_x, avg_y - pose_y);
    if (free_space_direction < min_free_space_direction_)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Free-space centroid is only %.3f m from the robot; recovery direction is undefined. "
        "Aborting Ackermann recovery.",
        free_space_direction);
      return Status::FAILED;
    }

    // visualize free space and destination
    if(visualization_){
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = node->now();
      marker.ns = "free_space";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = costmap.metadata.resolution;
      marker.scale.y = costmap.metadata.resolution;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      for (auto i = 0; i < free_points.size(); i++)
      {
        marker.points.push_back(free_points[i]);
      }
      markers.markers.push_back(marker);
      visualization_msgs::msg::Marker destination_marker;
      destination_marker.header.frame_id = global_frame_;
      destination_marker.header.stamp = node->now();
      destination_marker.ns = "destination";
      destination_marker.id = 0;
      destination_marker.type = visualization_msgs::msg::Marker::POINTS;
      destination_marker.action = visualization_msgs::msg::Marker::ADD;
      destination_marker.pose.orientation.w = 1.0;
      destination_marker.scale.x = costmap.metadata.resolution;
      destination_marker.scale.y = costmap.metadata.resolution;
      destination_marker.color.g = 1.0;
      destination_marker.color.a = 1.0;
      destination_marker.points.push_back(geometry_msgs::msg::Point());
      destination_marker.points.back().x = avg_x;
      destination_marker.points.back().y = avg_y;
      markers.markers.push_back(destination_marker);    
      marker_pub_->publish(markers);
    }
    
    // calculate angle to free space
    auto angle_to_free_space = std::atan2(avg_y - pose_y, avg_x - pose_x);
    auto angle_diff = angle_to_free_space - yaw;
    if (angle_diff > M_PI)
    {
      angle_diff -= 2 * M_PI;
    }
    else if (angle_diff < -M_PI)
    {
      angle_diff += 2 * M_PI;
    }
    RCLCPP_WARN(node->get_logger(), "angle_diff: %f deg", angle_diff*180/M_PI);

    // calculate move command
    if (holonomic_motion_)
    {
      twist_x_ = std::cos(angle_diff) * command->speed;
      twist_y_ = std::sin(angle_diff) * command->speed;
      twist_z_ = 0.0;
    }
    else
    {
      const double speed = std::abs(command->speed);
      const bool free_space_is_ahead = std::cos(angle_diff) >= 0.0;
      twist_x_ = free_space_is_ahead ? speed : -speed;
      twist_y_ = 0.0;
      const double movement_heading = free_space_is_ahead ?
        angle_diff : normalize_angle(angle_diff - std::copysign(M_PI, angle_diff));
      if (std::abs(movement_heading) > ackermann_heading_tolerance_)
      {
        RCLCPP_WARN(
          node->get_logger(),
          "free-space heading %.1f deg exceeds Ackermann recovery tolerance %.1f deg; "
          "aborting recovery instead of issuing a straight %s command",
          movement_heading * 180.0 / M_PI,
          ackermann_heading_tolerance_ * 180.0 / M_PI,
          twist_x_ >= 0.0 ? "forward" : "reverse");
        return Status::FAILED;
      }
      else
      {
        twist_z_ = clamp(movement_heading, -max_angular_speed_, max_angular_speed_);
      }
      if (min_turning_radius_ > 0.0)
      {
        const double feasible_angular_speed =
          std::min(max_angular_speed_, std::abs(twist_x_) / min_turning_radius_);
        const double requested_twist_z = twist_z_;
        twist_z_ = clamp(twist_z_, -feasible_angular_speed, feasible_angular_speed);
        if (std::abs(requested_twist_z) > feasible_angular_speed + 1e-6)
        {
          RCLCPP_WARN(
            node->get_logger(),
            "limiting recovery angular speed from %.3f to %.3f rad/s for "
            "min_turning_radius %.2f m at speed %.2f m/s",
            requested_twist_z, twist_z_, min_turning_radius_, twist_x_);
        }
      }
    }
    command_x_ = command->target.x;
    command_time_allowance_ = command->time_allowance;

    end_time_ = this->clock_->now() + command_time_allowance_;

    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }
    RCLCPP_WARN(
        this->logger_,
        "moving %f meters towards free space at angle %f with cmd_vel=(%.3f, %.3f, %.3f)",
        command_x_, angle_diff, twist_x_, twist_y_, twist_z_);

    return Status::SUCCEEDED;
  }

  Status BackUpTwzFree::onCycleUpdate()
  {
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0)
    {
      this->stopRobot();
      RCLCPP_WARN(
          this->logger_,
          "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
            current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
            this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }

    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);

    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

    if (distance >= std::fabs(command_x_))
    {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.y = twist_y_;
    cmd_vel->linear.x = twist_x_;
    cmd_vel->angular.z = twist_z_;

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    if (!isCollisionFree(distance, cmd_vel.get(), pose2d))
    {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    this->vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

} // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUpTwzFree, nav2_core::Behavior)
