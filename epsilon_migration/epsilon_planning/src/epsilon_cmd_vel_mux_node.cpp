#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class EpsilonCmdVelMuxNode : public rclcpp::Node
{
public:
  EpsilonCmdVelMuxNode()
  : Node("epsilon_cmd_vel_mux_node")
  {
    declare_parameter<std::string>("epsilon_cmd_vel_topic", "/epsilon/cmd_vel_raw");
    declare_parameter<std::string>("nav2_cmd_vel_topic", "/control/nav2_cmd_vel_raw");
    declare_parameter<std::string>("output_cmd_vel_topic", "/control/cmd_vel_raw");
    declare_parameter<std::string>("control_source", "nav2");
    declare_parameter<std::string>("control_mode_topic", "/epsilon/control_mode");
    declare_parameter<std::string>("selected_source_topic", "/epsilon/selected_control_source");
    declare_parameter<std::string>("epsilon_status_topic", "/epsilon/status");
    declare_parameter<double>("input_timeout", 0.4);
    declare_parameter<double>("epsilon_status_timeout", 1.5);
    declare_parameter<double>("publish_period", 0.05);
    declare_parameter<double>("selection_hold_time", 1.0);
    declare_parameter<bool>("fallback_to_nav2", true);

    epsilon_cmd_vel_topic_ = get_parameter("epsilon_cmd_vel_topic").as_string();
    nav2_cmd_vel_topic_ = get_parameter("nav2_cmd_vel_topic").as_string();
    output_cmd_vel_topic_ = get_parameter("output_cmd_vel_topic").as_string();
    control_source_ = get_parameter("control_source").as_string();
    control_mode_topic_ = get_parameter("control_mode_topic").as_string();
    selected_source_topic_ = get_parameter("selected_source_topic").as_string();
    epsilon_status_topic_ = get_parameter("epsilon_status_topic").as_string();
    input_timeout_ = get_parameter("input_timeout").as_double();
    epsilon_status_timeout_ = get_parameter("epsilon_status_timeout").as_double();
    publish_period_ = get_parameter("publish_period").as_double();
    selection_hold_time_ = get_parameter("selection_hold_time").as_double();
    fallback_to_nav2_ = get_parameter("fallback_to_nav2").as_bool();

    if (!IsValidSource(control_source_)) {
      RCLCPP_WARN(
        get_logger(), "invalid initial control_source=%s; using stop",
        control_source_.c_str());
      control_source_ = "stop";
    }

    const auto qos = rclcpp::QoS(10).best_effort();
    epsilon_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      epsilon_cmd_vel_topic_, qos,
      std::bind(&EpsilonCmdVelMuxNode::EpsilonCallback, this, std::placeholders::_1));
    nav2_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav2_cmd_vel_topic_, qos,
      std::bind(&EpsilonCmdVelMuxNode::Nav2Callback, this, std::placeholders::_1));
    if (!control_mode_topic_.empty()) {
      mode_sub_ = create_subscription<std_msgs::msg::String>(
        control_mode_topic_, 10,
        std::bind(&EpsilonCmdVelMuxNode::ModeCallback, this, std::placeholders::_1));
    }
    if (!epsilon_status_topic_.empty()) {
      status_sub_ = create_subscription<std_msgs::msg::String>(
        epsilon_status_topic_, 10,
        std::bind(&EpsilonCmdVelMuxNode::StatusCallback, this, std::placeholders::_1));
    }
    if (!selected_source_topic_.empty()) {
      selected_source_pub_ = create_publisher<std_msgs::msg::String>(selected_source_topic_, 10);
    }

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.02, publish_period_)),
      std::bind(&EpsilonCmdVelMuxNode::PublishSelectedCommand, this));

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel mux ready: source=%s epsilon=%s nav2=%s output=%s timeout=%.2fs status=%s selected_topic=%s fallback_to_nav2=%s",
      control_source_.c_str(), epsilon_cmd_vel_topic_.c_str(), nav2_cmd_vel_topic_.c_str(),
      output_cmd_vel_topic_.c_str(), input_timeout_, epsilon_status_topic_.c_str(),
      selected_source_topic_.c_str(),
      fallback_to_nav2_ ? "true" : "false");
  }

private:
  static bool IsValidSource(const std::string & source)
  {
    return source == "auto" || source == "nav2" || source == "epsilon" || source == "stop";
  }

  static bool IsFiniteTwist(const geometry_msgs::msg::Twist & msg)
  {
    return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
           std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
           std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
  }

  void EpsilonCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_epsilon_ = *msg;
    latest_epsilon_time_ = std::chrono::steady_clock::now();
  }

  void Nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_nav2_ = *msg;
    latest_nav2_time_ = std::chrono::steady_clock::now();
  }

  void StatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    epsilon_ready_ = msg->data == "ok" || msg->data.rfind("ok ", 0) == 0;
    latest_epsilon_status_time_ = std::chrono::steady_clock::now();
  }

  void ModeCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!IsValidSource(msg->data)) {
      RCLCPP_WARN(
        get_logger(), "ignoring invalid control source '%s'; use auto, nav2, epsilon, or stop",
        msg->data.c_str());
      return;
    }
    if (control_source_ != msg->data) {
      RCLCPP_INFO(
        get_logger(), "control source changed: %s -> %s",
        control_source_.c_str(), msg->data.c_str());
      control_source_ = msg->data;
      warned_stale_ = false;
    }
  }

  void PublishSelectedCommand()
  {
    geometry_msgs::msg::Twist selected;
    const auto now = std::chrono::steady_clock::now();
    const auto is_fresh = [this, now](bool has_sample,
        const std::chrono::steady_clock::time_point & sample_time) {
        if (!has_sample) {
          return false;
        }
        const auto age = std::chrono::duration<double>(now - sample_time).count();
        return input_timeout_ <= 0.0 || age <= input_timeout_;
      };
    const bool epsilon_fresh = is_fresh(latest_epsilon_.has_value(), latest_epsilon_time_);
    const bool nav2_fresh = is_fresh(latest_nav2_.has_value(), latest_nav2_time_);
    const bool epsilon_status_fresh =
      epsilon_ready_ && latest_epsilon_status_time_.has_value() &&
      (epsilon_status_timeout_ <= 0.0 ||
      std::chrono::duration<double>(now - *latest_epsilon_status_time_).count() <=
      epsilon_status_timeout_);

    const auto twist_for_source = [this, epsilon_fresh, nav2_fresh](
        const std::string & source) -> geometry_msgs::msg::Twist {
        if (source == "epsilon" && epsilon_fresh) {
          return *latest_epsilon_;
        }
        if (source == "nav2" && nav2_fresh) {
          return *latest_nav2_;
        }
        return geometry_msgs::msg::Twist();
      };
    const auto source_available = [epsilon_fresh, nav2_fresh, epsilon_status_fresh](
        const std::string & source) {
        if (source == "epsilon") {
          return epsilon_fresh && epsilon_status_fresh;
        }
        if (source == "nav2") {
          return nav2_fresh;
        }
        return source == "stop";
      };

    std::string selected_source = "stop";
    if (control_source_ == "auto") {
      if (epsilon_status_fresh && epsilon_fresh) {
        selected_source = "epsilon";
      } else if (nav2_fresh) {
        selected_source = "nav2";
      }
    } else if (control_source_ == "epsilon") {
      if (epsilon_fresh) {
        selected_source = "epsilon";
      } else if (fallback_to_nav2_ && nav2_fresh) {
        selected_source = "nav2";
      }
    } else if (control_source_ == "nav2" && nav2_fresh) {
      selected_source = "nav2";
    }

    selected = twist_for_source(selected_source);

    if (
      control_source_ == "auto" && selected_source != "stop" &&
      !last_selected_source_.empty() && last_selected_source_ != selected_source &&
      last_selected_source_ != "stop" && source_available(last_selected_source_))
    {
      if (last_source_switch_time_.has_value()) {
        const auto since_switch = std::chrono::duration<double>(
          now - *last_source_switch_time_).count();
        if (since_switch < std::max(0.0, selection_hold_time_)) {
          selected_source = last_selected_source_;
          selected = twist_for_source(selected_source);
        }
      }
    }

    if (selected_source != last_selected_source_) {
      RCLCPP_INFO(
        get_logger(), "selected command source: %s -> %s",
        last_selected_source_.c_str(), selected_source.c_str());
      last_selected_source_ = selected_source;
      last_source_switch_time_ = now;
    }

    if (selected_source != "stop" && !IsFiniteTwist(selected)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "selected %s cmd_vel contains non-finite values; publishing zero",
        selected_source.c_str());
      selected_source = "stop";
      selected = geometry_msgs::msg::Twist();
    }

    if (selected_source == "stop" && control_source_ != "stop" && !warned_stale_) {
      RCLCPP_WARN(
        get_logger(), "no fresh safe command is available for requested source %s; publishing zero",
        control_source_.c_str());
      warned_stale_ = true;
    } else if (selected_source != "stop") {
      warned_stale_ = false;
    }

    PublishSelectedSource(selected_source, epsilon_fresh, nav2_fresh, epsilon_status_fresh);
    output_pub_->publish(selected);
  }

  void PublishSelectedSource(
    const std::string & selected_source, bool epsilon_fresh,
    bool nav2_fresh, bool epsilon_status_fresh)
  {
    if (!selected_source_pub_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = "selected=" + selected_source +
      " requested=" + control_source_ +
      " epsilon_fresh=" + std::string(epsilon_fresh ? "true" : "false") +
      " nav2_fresh=" + std::string(nav2_fresh ? "true" : "false") +
      " epsilon_status_fresh=" + std::string(epsilon_status_fresh ? "true" : "false") +
      " fallback_to_nav2=" + std::string(fallback_to_nav2_ ? "true" : "false");
    selected_source_pub_->publish(msg);
  }

  std::string epsilon_cmd_vel_topic_;
  std::string nav2_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string control_source_;
  std::string control_mode_topic_;
  std::string selected_source_topic_;
  std::string epsilon_status_topic_;
  double input_timeout_{0.4};
  double epsilon_status_timeout_{1.5};
  double publish_period_{0.05};
  double selection_hold_time_{1.0};
  bool fallback_to_nav2_{true};

  std::optional<geometry_msgs::msg::Twist> latest_epsilon_;
  std::optional<geometry_msgs::msg::Twist> latest_nav2_;
  std::chrono::steady_clock::time_point latest_epsilon_time_;
  std::chrono::steady_clock::time_point latest_nav2_time_;
  std::optional<std::chrono::steady_clock::time_point> latest_epsilon_status_time_;
  std::optional<std::chrono::steady_clock::time_point> last_source_switch_time_;
  bool epsilon_ready_{false};
  bool warned_stale_{false};
  std::string last_selected_source_{""};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr epsilon_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_source_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EpsilonCmdVelMuxNode>());
  rclcpp::shutdown();
  return 0;
}
