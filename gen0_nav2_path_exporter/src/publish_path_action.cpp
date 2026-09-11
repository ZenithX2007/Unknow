#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gen0_nav2_path_exporter
{

class PublishPathAction : public BT::SyncActionNode
{
public:
  PublishPathAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput<std::string>("topic", topic_);
    if (topic_.empty()) {
      topic_ = "/plan_smoothed";
    }
    publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      topic_, rclcpp::QoS(1).transient_local().reliable());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "The smoothed path from Nav2's blackboard"),
      BT::InputPort<std::string>("topic", "/plan_smoothed", "ROS topic for the active path")
    };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto path = getInput<nav_msgs::msg::Path>("path");
    if (!path) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Nav2 smoothed path is not available on the behavior-tree blackboard");
      return BT::NodeStatus::FAILURE;
    }

    auto message = path.value();
    if (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0) {
      message.header.stamp = node_->now();
    }
    publisher_->publish(message);
    return BT::NodeStatus::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  std::string topic_{"/plan_smoothed"};
};

}  // namespace gen0_nav2_path_exporter

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gen0_nav2_path_exporter::PublishPathAction>("PublishPath");
}
