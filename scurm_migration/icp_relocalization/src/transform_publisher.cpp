#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <chrono>

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
        : Node("transform_publisher_node")
    {
        this->declare_parameter<std::string>("odom_frame_id","odom");
        this->declare_parameter<std::string>("map_frame_id","map");
        this->declare_parameter<bool>("publish_identity_until_icp", true);
        this->declare_parameter<double>("publish_rate_hz", 10.0);

        bool use_sim_time = true;
        if (this->has_parameter("use_sim_time"))
        {
            this->get_parameter("use_sim_time", use_sim_time);
        }
        else
        {
            this->declare_parameter<bool>("use_sim_time", true);
            this->get_parameter("use_sim_time", use_sim_time);
        }
        if (!use_sim_time)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
            use_sim_time = true;
        }
        this->get_parameter_or<std::string>("odom_frame_id", odom_frame_id, "odom");
        this->get_parameter_or<std::string>("map_frame_id", map_frame_id, "map");
        this->get_parameter_or<bool>("publish_identity_until_icp", publish_identity_until_icp, true);
        this->get_parameter_or<double>("publish_rate_hz", publish_rate_hz, 10.0);
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", 10, std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));
        relocalization_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "relocalization_result", 10, std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        current_transform_.header.frame_id = map_frame_id;
        current_transform_.child_frame_id = odom_frame_id;
        current_transform_.transform.rotation.w = 1.0;
        current_transform_.transform.translation.x = 0.0;
        current_transform_.transform.translation.y = 0.0;
        current_transform_.transform.translation.z = 0.0;

        const auto publish_period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
            std::bind(&TransformPublisherNode::publish_current_transform, this));

        if (publish_identity_until_icp)
        {
            publish_current_transform();
        }

        if (publish_identity_until_icp)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Publishing identity map -> odom until ICP result arrives at %.2f Hz (use_sim_time=%s)",
                publish_rate_hz,
                use_sim_time ? "true" : "false");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Waiting for ICP result before publishing map -> odom at %.2f Hz (use_sim_time=%s)",
                publish_rate_hz,
                use_sim_time ? "true" : "false");
        }
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_transform_.header.frame_id = map_frame_id;
        current_transform_.child_frame_id = odom_frame_id;
        current_transform_.transform.translation.x = msg->pose.pose.position.x;
        current_transform_.transform.translation.y = msg->pose.pose.position.y;
        current_transform_.transform.translation.z = msg->pose.pose.position.z;
        current_transform_.transform.rotation = msg->pose.pose.orientation;
        have_icp_result_ = true;
        publish_current_transform();
    }

    void publish_current_transform()
    {
        if (!have_icp_result_ && !publish_identity_until_icp)
        {
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = current_transform_.header.frame_id;
        transform.child_frame_id = current_transform_.child_frame_id;
        transform.transform = current_transform_.transform;

        broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr relocalization_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped current_transform_;
    std::string odom_frame_id, map_frame_id;
    bool have_icp_result_ = false;
    bool publish_identity_until_icp = true;
    double publish_rate_hz = 10.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
