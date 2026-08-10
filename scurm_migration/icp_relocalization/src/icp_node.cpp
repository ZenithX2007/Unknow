#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

class ICPNode : public rclcpp::Node
{
public:
    ICPNode()
        : Node("icp_node")
    {
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_a", 0.0);
        this->declare_parameter("solver_max_iter", 75);
        this->declare_parameter("max_correspondence_distance", 0.1);
        this->declare_parameter("RANSAC_outlier_rejection_threshold", 1.0);
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("fitness_score_thre", 0.0);
        this->declare_parameter("map_voxel_leaf_size", 0.1);
        this->declare_parameter("cloud_voxel_leaf_size", 0.1);
        this->declare_parameter("map_max_abs_coord", 1000.0);
        this->declare_parameter("map_min_z", -1000.0);
        this->declare_parameter("map_max_z", 1000.0);
        this->declare_parameter("max_target_map_points", 1000000);
        this->declare_parameter("publish_prior_map", true);
        this->declare_parameter("prior_map_publish_voxel_leaf_size", 1.0);
        this->declare_parameter("max_published_prior_map_points", 250000);
        this->declare_parameter("max_published_transformed_cloud_points", 50000);
        this->declare_parameter("converged_count_thre", 20);
        this->declare_parameter("pcl_type","livox");
        this->declare_parameter("input_cloud_to_base_x", 0.0);
        this->declare_parameter("input_cloud_to_base_y", 0.0);
        this->declare_parameter("input_cloud_to_base_z", 0.0);
        this->declare_parameter("input_cloud_to_base_roll", 0.0);
        this->declare_parameter("input_cloud_to_base_pitch", 0.0);
        this->declare_parameter("input_cloud_to_base_yaw", 0.0);
        this->declare_parameter("legacy_livox_roll_180", true);
        this->declare_parameter("update_initial_guess_on_high_error", false);

        this->get_parameter("initial_x", initial_x);
        this->get_parameter("initial_y", initial_y);
        this->get_parameter("initial_z", initial_z);
        this->get_parameter("initial_a", initial_a);
        this->get_parameter("solver_max_iter", solver_max_iter);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance);
        this->get_parameter("RANSAC_outlier_rejection_threshold", RANSAC_outlier_rejection_threshold);
        this->get_parameter("map_path", map_path);
        this->get_parameter("map_frame_id", map_frame);
        this->get_parameter("fitness_score_thre", fitness_score_thre);
        this->get_parameter("map_voxel_leaf_size", map_voxel_leaf_size);
        this->get_parameter("cloud_voxel_leaf_size", cloud_voxel_leaf_size);
        this->get_parameter("map_max_abs_coord", map_max_abs_coord);
        this->get_parameter("map_min_z", map_min_z);
        this->get_parameter("map_max_z", map_max_z);
        this->get_parameter("max_target_map_points", max_target_map_points);
        this->get_parameter("publish_prior_map", publish_prior_map);
        this->get_parameter("prior_map_publish_voxel_leaf_size", prior_map_publish_voxel_leaf_size);
        this->get_parameter("max_published_prior_map_points", max_published_prior_map_points);
        this->get_parameter("max_published_transformed_cloud_points", max_published_transformed_cloud_points);
        this->get_parameter("converged_count_thre", converged_count_thre);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("input_cloud_to_base_x", input_cloud_to_base_x);
        this->get_parameter("input_cloud_to_base_y", input_cloud_to_base_y);
        this->get_parameter("input_cloud_to_base_z", input_cloud_to_base_z);
        this->get_parameter("input_cloud_to_base_roll", input_cloud_to_base_roll);
        this->get_parameter("input_cloud_to_base_pitch", input_cloud_to_base_pitch);
        this->get_parameter("input_cloud_to_base_yaw", input_cloud_to_base_yaw);
        this->get_parameter("legacy_livox_roll_180", legacy_livox_roll_180);
        this->get_parameter("update_initial_guess_on_high_error", update_initial_guess_on_high_error);

        auto relocalization_pose_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", relocalization_pose_qos);
        relocalization_result_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/relocalization_result", relocalization_pose_qos);
#ifdef USE_LIVOX
        if(pcl_type == "livox")
        {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10, std::bind(&ICPNode::lvx_cloud_callback, this, std::placeholders::_1));
        }
        else
        {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
#endif
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&ICPNode::pose_callback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", 10);
        template_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/template_cloud", 10);
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);
        icp_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/icp_cloud", 10);

        // init guess
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = initial_x;
        initGuess(1, 3) = initial_y;
        initGuess(2, 3) = initial_z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_a);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", initial_x, initial_y, initial_z, initial_a);
        configure_input_cloud_transform();
        // Load the target point cloud from a PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *target_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read prior map PCD: %s", map_path.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %d data points from target.pcd", target_cloud_->width * target_cloud_->height);

        filter_invalid_and_bounds(
            target_cloud_,
            map_max_abs_coord,
            map_min_z,
            map_max_z,
            "target map");
        voxel_downsample(target_cloud_, map_voxel_leaf_size, "target map");
        limit_point_count(target_cloud_, max_target_map_points, "target map for ICP", true);
        RCLCPP_INFO(this->get_logger(), "Prepared target map with %zu data points", target_cloud_->size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr publish_cloud(new pcl::PointCloud<pcl::PointXYZ>(*target_cloud_));
        voxel_downsample(publish_cloud, prior_map_publish_voxel_leaf_size, "published prior map");
        limit_point_count(publish_cloud, max_published_prior_map_points, "published prior map", true);
        pcl::toROSMsg(*publish_cloud, target_cloud_msg);
        target_cloud_msg.header.stamp = this->now();
        target_cloud_msg.header.frame_id = map_frame;
        if (publish_prior_map)
        {
            map_pub_->publish(target_cloud_msg);
            template_cloud_pub_->publish(target_cloud_msg);
        }
    }

private:
    void configure_input_cloud_transform()
    {
        input_cloud_to_base_transform = Eigen::Matrix4f::Identity();
        tf2::Quaternion q;
        q.setRPY(input_cloud_to_base_roll, input_cloud_to_base_pitch, input_cloud_to_base_yaw);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                input_cloud_to_base_transform(i, j) = rot_mat[i][j];
            }
        }
        input_cloud_to_base_transform(0, 3) = input_cloud_to_base_x;
        input_cloud_to_base_transform(1, 3) = input_cloud_to_base_y;
        input_cloud_to_base_transform(2, 3) = input_cloud_to_base_z;

        if (legacy_livox_roll_180)
        {
            Eigen::Matrix4f legacy_rotation = Eigen::Matrix4f::Identity();
            legacy_rotation(1, 1) = -1;
            legacy_rotation(2, 2) = -1;
            input_cloud_to_base_transform = input_cloud_to_base_transform * legacy_rotation;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Input cloud to base: xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f), legacy_livox_roll_180=%s, update_initial_guess_on_high_error=%s",
            input_cloud_to_base_x,
            input_cloud_to_base_y,
            input_cloud_to_base_z,
            input_cloud_to_base_roll,
            input_cloud_to_base_pitch,
            input_cloud_to_base_yaw,
            legacy_livox_roll_180 ? "true" : "false",
            update_initial_guess_on_high_error ? "true" : "false");
    }

    void transform_input_cloud_to_base(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
    {
        pcl::transformPointCloud(*input_cloud, *input_cloud, input_cloud_to_base_transform);
    }

    void publish_target_map()
    {
        if (!publish_prior_map || target_cloud_msg.data.empty())
        {
            return;
        }
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }

    void filter_invalid_and_bounds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        double max_abs_coord,
        double min_z,
        double max_z,
        const std::string &label)
    {
        if (cloud->empty())
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        filtered->reserve(cloud->size());
        for (const auto &point : cloud->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            if (max_abs_coord > 0.0 &&
                (std::abs(point.x) > max_abs_coord ||
                 std::abs(point.y) > max_abs_coord ||
                 std::abs(point.z) > max_abs_coord))
            {
                continue;
            }
            if (point.z < min_z || point.z > max_z)
            {
                continue;
            }
            filtered->push_back(point);
        }
        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = true;

        const size_t removed = cloud->size() - filtered->size();
        cloud->swap(*filtered);
        if (removed > 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Filtered %zu invalid/out-of-bounds points from %s",
                removed,
                label.c_str());
        }
    }

    void voxel_downsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        double leaf_size,
        const std::string &label)
    {
        if (leaf_size <= 0.0 || cloud->empty())
        {
            return;
        }

        const size_t before = cloud->size();
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        try
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(cloud);
            voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel_grid.filter(*filtered);
        }
        catch (const std::exception &error)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Voxel downsample failed for %s with leaf %.3f: %s",
                label.c_str(),
                leaf_size,
                error.what());
            return;
        }

        if (filtered->empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Voxel downsample produced an empty %s cloud; keeping the original cloud",
                label.c_str());
            return;
        }

        cloud->swap(*filtered);
        RCLCPP_INFO(
            this->get_logger(),
            "Downsampled %s from %zu to %zu points with leaf %.3f",
            label.c_str(),
            before,
            cloud->size(),
            leaf_size);
    }

    void limit_point_count(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        int max_points,
        const std::string &label,
        bool log)
    {
        if (max_points <= 0 || cloud->size() <= static_cast<size_t>(max_points))
        {
            return;
        }

        const size_t before = cloud->size();
        const size_t limit = static_cast<size_t>(max_points);
        const size_t stride = (before + limit - 1) / limit;
        pcl::PointCloud<pcl::PointXYZ>::Ptr limited(new pcl::PointCloud<pcl::PointXYZ>);
        limited->reserve(limit);
        for (size_t i = 0; i < before && limited->size() < limit; i += stride)
        {
            limited->push_back(cloud->points[i]);
        }
        limited->width = limited->size();
        limited->height = 1;
        limited->is_dense = cloud->is_dense;
        cloud->swap(*limited);

        if (log)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Limited %s from %zu to %zu points",
                label.c_str(),
                before,
                cloud->size());
        }
    }

    void publish_transformed_cloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
        const Eigen::Matrix4f &transformation)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, transformation);
        limit_point_count(
            transformed_cloud,
            max_published_transformed_cloud_points,
            "published transformed cloud",
            false);

        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
        transformed_cloud_msg.header.stamp = this->now();
        transformed_cloud_msg.header.frame_id = map_frame;
        transformed_cloud_pub_->publish(transformed_cloud_msg);
        icp_cloud_pub_->publish(transformed_cloud_msg);
    }

    void publish_pose(const Eigen::Matrix4f &transformation_result)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame;
        pose_msg.pose.pose.position.x = transformation_result(0, 3);
        pose_msg.pose.pose.position.y = transformation_result(1, 3);
        pose_msg.pose.pose.position.z = transformation_result(2, 3);

        Eigen::Matrix3f rotation = transformation_result.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        q.normalize();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        publisher_->publish(pose_msg);
        relocalization_result_pub_->publish(pose_msg);
    }

    void run_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
    {
        if (relocalization_pose_published)
        {
            return;
        }
        if (target_cloud_->empty())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Prior map is empty; ICP pose cannot be computed");
            return;
        }

        transform_input_cloud_to_base(input_cloud);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);

        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, initGuess);

        double fitness_score = icp.getFitnessScore();
        bool icp_converged = icp.hasConverged();
        RCLCPP_INFO(this->get_logger(), "ICP fitness score: %f", fitness_score);

        if (icp_converged && fitness_score < fitness_score_thre)
        {
            converged_count++;
            Eigen::Matrix4f transformation_result = icp.getFinalTransformation();
            initGuess = transformation_result;

            int required_count = std::max(1, converged_count_thre);
            if (converged_count < required_count)
            {
                publish_transformed_cloud(input_cloud, transformation_result);
                publish_target_map();
                RCLCPP_INFO(
                    this->get_logger(),
                    "ICP converged with low error, count: %d/%d, no pose is published yet",
                    converged_count,
                    required_count);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "ICP converged, pose is published");
            publish_pose(transformation_result);
            publish_transformed_cloud(input_cloud, transformation_result);
            publish_target_map();
            relocalization_pose_published = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Relocalization pose latched on /icp_result; keeping node alive for late subscribers.");
            return;
        }

        converged_count = 0;
        if (icp_converged && update_initial_guess_on_high_error)
        {
            initGuess = icp.getFinalTransformation();
        }

        publish_transformed_cloud(input_cloud, initGuess);
        publish_target_map();
        if (icp_converged)
        {
            RCLCPP_INFO(this->get_logger(), "ICP converged with high error, no pose is published");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "ICP did not converge, no pose is published");
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        run_icp(input_cloud);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        const size_t point_count = std::min(
            static_cast<size_t>(msg->point_num), msg->points.size());
        for (size_t i = 0; i < point_count; i++)
        {
            pcl::PointXYZ point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            input_cloud->push_back(point);
        }
        input_cloud->width = input_cloud->size();
        input_cloud->height = 1;

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        run_icp(input_cloud);
    }
#endif

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Convert the incoming pose to an Eigen matrix
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = msg->pose.pose.position.x;
        initGuess(1, 3) = msg->pose.pose.position.y;
        initGuess(2, 3) = msg->pose.pose.position.z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        double r,p,yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr relocalization_result_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr template_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};

    Eigen::Matrix4f initGuess;
    Eigen::Matrix4f input_cloud_to_base_transform;
    double initial_x, initial_y, initial_z, initial_a;
    double input_cloud_to_base_x, input_cloud_to_base_y, input_cloud_to_base_z;
    double input_cloud_to_base_roll, input_cloud_to_base_pitch, input_cloud_to_base_yaw;
    int solver_max_iter;
    double max_correspondence_distance, RANSAC_outlier_rejection_threshold;
    std::string map_path, map_frame;
    double fitness_score_thre;
    double map_voxel_leaf_size, cloud_voxel_leaf_size, map_max_abs_coord;
    double map_min_z, map_max_z;
    double prior_map_publish_voxel_leaf_size;
    sensor_msgs::msg::PointCloud2 target_cloud_msg;
    int converged_count = 0;
    int converged_count_thre;
    int max_target_map_points;
    int max_published_prior_map_points;
    int max_published_transformed_cloud_points;
    bool legacy_livox_roll_180;
    bool update_initial_guess_on_high_error;
    bool publish_prior_map;
    bool relocalization_pose_published = false;
    std::string pcl_type;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
