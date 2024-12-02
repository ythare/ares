#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <boost/make_shared.hpp>
#include <Eigen/Core>

using namespace std;

class FilterPointcloud : public rclcpp::Node
{
public:
    FilterPointcloud(const rclcpp::NodeOptions &options)
        : Node("FilterPointcloud", options)
    {
        RCLCPP_INFO(get_logger(), "Start FilterPointcloud!");
        // Parameters
        livox_points_filter_message_name_ = declare_parameter<std::string>("livox_points_filter_message_name", "/livox/lidar/pointcloud_filter");
        livox_points_message_name_ = declare_parameter<std::string>("livox_points_message_name", "/livox/lidar/pointcloud");
        odom_topic_name_ = declare_parameter<std::string>("odom_topic_name", "/Odometry");
        thre_low_ = declare_parameter<double>("thre_low", 0.3);
        thre_high_ = declare_parameter<double>("thre_high", 0.3);

        RCLCPP_INFO(this->get_logger(), "thre_low: %f , thre_high: %f", thre_low_, thre_high_);

        // Initialize point cloud container
        raw_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // Publishers
        pub_livox_points_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(livox_points_filter_message_name_, 10);

        // Subscribers
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_name_, 10, bind(&FilterPointcloud::odom_callback, this, std::placeholders::_1));
        sub_pointcloud2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            livox_points_message_name_, 10, bind(&FilterPointcloud::point2filter_callback, this, std::placeholders::_1));
    }

private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;
        pose_z_ = msg->pose.pose.position.z;
    }

    void point2filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PCLPointCloud2 pcl_pointcloud2;
        pcl_conversions::toPCL(*msg, pcl_pointcloud2);

        pcl::PCLPointCloud2 cloud_after_PassThrough;

        // Box filter for point cloud
        pcl::CropBox<pcl::PCLPointCloud2> box_filter;
        box_filter.setMin(Eigen::Vector4f(0 - thre_low_, 0 - thre_low_, 0 - thre_low_, 1.0));
        box_filter.setMax(Eigen::Vector4f(0 + thre_high_, 0 + thre_high_, 0 + thre_high_, 1.0));
        box_filter.setNegative(true);

        // Use std::shared_ptr instead of boost::shared_ptr
        std::shared_ptr<pcl::PCLPointCloud2> pcl_pointcloud2_ptr = std::make_shared<pcl::PCLPointCloud2>(pcl_pointcloud2);

        // Pass the pointer to setInputCloud
        box_filter.setInputCloud(pcl_pointcloud2_ptr); // Pass the pointer

        box_filter.filter(cloud_after_PassThrough);

        // Convert the filtered cloud back to ROS PointCloud2 format
        sensor_msgs::msg::PointCloud2 sensor_pointcloud2;
        pcl_conversions::moveFromPCL(cloud_after_PassThrough, sensor_pointcloud2);

        // Publish the filtered point cloud
        pub_livox_points_filtered_->publish(sensor_pointcloud2);
    }

    // Member variables
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_livox_points_filtered_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_raw_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud2_;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> raw_cloud_;

    std::string livox_points_message_name_;
    std::string livox_points_filter_message_name_;
    std::string odom_topic_name_;

    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    double pose_z_ = 0.0;

    double thre_low_;
    double thre_high_;
};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(FilterPointcloud)