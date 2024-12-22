#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace small_gicp_relocalization
{

  SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions &options)
      : Node("small_gicp_relocalization", options),
        result_t_(Eigen::Isometry3d::Identity()),
        previous_result_t_(Eigen::Isometry3d::Identity())
  {
    this->declare_parameter("num_threads", 4);
    this->declare_parameter("num_neighbors", 20);
    this->declare_parameter("global_leaf_size", 0.25);
    this->declare_parameter("registered_leaf_size", 0.25);
    this->declare_parameter("max_dist_sq", 1.0);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "");
    this->declare_parameter("lidar_frame", "");
    this->declare_parameter("prior_pcd_file", "");

    this->get_parameter("num_threads", num_threads_);
    this->get_parameter("num_neighbors", num_neighbors_);
    this->get_parameter("global_leaf_size", global_leaf_size_);
    this->get_parameter("registered_leaf_size", registered_leaf_size_);
    this->get_parameter("max_dist_sq", max_dist_sq_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("prior_pcd_file", prior_pcd_file_);

    registered_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_ = std::make_shared<
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    loadGlobalMap(prior_pcd_file_);

    // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
    target_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *global_map_, global_leaf_size_);

    // Estimate covariances of points
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

    // Create KdTree for target
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));

    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 10,
        std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

    register_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // 5 Hz
        std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

    transform_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), // 20 Hz
        std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
  }

  void SmallGicpRelocalizationNode::loadGlobalMap(const std::string &file_name)
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

    // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
    Eigen::Affine3d odom_to_lidar_odom;
    while (true)
    {
      try
      {
        auto tf_stamped = tf_buffer_->lookupTransform(
            base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
        odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
        RCLCPP_INFO_STREAM(
            this->get_logger(), "odom_to_lidar_odom: translation = "
                                    << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                                    << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
        break;
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Retrying...", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
    pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
  }

  void SmallGicpRelocalizationNode::registeredPcdCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_scan_time_ = msg->header.stamp;

    pcl::fromROSMsg(*msg, *registered_scan_);

    // Downsample Registered points and convert them into pcl::PointCloud<pcl::PointCovariance>.
    source_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *registered_scan_, registered_leaf_size_);

    // Estimate point covariances
    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    // Create KdTree for source.
    source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        source_, small_gicp::KdTreeBuilderOMP(num_threads_));
  }

  void SmallGicpRelocalizationNode::performRegistration()
  {
    if (!source_ || !source_tree_)
    {
      return;
    }

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = max_dist_sq_;

    // Align point clouds using the previous result as the initial transformation
    auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

    if (!result.converged)
    {
      RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
      return;
    }

    result_t_ = result.T_target_source;
    previous_result_t_ = result.T_target_source;
  }

  void SmallGicpRelocalizationNode::publishTransform()
  {
    if (result_t_.matrix().isZero())
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
    transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id = odom_frame_;

    const Eigen::Vector3d translation = result_t_.translation();
    const Eigen::Quaterniond rotation(result_t_.rotation());

    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);
  }

} // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)