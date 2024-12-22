#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <pcl/io/pcd_io.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string lidar_frame_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
