#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/twist.hpp"

const std::string CMD_VEL_TOPIC = "base_vel";

class pid_follow_ : public rclcpp::Node
{
public:
    pid_follow_(const rclcpp::NodeOptions &options) : Node("pid_follow_", options)
    {
        RCLCPP_INFO(this->get_logger(), "Start Pid Path Follow");
        // Declare and get parameter
        this->declare_parameter<double>("max_x_speed", 1.0);
        this->declare_parameter<double>("max_y_speed", 1.0);
        this->declare_parameter<double>("p_value", 1.0);
        this->declare_parameter<int>("plan_frequency", 50);
        this->declare_parameter<double>("goal_dist_tolerance", 0.3);

        this->get_parameter("max_x_speed", max_x_speed_);
        this->get_parameter("max_y_speed", max_y_speed_);
        this->get_parameter("p_value", p_value_);
        this->get_parameter("plan_frequency", plan_frequency);
        this->get_parameter("goal_dist_tolerance", goal_dist_tolerance_);
        // 订阅全局路径
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_plan", 10, std::bind(&pid_follow_::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            CMD_VEL_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)));
        // TF broadcaster
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
        // Create a timer
        auto period = std::chrono::milliseconds(1000 / plan_frequency);
        timer_ = this->create_wall_timer(
            period, std::bind(&pid_follow_::timerCallback, this));
    }

private:
    void timerCallback()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform;
            transform = tf2_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            robot_pose_.position.x = transform.transform.translation.x;
            robot_pose_.position.y = transform.transform.translation.y;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            m.getRPY(robot_roll_, robot_pitch_, robot_yaw_);
            // RCLCPP_INFO(this->get_logger(),"Current yaw: %f, Current x: %f, Current y: %f",robot_yaw_,robot_pose_.position.x,robot_pose_.position.y);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failure %s", ex.what());
        }
        if (plan_)
        {
            // 发布
            cmd_pub_->publish(cmd);
            plan_ = false;
        }
        else
        {
            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.angular.z = 0;
            cmd_pub_->publish(cmd);
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg || msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received an empty path.");
            return;
        }
        // 获取路径上的下一个点
        const auto &target_pose = msg->poses[5].pose.position;
        dist_diff_ = hypot(robot_pose_.position.x - (msg->poses[msg->poses.size() - 1].pose.position.x), robot_pose_.position.y - msg->poses[msg->poses.size() - 1].pose.position.y);

        if (dist_diff_ < -100 || dist_diff_ > 100 || dist_diff_ < goal_dist_tolerance_)
        { // 错误判断和目标点忍耐判断
            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.angular.z = 0;
            plan_ = true;
        }

        // 计算偏航角差
        yaw_diff = calculateYawDiff(target_pose, robot_yaw_, robot_pose_);
        vx_global = max_x_speed_ * cos(yaw_diff) * p_value_;
        vy_global = max_y_speed_ * sin(yaw_diff) * p_value_;
        if (dist_diff_ > goal_dist_tolerance_ * 3)
        {
            cmd.linear.x = vx_global;
            cmd.linear.y = vy_global;
            plan_ = true;
        }
        else if (dist_diff_ < goal_dist_tolerance_ * 3 && dist_diff_ > goal_dist_tolerance_)
        {
            cmd.linear.x = vx_global * (dist_diff_ / (3 * goal_dist_tolerance_));
            cmd.linear.y = vy_global * (dist_diff_ / (3 * goal_dist_tolerance_));
            cmd.angular.z = 0;
            plan_ = true;
        }
    }

    double calculateYawDiff(const geometry_msgs::msg::Point &target_pose, double robot_yaw, const geometry_msgs::msg::Pose &robot_pose)
    {
        double target_yaw = atan2(target_pose.y - robot_pose.position.y, target_pose.x - robot_pose.position.x);
        double yaw_diff = target_yaw - robot_yaw;
        // 规范化角度差到 [-π, π] 范围
        while (yaw_diff > M_PI)
            yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI)
            yaw_diff += 2 * M_PI;

        return yaw_diff;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener;

    int plan_frequency;

    geometry_msgs::msg::Pose robot_pose_;
    double robot_yaw_;
    double robot_roll_;
    double robot_pitch_;
    double yaw_diff;
    double dist_diff_;
    rclcpp::TimerBase::SharedPtr timer_;

    double vx_global;
    double vy_global;
    double max_x_speed_;
    double max_y_speed_;
    double p_value_;
    double goal_dist_tolerance_;

    geometry_msgs::msg::Twist cmd;
    bool plan_ = false;
    // rclcpp::Service<robot_msgs::srv::Decision>::SharedPtr decision_server_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_follow_);