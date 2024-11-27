#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rm_decision_interfaces/msg/all_robot_hp.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/game_status.hpp"
#include "rm_decision_interfaces/msg/decision_num.hpp"
namespace rm_serial_driver
{
    class RMSerialDriver : public rclcpp::Node
    {
    public:
        explicit RMSerialDriver(const rclcpp::NodeOptions &options);
        ~RMSerialDriver() override;

    private:
        void getParams();
        void receiveData();
        void sendCtrlDate(const geometry_msgs::msg::Twist::SharedPtr msg);
        void reopenPort();
        // void setParam(const rclcpp::Parameter &param);

        // serial port
        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
        // create publisher
        rclcpp::Publisher<rm_decision_interfaces::msg::AllRobotHP>::SharedPtr all_robot_hp_pub_;
        rclcpp::Publisher<rm_decision_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
        rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
        rclcpp::Publisher<rm_decision_interfaces::msg::DecisionNum>::SharedPtr decision_num_pub_;

        std::thread receive_thread_;
        const std::string cmd_vel_topic;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        // Receive packet (Referee system)
        rm_decision_interfaces::msg::AllRobotHP all_robot_hp_;
        rm_decision_interfaces::msg::RobotStatus robot_status_;
        rm_decision_interfaces::msg::GameStatus game_status_;
        rm_decision_interfaces::msg::DecisionNum decision_num_;
    };
}
#endif