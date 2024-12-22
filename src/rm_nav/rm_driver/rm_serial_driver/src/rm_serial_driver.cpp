#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
    RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
        : Node("rm_serial_driver", options),
          owned_ctx_{new IoContext(2)},
          serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {
        RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
        getParams();
        // Create Publisher
        all_robot_hp_pub_ =
            this->create_publisher<rm_decision_interfaces::msg::AllRobotHP>("/all_robot_hp", 3);
        robot_status_pub_ =
            this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("/robot_status", 10);
        game_status_pub_ =
            this->create_publisher<rm_decision_interfaces::msg::GameStatus>("/game_status", 1);
        decision_num_pub_ =
            this->create_publisher<rm_decision_interfaces::msg::DecisionNum>("/decision_num", 1);

        try
        {
            serial_driver_->init_port(device_name_, *device_config_);
            if (!serial_driver_->port()->is_open())
            {
                serial_driver_->port()->open();
                receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
            throw ex;
        }
        try
        {
            const std::string cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_chassis");
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The cmd_vek topic name provided was invalid");
            throw ex;
        }
        // Create Subscription
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_chassis", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&RMSerialDriver::sendCtrlDate, this, std::placeholders::_1));
    }

    RMSerialDriver::~RMSerialDriver()
    {
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }

        if (serial_driver_->port()->is_open())
        {
            serial_driver_->port()->close();
        }

        if (owned_ctx_)
        {
            owned_ctx_->waitForExit();
        }
    }

    void RMSerialDriver::getParams()
    {
        using FlowControl = drivers::serial_driver::FlowControl;
        using Parity = drivers::serial_driver::Parity;
        using StopBits = drivers::serial_driver::StopBits;
        uint32_t band_rate{};
        auto fc = FlowControl::NONE;
        auto pt = Parity::NONE;
        auto sb = StopBits::ONE;
        try
        {
            device_name_ = declare_parameter<std::string>("device_name", "");
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
            throw ex;
        }
        try
        {
            band_rate = declare_parameter<int>("band_rate", 0);
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The band_rate provided was invali");
            throw ex;
        }
        try
        {
            const auto fc_string = declare_parameter<std::string>("flow_control", "");
            if (fc_string == "none")
                fc = FlowControl::NONE;
            else if (fc_string == "hardware")
                fc = FlowControl::HARDWARE;
            else if (fc_string == "software")
                fc = FlowControl::SOFTWARE;
            else
            {
                throw std::invalid_argument{
                    "The flow_control parameter must be one of: none, software, or "
                    "hardware."};
            }
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
            throw ex;
        }
        try
        {
            const auto pt_string = declare_parameter<std::string>("parity", "");
            if (pt_string == "none")
            {
                pt = Parity::NONE;
            }
            else if (pt_string == "odd")
            {
                pt = Parity::ODD;
            }
            else if (pt_string == "even")
            {
                pt = Parity::EVEN;
            }
            else
            {
                throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
            }
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
            throw ex;
        }
        try
        {
            const auto sb_string = declare_parameter<std::string>("stop_bits", "");

            if (sb_string == "1" || sb_string == "1.0")
            {
                sb = StopBits::ONE;
            }
            else if (sb_string == "1.5")
            {
                sb = StopBits::ONE_POINT_FIVE;
            }
            else if (sb_string == "2" || sb_string == "2.0")
            {
                sb = StopBits::TWO;
            }
            else
            {
                throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
            }
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }

        device_config_ =
            std::make_unique<drivers::serial_driver::SerialPortConfig>(band_rate, fc, pt, sb);
    }

    void RMSerialDriver::receiveData()
    {
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data;
        data.reserve(sizeof(ReceivePacket));
        while (rclcpp::ok())
        {
            try
            {
                serial_driver_->port()->receive(header);
                if (header[0] == 0x55)
                {
                    data.resize(sizeof(ReceivePacket) - 1);
                    serial_driver_->port()->receive(data);
                    data.insert(data.begin(), header[0]);
                    ReceivePacket packet = fromVector(data);
                    bool crc_ok =
                        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
                    if (crc_ok)
                    {
                        // RCLCPP_INFO(get_logger(), "Receiveing,%d", robot_status_.is_attacked);
                        all_robot_hp_.red_1_robot_hp = packet.red_1_robot_hp;
                        all_robot_hp_.red_2_robot_hp = packet.red_2_robot_hp;
                        all_robot_hp_.red_3_robot_hp = packet.red_3_robot_hp;
                        all_robot_hp_.red_4_robot_hp = packet.red_4_robot_hp;
                        all_robot_hp_.red_5_robot_hp = packet.red_5_robot_hp;
                        all_robot_hp_.red_7_robot_hp = packet.red_7_robot_hp;
                        all_robot_hp_.red_base_hp = packet.red_base_hp;
                        all_robot_hp_.red_outpost_hp = packet.red_outpost_hp;
                        all_robot_hp_.blue_1_robot_hp = packet.blue_1_robot_hp;
                        all_robot_hp_.blue_2_robot_hp = packet.blue_2_robot_hp;
                        all_robot_hp_.blue_3_robot_hp = packet.blue_3_robot_hp;
                        all_robot_hp_.blue_4_robot_hp = packet.blue_4_robot_hp;
                        all_robot_hp_.blue_5_robot_hp = packet.blue_5_robot_hp;
                        all_robot_hp_.blue_7_robot_hp = packet.blue_7_robot_hp;
                        all_robot_hp_.blue_base_hp = packet.blue_base_hp;
                        all_robot_hp_.blue_outpost_hp = packet.blue_outpost_hp;
                        all_robot_hp_pub_->publish(all_robot_hp_);

                        game_status_.game_progress = packet.game_progress;
                        game_status_.stage_remain_time = packet.stage_remain_time;
                        game_status_pub_->publish(game_status_);

                        robot_status_.robot_id = packet.robot_id;
                        robot_status_.current_hp = packet.current_hp;
                        robot_status_.shooter_heat = packet.shooter_heat;
                        robot_status_.team_color = packet.team_color;
                        robot_status_.is_attacked = packet.is_attacked;
                        robot_status_.is_detect_enemy = packet.is_detect_enemy;
                        robot_status_pub_->publish(robot_status_);

                        decision_num_.decision_num = packet.decision_num;
                        decision_num_pub_->publish(decision_num_);
                    }
                }
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
                reopenPort();
            }
        }
    }

    void RMSerialDriver::sendCtrlDate(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        try
        {
            // RCLCPP_INFO(get_logger(), "Sending");
            SendPacket packet;
            packet.vx = msg->linear.x;
            packet.vy = msg->linear.y;
            packet.vz = msg->angular.z;
            // packet.vx = 1.5;
            // packet.vy = 2.5;
            // packet.vz = 3.5;
            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
            std::vector<uint8_t> data = toVector(packet);
            serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            reopenPort();
        }
    }

    void RMSerialDriver::reopenPort()
    {
        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
        try
        {
            if (serial_driver_->port()->is_open())
            {
                serial_driver_->port()->close();
            }
            serial_driver_->port()->open();
            RCLCPP_INFO(get_logger(), "Successfully reopened port");
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
            if (rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::seconds(1));
                reopenPort();
            }
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)