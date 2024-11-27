#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
// referee date
#include "rm_decision_interfaces/msg/all_robot_hp.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/game_status.hpp"
#include "rm_decision_interfaces/msg/decision_num.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

class virtual_serial : public rclcpp::Node
{
private:
    rclcpp::Publisher<rm_decision_interfaces::msg::AllRobotHP>::SharedPtr all_robot_hp_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::DecisionNum>::SharedPtr decision_num_pub_;

    rm_decision_interfaces::msg::AllRobotHP all_robot_hp_;
    rm_decision_interfaces::msg::RobotStatus robot_status_;
    rm_decision_interfaces::msg::GameStatus game_status_;
    rm_decision_interfaces::msg::DecisionNum decision_num_;

    std::thread pub_thread_;

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_event_subscriber_; // OnSetParametersCallbackHandle 变量

public:
    virtual_serial(const rclcpp::NodeOptions &options);
    ~virtual_serial();
    void getParams();
    void pubDate();
    rcl_interfaces::msg::ParameterDescriptor descriptor_Params(int low, int up);
    void parameterCallback(const std::vector<rclcpp::Parameter> &parameters); // 回调函数
};

virtual_serial::virtual_serial(const rclcpp::NodeOptions &options)
    : Node("virtual_serial", options)
{
    RCLCPP_INFO(get_logger(), "Start virtual_serial!");

    all_robot_hp_pub_ =
        this->create_publisher<rm_decision_interfaces::msg::AllRobotHP>("/all_robot_hp", 3);
    robot_status_pub_ =
        this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("/robot_status", 10);
    game_status_pub_ =
        this->create_publisher<rm_decision_interfaces::msg::GameStatus>("/game_status", 1);
    decision_num_pub_ =
        this->create_publisher<rm_decision_interfaces::msg::DecisionNum>("/decision_num", 1);
    getParams();

    pub_thread_ = std::thread(&virtual_serial::pubDate, this);

    parameter_event_subscriber_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
        {
            parameterCallback(parameters);
            return rcl_interfaces::msg::SetParametersResult();
        });
}

virtual_serial::~virtual_serial()
{
    if (pub_thread_.joinable())
    {
        pub_thread_.join();
    }
}

void virtual_serial::pubDate()
{
    while (rclcpp::ok())
    {
        all_robot_hp_pub_->publish(all_robot_hp_);
        robot_status_pub_->publish(robot_status_);
        game_status_pub_->publish(game_status_);
        decision_num_pub_->publish(decision_num_);
    }
}

rcl_interfaces::msg::ParameterDescriptor virtual_serial::descriptor_Params(int low, int up)
{
    if (low > up)
        RCLCPP_ERROR(get_logger(), "Params lower thred is large than the upper");
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "";
    descriptor.name = "name";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = low;
    descriptor.integer_range[0].to_value = up;
    descriptor.integer_range[0].step = 1;
    return descriptor;
}
void virtual_serial::getParams()
{
    declare_parameter<uint16_t>("hp_blue_1_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_blue_2_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_blue_3_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_blue_4_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_blue_5_robot", 200, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_blue_7_robot", 200, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_blue_base", 1000, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_blue_outpost", 1000, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_1_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_2_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_3_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_4_robot", 100, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_5_robot", 200, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_7_robot", 200, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_base", 20, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("hp_red_outpost", 1000, descriptor_Params(0, 1000));

    declare_parameter<uint8_t>("robot_id", 7, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("current_hp", 500, descriptor_Params(0, 1000));
    declare_parameter<uint16_t>("shooter_heat", 20, descriptor_Params(0, 1000));
    declare_parameter<bool>("team_color", 0, descriptor_Params(0, 1));
    declare_parameter<bool>("is_attacked", 0, descriptor_Params(0, 1));
    declare_parameter<bool>("is_detect_enemy", 0, descriptor_Params(0, 1));

    declare_parameter<uint8_t>("game_progress", 4, descriptor_Params(0, 5));
    declare_parameter<uint16_t>("stage_remain_time", 300, descriptor_Params(0, 600));

    declare_parameter<uint8_t>("decision_num", 1, descriptor_Params(0, 5));

    all_robot_hp_.blue_1_robot_hp = this->get_parameter("hp_blue_1_robot").as_int();
    all_robot_hp_.blue_2_robot_hp = this->get_parameter("hp_blue_2_robot").as_int();
    all_robot_hp_.blue_3_robot_hp = this->get_parameter("hp_blue_3_robot").as_int();
    all_robot_hp_.blue_4_robot_hp = this->get_parameter("hp_blue_4_robot").as_int();
    all_robot_hp_.blue_5_robot_hp = this->get_parameter("hp_blue_5_robot").as_int();
    all_robot_hp_.blue_7_robot_hp = this->get_parameter("hp_blue_7_robot").as_int();
    all_robot_hp_.blue_base_hp = this->get_parameter("hp_blue_base").as_int();
    all_robot_hp_.blue_outpost_hp = this->get_parameter("hp_blue_outpost").as_int();

    all_robot_hp_.red_1_robot_hp = this->get_parameter("hp_red_1_robot").as_int();
    all_robot_hp_.red_2_robot_hp = this->get_parameter("hp_red_2_robot").as_int();
    all_robot_hp_.red_3_robot_hp = this->get_parameter("hp_red_3_robot").as_int();
    all_robot_hp_.red_4_robot_hp = this->get_parameter("hp_red_4_robot").as_int();
    all_robot_hp_.red_5_robot_hp = this->get_parameter("hp_red_5_robot").as_int();
    all_robot_hp_.red_7_robot_hp = this->get_parameter("hp_red_7_robot").as_int();
    all_robot_hp_.red_base_hp = this->get_parameter("hp_red_base").as_int();
    all_robot_hp_.red_outpost_hp = this->get_parameter("hp_red_outpost").as_int();

    robot_status_.is_attacked = this->get_parameter("is_attacked").as_bool();
    robot_status_.is_detect_enemy = this->get_parameter("is_detect_enemy").as_bool();
    robot_status_.robot_id = this->get_parameter("robot_id").as_int();
    robot_status_.shooter_heat = this->get_parameter("shooter_heat").as_int();
    robot_status_.team_color = this->get_parameter("team_color").as_bool();
    robot_status_.current_hp = this->get_parameter("current_hp").as_int();

    game_status_.game_progress = this->get_parameter("game_progress").as_int();
    game_status_.stage_remain_time = this->get_parameter("stage_remain_time").as_int();

    decision_num_.decision_num = this->get_parameter("decision_num").as_int();
}

void virtual_serial::parameterCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    for (auto &param : parameters)
    {
        RCLCPP_INFO(get_logger(), "Param update");
        if (param.get_name() == "hp_blue_1_robot")
        {
            all_robot_hp_.blue_1_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_blue_2_robot")
        {
            all_robot_hp_.blue_2_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_blue_3_robot")
        {
            all_robot_hp_.blue_3_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_blue_4_robot")
        {
            all_robot_hp_.blue_4_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_blue_5_robot")
        {
            all_robot_hp_.blue_5_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_blue_7_robot")
        {
            all_robot_hp_.blue_7_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_blue_outpost")
        {
            all_robot_hp_.blue_outpost_hp = param.as_int();
        }
        else if (param.get_name() == "hp_blue_base")
        {
            all_robot_hp_.blue_base_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_1_robot")
        {
            all_robot_hp_.red_1_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_2_robot")
        {
            all_robot_hp_.red_2_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_3_robot")
        {
            all_robot_hp_.red_3_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_4_robot")
        {
            all_robot_hp_.red_4_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_5_robot")
        {
            all_robot_hp_.red_5_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_7_robot")
        {
            all_robot_hp_.red_7_robot_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_outpost")
        {
            all_robot_hp_.red_outpost_hp = param.as_int();
        }
        else if (param.get_name() == "hp_red_base")
        {
            all_robot_hp_.red_base_hp = param.as_int();
        }
        // robot_status
        else if (param.get_name() == "robot_id")
        {
            robot_status_.robot_id = param.as_int();
        }
        else if (param.get_name() == "current_hp")
        {
            robot_status_.current_hp = param.as_int();
        }
        else if (param.get_name() == "shooter_heat")
        {
            robot_status_.shooter_heat = param.as_int();
        }
        else if (param.get_name() == "team_color")
        {
            robot_status_.team_color = param.as_bool();
        }
        else if (param.get_name() == "is_attacked")
        {
            robot_status_.is_attacked = param.as_bool();
        }
        else if (param.get_name() == "is_detect_enemy")
        {
            robot_status_.is_detect_enemy = param.as_bool();
        }
        // game_status
        else if (param.get_name() == "game_progress")
        {
            game_status_.game_progress = param.as_int();
        }
        else if (param.get_name() == "stage_remain_time")
        {
            game_status_.stage_remain_time = param.as_int();
        }
        // decision_num
        else if (param.get_name() == "decision_num")
        {
            decision_num_.decision_num = param.as_int();
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(virtual_serial)