#include "rm_behavior_tree/plugins/condition/is_equal.hpp"
#include "rclcpp/rclcpp.hpp"
namespace rm_behavior_tree
{

    IsEqualAction::IsEqualAction(const std::string &name, const BT::NodeConfig &config)
        : BT::SimpleConditionNode(name, std::bind(&IsEqualAction::checkIsEqual, this), config)
    {
    }

    BT::NodeStatus IsEqualAction::checkIsEqual()
    {
        auto Variable_A = getInput<int>("Variable_A");
        auto Variable_B = getInput<int>("Variable_B");
        if (!Variable_A)
        {
            // std::cout << "missing required input [game_status]" << '\n';
            return BT::NodeStatus::FAILURE;
        }
        // RCLCPP_INFO(rclcpp::get_logger("print_message  Variable a and b:"),
        //             "%d", Variable_A.value());
        if (Variable_A)
        {
            if (Variable_A.value() == Variable_B.value())
                // 超过延时时间
                return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IsEqualAction>("IsEqual");
}
