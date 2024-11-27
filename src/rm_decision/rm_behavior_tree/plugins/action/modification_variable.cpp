#include "behaviortree_cpp/action_node.h"
#include "rm_behavior_tree/plugins/action/modification_variable.hpp"
#include <iostream>

namespace rm_behavior_tree
{
    ModificationVariableAction::ModificationVariableAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus ModificationVariableAction::tick()
    {
        int variable_output;
        auto variable_input = getInput<int>("variable_input");
        auto mod_num = getInput<int>("mod_num");
        if (!variable_input)
        {
            std::cerr << "Missing required input [message_to_print]" << '\n';
            return BT::NodeStatus::FAILURE;
        }
        std::cout << variable_input.value() << '\n';
        variable_output = variable_input.value() + mod_num.value();
        RCLCPP_INFO(rclcpp::get_logger("print_message: current point:"), "%d", variable_output);
        setOutput<int>("variable_output", variable_output);
        return BT::NodeStatus::SUCCESS;
    }
} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::ModificationVariableAction>("ModificationVariable");
}