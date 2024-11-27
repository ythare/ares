#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_MESSAGE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_MESSAGE_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

namespace rm_behavior_tree
{
    class ModificationVariableAction : public BT::SyncActionNode
    {
    public:
        ModificationVariableAction(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("variable_input"),
                BT::InputPort<int>("mod_num"),
                BT::OutputPort<int>("variable_output")};
        }

        BT::NodeStatus tick() override;
    };
} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PRINT_MESSAGE_HPP_
