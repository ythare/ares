#include "rm_behavior_tree/plugins/condition/is_attacked.hpp"

namespace rm_behavior_tree
{

  IsAttakedAction::IsAttakedAction(const std::string &name, const BT::NodeConfig &config)
      : BT::SimpleConditionNode(name, std::bind(&IsAttakedAction::checkRobotAttacked, this), config)
  {
  }

  BT::NodeStatus IsAttakedAction::checkRobotAttacked()
  {
    auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("message");

    if (!msg)
    {
      // std::cout << "missing required input [game_status]" << '\n';
      return BT::NodeStatus::FAILURE;
    }

    if (msg->is_attacked)
    {
      // 机器人受到攻击
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAttakedAction>("IsAttaked");
}
