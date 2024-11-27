#include "rm_behavior_tree/plugins/condition/is_detect_enemy.hpp"

namespace rm_behavior_tree
{

  IsDetectEnemyAction::IsDetectEnemyAction(const std::string &name, const BT::NodeConfig &config)
      : BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyAction::detectEnemyStatus, this), config)
  {
  }

  BT::NodeStatus IsDetectEnemyAction::detectEnemyStatus()
  {
    auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("message");

    if (!msg)
    {
      std::cerr << "Missing required input [message]" << '\n';
      return BT::NodeStatus::FAILURE;
    }
    if (msg->is_detect_enemy)
    {
      // std::cout << "检测到敌人" << '\n';
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsDetectEnemyAction>("IsDetectEnemy");
}