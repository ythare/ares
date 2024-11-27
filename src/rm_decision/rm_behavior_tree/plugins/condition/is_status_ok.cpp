#include "rm_behavior_tree/plugins/condition/is_status_ok.hpp"

namespace rm_behavior_tree
{

IsStatusOKAction::IsStatusOKAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusOKAction::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsStatusOKAction::checkRobotStatus()
{
  int hp_threshold, heat_threshold;
  auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("message");
  getInput("hp_threshold", hp_threshold);
  getInput("heat_threshold", heat_threshold);

  if (!msg) {
    // throw BT::RuntimeError("missing required input [game_status]: ", msg.error());
    // std::cout << "missing required input [game_status]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  if (msg->current_hp < hp_threshold || msg->shooter_heat > heat_threshold) {
    // std::cout << "血量/热量达到预警值" << '\n';
    return BT::NodeStatus::FAILURE;
  } else {
    // std::cout << "血量/热量正常" << '\n';
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsStatusOKAction>("IsStatusOK");
}
