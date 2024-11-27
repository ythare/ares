#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/robot_status.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断机器人是否被攻击掉血
 * @param[in] message 机器人状态话题id
 */
class IsAttakedAction : public BT::SimpleConditionNode
{
public:
  IsAttakedAction(const std::string & name, const BT::NodeConfig & config);

  // BT::NodeStatus checkGameStart(BT::TreeNode & self_node)
  BT::NodeStatus checkRobotAttacked();

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<rm_decision_interfaces::msg::RobotStatus>("message")};
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_