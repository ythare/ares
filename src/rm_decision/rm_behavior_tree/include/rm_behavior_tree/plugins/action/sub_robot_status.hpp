#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_STATUS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"

namespace rm_behavior_tree
{
class SubRobotStatusAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RobotStatus>
{
public:
  SubRobotStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RobotStatus>("robot_status")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RobotStatus> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_STATUS_HPP_