#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_DELAY_S_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_DELAY_S_HPP_

#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{

    /**
     * @brief condition节点，用于判断机器人是否被攻击掉血
     * @param[in] message 机器人状态话题id
     */
    class IsEqualAction : public BT::SimpleConditionNode
    {
    public:
        IsEqualAction(const std::string &name, const BT::NodeConfig &config);
        // BT::NodeStatus checkGameStart(BT::TreeNode & self_node)
        BT::NodeStatus checkIsEqual();

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("Variable_A"),
                BT::InputPort<int>("Variable_B")};
        }
    };
}
// namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_ATTACKED_HPP_