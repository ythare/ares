#include "rm_behavior_tree/plugins/condition/is_delay_s.hpp"

namespace rm_behavior_tree
{

    IsDelaysAction::IsDelaysAction(const std::string &name, const BT::NodeConfig &config)
        : BT::SimpleConditionNode(name, std::bind(&IsDelaysAction::checkDelayTime, this), config)
    {
    }

    BT::NodeStatus IsDelaysAction::checkDelayTime()
    {
        auto msg = getInput<rm_decision_interfaces::msg::GameStatus>("message");
        auto delay_time = getInput<std::int8_t>("delay_time");
        if (start_time < 0)
            start_time = msg->stage_remain_time;
        if (!msg)
        {
            // std::cout << "missing required input [game_status]" << '\n';
            return BT::NodeStatus::FAILURE;
        }
        if (msg)
        {
            if (start_time -
                    getInput<rm_decision_interfaces::msg::GameStatus>("message")->stage_remain_time >=
                delay_time.value())
                // 超过延时时间
                return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IsDelaysAction>("IsDelays");
}
