//
// Created by lbw on 25-5-4.
//

#include <clock/clock.h>
#include <rm_utils/logger/log.hpp>

ckyf::clock::Clock::Clock(const rclcpp::NodeOptions& options):Node("clock",options)
{
    FYT_REGISTER_LOGGER("clock", "~/fyt2024-log", INFO);
    FYT_INFO("clock", "Clock initialized");
}

#include <register_node_macro.h>
REGISTER_NODE(ckyf::clock::Clock)
namespace global_node
{
    std::shared_ptr<ckyf::clock::Clock> Clock;
}
