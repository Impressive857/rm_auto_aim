//
// Created by lbw on 25-5-4.
//

#ifndef CLOCK_H
#define CLOCK_H
#include <rclcpp/rclcpp.hpp>

namespace ckyf
{
    namespace clock
    {
        class Clock : public rclcpp::Node
        {
        public:
            Clock(Clock const&) = delete;
            Clock& operator=(Clock const&) = delete;

            rclcpp::Time time() { return this->now(); }

            Clock(const rclcpp::NodeOptions& options);
        };
    }
}

#include <global_node_map.hpp>
NODE_NAME_DEF(ckyf::clock::Clock,Clock)

#endif //CLOCK_H
