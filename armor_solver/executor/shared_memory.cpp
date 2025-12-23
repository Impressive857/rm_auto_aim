//
// Created by lbw on 25-5-6.
//
#include <factory/factory_init_shut.h>

namespace load_factory
{
    namespace map
    {
        std::map<std::string, std::shared_ptr<rclcpp::Node>> node_map;
        std::map<std::string, std::function<rclcpp::Node::SharedPtr()>> factory_map;
        std::map<std::string, std::function<void(std::shared_ptr<rclcpp::Node>)>> global_node_map;
    }
}
