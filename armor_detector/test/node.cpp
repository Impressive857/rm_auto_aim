//
// Created by lbw on 25-4-3.
//
#include <armor_detector/armor_detector_node.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fyt::auto_aim::ArmorDetectorNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
}