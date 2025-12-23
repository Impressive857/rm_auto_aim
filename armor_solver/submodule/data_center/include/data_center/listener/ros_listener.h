//
// Created by lbw on 25-5-3.
//

#ifndef ROS_LISTENER_H
#define ROS_LISTENER_H

#include <rclcpp/rclcpp.hpp>
#include <DataCenterCommon>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rm_interfaces/msg/operator.hpp>
#include <rm_interfaces/msg/chassis.hpp>

namespace ckyf
{
    namespace data_center
    {
        using Operator = rm_interfaces::msg::Operator;
        using Chassis = rm_interfaces::msg::Chassis;

        class ROSListener : public rclcpp::Node
        {
        public:
            explicit ROSListener(rclcpp::NodeOptions option);
            ~ROSListener() = default;

            void setTargetFrame(const std::string& target_frame);

        private:
            bool initDone = false;

            rclcpp::Subscription<rm_interfaces::msg::EnemyPositions>::SharedPtr enemy_sub_;
            void enemiesCallback(EnemyPositions::SharedPtr chooser_msg);

            //operator操作手数据
            rclcpp::Subscription<Operator>::SharedPtr operator_sub_;

            //tf信息
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

            //imu数据
            rclcpp::Subscription<Chassis>::SharedPtr chassis_sub_;
            void ChassisCallback(Chassis::SharedPtr chassis_msg);

            //目标frame_id
            std::string target_frame_;
        };
    }
}

#include <register_node_macro.h>
REGISTER_NODE(ckyf::data_center::ROSListener)

#endif //ROS_LISTENER_H
