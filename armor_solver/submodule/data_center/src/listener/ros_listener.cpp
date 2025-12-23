//
// Created by lbw on 25-5-3.
//

#include "data_center/listener/ros_listener.h"
#include <data_center/data_recorder.h>
#include <clock/clock.h>
#include <parameter/parameter.h>

ckyf::data_center::ROSListener::ROSListener(rclcpp::NodeOptions option): Node("ros_listener", option)
{

    operator_sub_ = this->create_subscription<Operator>("receive/operator",10,
        [this](const Operator& operator_msg)
    {
            global_node::DataRecorder->updateOperator(operator_msg);
    });

    std::thread initTF([this]()
    {
        do
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }while (global_node::Clock==nullptr || global_node::Parameter == nullptr);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(global_node::Clock->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        enemy_sub_ = this->create_subscription<rm_interfaces::msg::EnemyPositions>("multi_combine/enemies",
            rclcpp::SensorDataQoS(),
            std::bind(&ROSListener::enemiesCallback,
                      this, std::placeholders::_1));

        global_node::Parameter->get_parameter("target_frame", target_frame_);
    });
    initTF.detach();
    target_frame_ = "odom";

    //chassis
    chassis_sub_ = this->create_subscription<Chassis>("multi_combine/chassis",10,std::bind(
        &ROSListener::ChassisCallback,this,std::placeholders::_1));

}

void ckyf::data_center::ROSListener::setTargetFrame(const std::string& target_frame)
{
    target_frame_ = target_frame;
}

void ckyf::data_center::ROSListener::enemiesCallback(EnemyPositions::SharedPtr enemy_positions_msg)
{
    try
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = enemy_positions_msg->header;
        for (auto& enemy : enemy_positions_msg->enemies)
        {
            if (!enemy.is_detected)
                continue;
            ps.pose = enemy.pose;
            enemy.pose = tf_buffer_->transform(ps, target_frame_).pose;
            enemy.header.stamp = global_node::Clock->time();
            global_node::DataRecorder->sensorDetectEnemy(enemy);
        }
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
    }
}

void ckyf::data_center::ROSListener::ChassisCallback(Chassis::SharedPtr chassis_msg)
{
    global_node::DataRecorder->updateChassis(*chassis_msg);
}

