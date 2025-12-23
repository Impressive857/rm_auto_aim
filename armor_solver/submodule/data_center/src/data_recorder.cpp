//
// Created by lbw on 25-5-3.
//

#include "data_center/data_recorder.h"
#include <global_node_map.hpp>
#include <rm_utils/logger/log.hpp>
#include <global_node_map.hpp>
#include <clock/clock.h>
#include <parameter/parameter.h>


using namespace std::chrono_literals;

ckyf::data_center::DataRecorder::DataRecorder(rclcpp::NodeOptions options): Node("data_center", options)
{
    FYT_REGISTER_LOGGER("data_center", "~/fyt2024-log", INFO);
    enemy_positions_ = std::make_shared<EnemyPositions>();
    update_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                            std::bind(&DataRecorder::update_timer_callback, this));

    strategist = this->create_client<EnemyStrategist>("robot_strategist/choose_robot");

    //TODO 修改这里的Delay
    int delay_count = 0;
    serviceOn = true;
    // global_node::Parameter->get_parameter("debug", debug);
    int delay = 2;
    while (!strategist->wait_for_service(0.5s))
    {
        if (!rclcpp::ok())
        {
            FYT_ERROR("data_center", "[{}] Interrupted while waiting for the service. Exiting.",
                      strategist->get_service_name());
        }
        FYT_INFO("data_center", "[{}] service not available, waiting again...", strategist->get_service_name());
        delay_count++;
        if (delay_count >= delay)
        {
            serviceOn = false;
            break;
        }
    }
    std::thread th([this]()
    {
        while (global_node::Parameter == nullptr)
        {
            std::this_thread::sleep_for(100ms);
        }

        std::vector<std::string> id_vec;
        global_node::Parameter->get_parameter("object_id", id_vec);
        for (const auto& id : id_vec)
        {
            valid_id_.insert(id);
            EnemyPosition enemy;
            enemy.id = id;
            enemy_positions_->enemies.push_back(enemy);
        }
    });
    th.detach();

    FYT_INFO("data_center", "DataRecord Started");
}


bool ckyf::data_center::DataRecorder::registerRobotID(const std::string& robot_id)
{
    if (is_id_valid(robot_id))
    {
        valid_id_.insert(robot_id);
        return true;
    }
    return false;
}

void ckyf::data_center::DataRecorder::mainDetectEnemy(const EnemyPosition& enemy)
{
    std::mutex mutex;
    {
        std::lock_guard instance_lock(mutex);
        auto enemy_iter = std::find_if(enemy_positions_->enemies.begin(),
                                       enemy_positions_->enemies.end(),
                                       [enemy](const EnemyPosition& enemy_)
                                       {
                                           return enemy.id == enemy_.id;
                                       });
        if (enemy_iter == enemy_positions_->enemies.end())
        {
            if (registerRobotID(enemy.id))
            {
                enemy_positions_->enemies.push_back(enemy);
            }
            else
                return;
        }
        else
        {
            enemy_iter->main = true;
            enemy_iter->pose = enemy.pose;
        }

        enemy_iter->header = enemy.header;
        enemy_positions_->header.stamp = global_node::Clock->time();
    }
}

void ckyf::data_center::DataRecorder::sensorDetectEnemy(const EnemyPosition& enemy)
{
    std::mutex mutex;
    {
        std::lock_guard instance_lock(mutex);
        auto enemy_iter = std::find_if(enemy_positions_->enemies.begin(),
                                       enemy_positions_->enemies.end(),
                                       [enemy](const EnemyPosition& enemy_)
                                       {
                                           return enemy.id == enemy_.id;
                                       });
        if (enemy_iter == enemy_positions_->enemies.end())
        {
            if (registerRobotID(enemy.id))
            {
                // enemy.header.stamp = global_node::Clock->time();
                enemy_positions_->enemies.push_back(enemy);
            }
            else
                return;
        }
        else
        {
            enemy_iter->is_detected = enemy.is_detected;
            enemy_iter->occlusion = enemy.occlusion;
            enemy_iter->invincible = enemy.invincible;
            enemy_iter->main = enemy.main;
            enemy_iter->id = enemy.id;
            enemy_iter->hp = enemy.hp;
            enemy_iter->attack_enhance = enemy.attack_enhance;
            enemy_iter->header.stamp = global_node::Clock->time();
            if (!enemy_iter->main)
            {
                enemy_iter->pose = enemy.pose;
            }
        }

        enemy_positions_->header.stamp = global_node::Clock->time();
    }
}

void ckyf::data_center::DataRecorder::updateChassis(const Chassis& chassis)
{
    chassis_state_.header = chassis.header;
    chassis_state_.valid = true;
    chassis_state_.data = chassis;
}

bool ckyf::data_center::DataRecorder::getChassis(Chassis& chassis) const
{
    chassis = chassis_state_.data;
    return chassis_state_.valid;
}

void ckyf::data_center::DataRecorder::updateOperator(const Operator& op)
{
    operator_state_.header.stamp = global_node::Clock->time();
    operator_state_.data = op;
    operator_state_.valid = true;
}

bool ckyf::data_center::DataRecorder::getOperator(Operator& op) const
{
    op = operator_state_.data;
    return operator_state_.valid;
}

rclcpp::Client<ckyf::data_center::EnemyStrategist>::FutureAndRequestId
ckyf::data_center::DataRecorder::ask_for_strategy()
{
    EnemyStrategist::Request::SharedPtr request = std::make_shared<EnemyStrategist::Request>();
    request->header.stamp = global_node::Clock->time();
    request->all_enemy = *enemy_positions_;
    return strategist->async_send_request(request);
}

bool ckyf::data_center::DataRecorder::is_id_valid(const std::string& robot_id)
{
    return std::find(valid_id_.begin(), valid_id_.end(), robot_id) != valid_id_.end();
}

void ckyf::data_center::DataRecorder::update_timer_callback()
{
    rclcpp::Time time = global_node::Clock->time();
    for (auto& enemy : enemy_positions_->enemies)
    {
        auto duration = time - enemy.header.stamp;
        if (enemy.main)
            if (duration.seconds() > 0.1)
            {
                enemy.main = false;
            }
        if (duration.seconds() > 2.5)
        {
            enemy.is_detected = false;
        }
    }

    checkValid(chassis_state_);
    checkValid(operator_state_);

}


#include <register_node_macro.h>
REGISTER_NODE(ckyf::data_center::DataRecorder)
