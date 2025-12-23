//
// Created by lbw on 25-5-2.
//

#include "armor_solver/gimbal_controller/target_adviser.h"
#include <rm_utils/logger/log.hpp>
#include "armor_solver/kalman_pool/common.h"
#include <parameter/parameter.h>
#include <clock/clock.h>

ckyf::auto_aim::TargetAdviser* ckyf::auto_aim::TargetAdviser::instance_ = nullptr;

std::string ckyf::auto_aim::TargetAdviser::find_min_armor(const ArmorGroup& armor_groups)
{
    //默认选择距离图像中心最近的装甲板作为目标
    double min_to_center_distance = 1e8;
    std::string target_id = NoTarget;
    for (const auto& [id, armors] : armor_groups.armor_group)
    {
        //选择target
        for (auto& armor : armors.armors)
        {
            if (min_to_center_distance > armor.distance_to_image_center)
            {
                min_to_center_distance = armor.distance_to_image_center;
                target_id = id;
            }
        }
    }
    return target_id;
}

ckyf::auto_aim::TargetAdviser::TargetAdviser(): extern_target_advice_(), shoot_target_(), drop_time_(0)
{
    target_lock_ = false;
    advice_state = AdviceState::MinDistance;
    init();
}

void ckyf::auto_aim::TargetAdviser::updateShootTarget()
{
    //确定advice_state情况
    if (advice_state == AdviceState::Lock)
    {
        return;
    }

    //存在外部建议，并且外部建议 timestamp 有效
    if (extern_target_advice_.has_target_advice && examine_timestamp(extern_target_advice_.header.stamp))
    {
        advice_state = AdviceState::Extern;
    }
    else
    {
        extern_target_advice_.has_target_advice = false;
        advice_state = AdviceState::MinDistance;
    }

    //确定射击对象
    shoot_target_.header.stamp = global_node::Clock->time();
    if (advice_state == AdviceState::Extern)
    {
        shoot_target_.id = extern_target_advice_.id;
        shoot_target_.has_target = extern_target_advice_.has_target_advice;
    }
    if (!armor_group_.temp_lost_group.empty())//
    {
        return;
    }
    else if (advice_state == AdviceState::MinDistance)
    {
        armor_group_mutex_.lock();
        std::string target_id = find_min_armor(armor_group_);
        armor_group_mutex_.unlock();

        shoot_target_.id = target_id;
        shoot_target_.has_target = target_id != NoTarget;
    }
}

ckyf::auto_aim::TargetAdviser::~TargetAdviser()
{
    delete instance_;
}

ckyf::auto_aim::TargetAdviser* ckyf::auto_aim::TargetAdviser::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new TargetAdviser();
    }
    return instance_;
}

void ckyf::auto_aim::TargetAdviser::init()
{
    global_node::Parameter->get_parameter("adviser.drop_time", drop_time_);
}

void ckyf::auto_aim::TargetAdviser::lock()
{
    target_lock_ = true;
}

void ckyf::auto_aim::TargetAdviser::unlock()
{
    target_lock_ = false;
}

void ckyf::auto_aim::TargetAdviser::putExternAdvice(const TargetAdvice& advice)
{
    extern_target_advice_ = advice;
    extern_target_advice_.header.stamp = global_node::Clock->time();
}

void ckyf::auto_aim::TargetAdviser::putDetectArmorGroup(const ArmorGroup& armor_group)
{
    armor_group_mutex_.lock();
    armor_group_ = armor_group;
    armor_group_mutex_.unlock();
}

std::string ckyf::auto_aim::TargetAdviser::getShootTarget()
{
    updateShootTarget();
    if (advice_state == AdviceState::Lock)
    {
        return shoot_target_.id;
    }
    if (shoot_target_.has_target)
        return shoot_target_.id;

    return NoTarget;
}

bool ckyf::auto_aim::TargetAdviser::examine_timestamp(rclcpp::Time timestamp)
{
    auto now = global_node::Clock->time();
    auto duration = now - timestamp;
    return duration.seconds() < drop_time_;
}


