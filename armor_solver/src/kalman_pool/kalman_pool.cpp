//
// Created by lbw on 25-3-10.
//
#include "armor_solver/kalman_pool/kalman_pool.h"

#include <rm_utils/logger/log.hpp>
#include <parameter/parameter.h>
#include <visualization/visualization.h>

KalmanPool::KalmanPool() = default;

KalmanPool::~KalmanPool() = default;

void KalmanPool::createObject(KalmanCommon::ObjectType object_type, const std::string& id)
{
    switch (object_type)
    {
    case KalmanCommon::ObjectType::Robot:
        if (tracked_objects_[id] == nullptr)
        {
            tracked_objects_[id] = std::make_shared<Object::Robot>(id);
        }
        break;
    case KalmanCommon::ObjectType::Outpost:
        if (tracked_objects_[id] == nullptr)
        {
            tracked_objects_[id] = std::make_shared<Object::Outpost>(id);
        }
        break;
    default:
        break;
    }
    tracked_objects_[id]->id = id;
    tracked_objects_[id]->init();
}

void KalmanPool::resetFromSensor(const std::string& id, const Pose& pose)
{
    auto& object = tracked_objects_[id];
    object->resetFromSensor(pose);
}

void KalmanPool::sensorDetect(const Enemy& enemy)
{
    if (isExist(enemy.id))
    {
        auto object = tracked_objects_[enemy.id];
        if (enemy.is_detected)
        {
            object->SensorDetect();
            resetFromSensor(enemy.id, enemy.pose);
        }
        else
        {
            object->SensorLost();
        }
    }
}

rm_interfaces::msg::Target KalmanPool::predict(const std::string& id)
{
    const auto& object = tracked_objects_[id];
    rm_interfaces::msg::Target target_msg;

    // auto predict = is_detect ? object->getState() : object->predict();
    target_msg = object->predict();

    target_msg.tracking = object->getTrackState() == KalmanCommon::Detect_Tracking ||
        object->getTrackState() == KalmanCommon::Detect_Temp_Lost;
    target_msg.id = id;
    return target_msg;
}
rm_interfaces::msg::Target KalmanPool::predict(const std::string& id,double dt_)
{
    const auto& object = tracked_objects_[id];
    rm_interfaces::msg::Target target_msg;
    // auto predict = is_detect ? object->getState() : object->predict();
    target_msg = object->predict(dt_);
    target_msg.tracking = object->getTrackState() == KalmanCommon::Detect_Tracking ||
        object->getTrackState() == KalmanCommon::Detect_Temp_Lost;
    target_msg.id = id;
    return target_msg;
}
rm_interfaces::msg::Target KalmanPool::getState(const std::string& id)
{
    if (isExist(id))
    {
        auto object = tracked_objects_[id];
        return object->getState();
    }
    rm_interfaces::msg::Target target;
    target.id = NoTarget;
    return target;
}

void KalmanPool::pushDebug()
{
    for (const auto& [id, object] : tracked_objects_)
    {
        if (object == nullptr)
        {
            continue;
        }
        global_node::Visualization->debug_kalman_object_state(id, object->getTrackState());
        if (id == "outpost")
        {
            global_node::Visualization->debug_kalman_motion_state(id, "Rotation");
        }
        else if (id != "NoTarget")
        {
            auto robot = std::dynamic_pointer_cast<Object::Robot>(object);
            global_node::Visualization->debug_kalman_motion_state(id, "New");
        }
    }
}

void KalmanPool::changeState(const std::string& id, KalmanCommon::ExternState estate)
{
    const auto& object = tracked_objects_[id];
    switch (estate)
    {
    case KalmanCommon::ExternState::Main_Detect:
        if (object->MainDetect())
        {
        }
        break;
    case KalmanCommon::ExternState::Main_Lost:
        object->MainLost();
        break;
    case KalmanCommon::ExternState::Sensor_Detect:
        object->SensorDetect();
        break;
    case KalmanCommon::ExternState::Sensor_Lost:
        object->SensorLost();
        break;
    default:
        break;
    }
}
