//
// Created by lbw on 25-3-10.
//

#ifndef KALMAN_POOL_H
#define KALMAN_POOL_H
//std
#include <map>
#include <memory>
#include <future>
#include <utility>
//project
#include "armor_solver/kalman_pool/robot.h"
#include "armor_solver/kalman_pool/outpost.h"
#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/msg/armor.hpp>
#include <rm_interfaces/msg/measurement.hpp>
#include <rm_interfaces/msg/enemy_position.hpp>

#include "armor_solver/kalman_pool/common.h"

class KalmanPool
{
    using Pose = geometry_msgs::msg::Pose;
    using Header = std_msgs::msg::Header;
    using Measurement = rm_interfaces::msg::Measurement;
    using Enemy = rm_interfaces::msg::EnemyPosition;

public:
    KalmanPool();
    ~KalmanPool();
    //初始化
    void createObject(KalmanCommon::ObjectType object_type, const std::string& id);
    bool isDetectTracking(std::string& id);
    //状态机改变
    KalmanCommon::Track_State getTrackState(const std::string& id);
    void changeState(const std::string& id, KalmanCommon::ExternState estate);
    void resetFromSensor(const std::string& id, const Pose& pose);
    void sensorDetect(const Enemy& enemy);
    //卡尔曼预测
    rm_interfaces::msg::Measurement measure(const std::string& id, rm_interfaces::msg::Armors& same_num_armors);
    rm_interfaces::msg::Target predict(const std::string& id);
    rm_interfaces::msg::Target getState(const std::string& id);

    std::map<std::string, rm_interfaces::msg::Measurement> measurements;

    bool isExist(const std::string& id){return tracked_objects_[id]!=nullptr;}

    //Debug
    void pushDebug();
private:
    //卡尔曼池
    std::map<std::string, std::shared_ptr<ObjectBase>> tracked_objects_;
};

inline bool KalmanPool::isDetectTracking(std::string& id)
{
    return tracked_objects_[id]->isDetectTracking();
}

/*!
 *
 * @param id 装甲板id
 * @param header 时间戳，不会被修改
 * @param armor 这个变量可能会被修改，也就是优化选解
 */
#include <visualization/visualization.h>
inline rm_interfaces::msg::Measurement KalmanPool::measure(const std::string& id, rm_interfaces::msg::Armors& same_num_armors )
{
    const auto& object = tracked_objects_[id];
    return object->update(same_num_armors);
}

inline KalmanCommon::Track_State KalmanPool::getTrackState(const std::string& id)
{
    return tracked_objects_[id]->getTrackState();
}

#endif //KALMAN_POOL_H
