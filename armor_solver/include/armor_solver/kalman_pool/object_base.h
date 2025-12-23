//
// Created by lbw on 25-3-10.
//

#ifndef OBJECTBASE_H
#define OBJECTBASE_H
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <armor_solver/kalman_pool/common.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rm_interfaces/msg/armor.hpp>
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/msg/measurement.hpp>

class ObjectBase
{
public:
    explicit ObjectBase(const std::string& ID);
    virtual ~ObjectBase() = default;
    virtual void init() = 0;
    virtual rm_interfaces::msg::Measurement update(rm_interfaces::msg::Armors& same_num_armors) = 0;
    [[nodiscard]] virtual rm_interfaces::msg::Target getState() = 0;
    [[nodiscard]] virtual rm_interfaces::msg::Target predict() = 0;
    [[nodiscard]] virtual rm_interfaces::msg::Target predict(double dt_) = 0;

    bool isDetectTracking();

    [[nodiscard]] virtual bool MainDetect();
    virtual void MainLost();
    virtual void SensorDetect();
    virtual void SensorLost();
    [[nodiscard]] KalmanCommon::Track_State getTrackState();

    virtual void resetFromSensor(const geometry_msgs::msg::Pose& pose) = 0;

    std::string id;

    template<typename T>
    int sign(T num)
    {
        if (num > 0) return 1;
        if (num < 0) return -1;
        return 0;
    }
protected:
    KalmanCommon::Track_State track_state_ = KalmanCommon::Track_State::Sensing_Lost;
    rclcpp::Time last_time_;

    int detect_frame_count_ = 0;
    bool first_frame_ = true;
private:
    rclcpp::Time last_lost_time_;
};

#endif //OBJECTBASE_H
