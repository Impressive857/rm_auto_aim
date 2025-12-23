//
// Created by lbw on 25-3-10.
//

#ifndef KALMAN_POOL_COMMON_H
#define KALMAN_POOL_COMMON_H
#include <Eigen/Dense>
#include <memory>
#include <rm_utils/math/extended_kalman_filter.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rm_interfaces/msg/debug_back.hpp>
#define NoTarget "NoTarget"

namespace KalmanCommon
{
    enum ObjectType
    {
        Robot,
        Outpost,
        FixTop,
        Unknown,
    };

    inline ObjectType id_to_type(const std::string& id)
    {
        if (id == "outpost")
            return Outpost;
        if (id == "1" || id == "2" ||
            id == "3" || id == "4" ||
            id == "5" || id == "sentry")
            return Robot;
        std::cerr << "Unknown id: " << id << std::endl;
        return Unknown;
    }

    enum Track_State
    {
        Detect_Tracking = 5,
        Detect_Temp_Lost = 4,
        Detect_Fitting = 3,
        Sensing_Detecting = 2,
        Detect_Lost = 1,
        Sensing_Lost = 0,
    };

    enum ExternState
    {
        Main_Detect,
        Main_Lost,
        Sensor_Detect,
        Sensor_Lost,
    };

    inline std::string trackState_to_str(Track_State state)
    {
        switch (state)
        {
        case Track_State::Detect_Tracking:
            return "Detect_Tracking";
        case Track_State::Detect_Temp_Lost:
            return "Detect_Temp_Lost";
        case Track_State::Detect_Fitting:
            return "Detect_Fitting";
        case Track_State::Sensing_Detecting:
            return "Sensing_Detecting";
        case Track_State::Sensing_Lost:
            return "Sensing_Lost";
        case Track_State::Detect_Lost:
            return "Detect_Lost";
        default:
            return "ERROR";
        }
    }

    inline double orientationToYaw(const geometry_msgs::msg::Quaternion& q) noexcept
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    inline double orientationToPitch(const geometry_msgs::msg::Quaternion& q) noexcept
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return pitch;
    }
}


#endif //KALMAN_POOL_COMMON_H
