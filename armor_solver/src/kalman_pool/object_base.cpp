//
// Created by lbw on 25-3-10.
//

#include "armor_solver/kalman_pool/object_base.h"

#include <clock/clock.h>
#include <visualization/visualization.h>

ObjectBase::ObjectBase(const std::string& ID)
{
    id = ID;
    last_time_ = global_node::Clock->time();
}

bool ObjectBase::isDetectTracking()
{
    return track_state_ == KalmanCommon::Detect_Tracking;
}

bool ObjectBase::MainDetect()
{
    last_lost_time_ = global_node::Clock->time();
    detect_frame_count_++;
    if (track_state_ == KalmanCommon::Sensing_Lost ||
        track_state_ == KalmanCommon::Sensing_Detecting ||
        track_state_ == KalmanCommon::Main_Lost)
    {
        first_frame_ = true;
        track_state_ = KalmanCommon::Detect_Fitting;
        return true;
    }
    if (track_state_ == KalmanCommon::Detect_Temp_Lost)
    {
        first_frame_ = false;
        track_state_ = KalmanCommon::Detect_Tracking;
    }
    else if (track_state_ == KalmanCommon::Detect_Fitting && detect_frame_count_ >= 10)
    {
        first_frame_ = false;
        track_state_ = KalmanCommon::Detect_Tracking;
    }
    else if (track_state_ == KalmanCommon::Detect_Tracking)
    {
        first_frame_ = false;
    }
    first_frame_ = false;
    return false;
}

void ObjectBase::MainLost()
{
    detect_frame_count_ = 0;
    first_frame_ = false;

    double lost_time;
    if (id == "outpost")
    {
        lost_time = 1.0;
    }
    else
    {
        lost_time = 0.2;
    }
    if (track_state_ == KalmanCommon::Detect_Fitting || track_state_ == KalmanCommon::Detect_Tracking)
    {
        track_state_ = KalmanCommon::Detect_Temp_Lost;
    }
    else if (track_state_ == KalmanCommon::Detect_Temp_Lost)
    {
        auto duration = global_node::Clock->time() - last_lost_time_;
        if (duration.seconds() > lost_time)
        {
            track_state_ = KalmanCommon::Detect_Lost;
        }
    }
}

void ObjectBase::SensorDetect()
{
    if (track_state_ == KalmanCommon::Sensing_Lost ||
        track_state_ == KalmanCommon::Detect_Lost)
        track_state_ = KalmanCommon::Sensing_Detecting;
}

void ObjectBase::SensorLost()
{
    if (track_state_ == KalmanCommon::Sensing_Detecting ||
        track_state_ == KalmanCommon::Detect_Lost)
        track_state_ = KalmanCommon::Sensing_Lost;
}

KalmanCommon::Track_State ObjectBase::getTrackState()
{
    return track_state_;
}
