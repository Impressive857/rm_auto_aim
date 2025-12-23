//
// Created by lbw on 25-5-14.
//

#include "armor_solver/kalman_pool/motion_model/translate.h"

#include <angles/angles.h>
#include <parameter/parameter.h>

#include "clock/clock.h"
#include "visualization/visualization.h"

MotionModel::Translate::Translate(const std::string& id): ObjectBase(id)
{
    Translate::init();
}

MotionModel::Translate::~Translate() = default;

void MotionModel::Translate::init()
{
    getParam();

    cal_A = [this](double dt)
    {
        Eigen::Matrix<double, X_N, X_N> A;

        A << 1, dt, 0.5 * dt * dt, 0, 0, 0, 0, 0, 0,
            0, 1, dt, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, dt, 0.5 * dt * dt, 0, 0, 0,
            0, 0, 0, 0, 1, dt, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, dt, 0.5 * dt * dt,
            0, 0, 0, 0, 0, 0, 0, 1, dt,
            0, 0, 0, 0, 0, 0, 0, 0, 1;
        return A;
    };

    Eigen::Matrix<double, Z_N, X_N> H;
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0;

    auto u_q = [this]()
    {
        Eigen::MatrixXd Q;
        double Q_ax, Q_ay, Q_az;
        Q_ax = noise_.s2qx;
        Q_ay = noise_.s2qy;
        Q_az = noise_.s2qz;
        double dt = dt_;
        Q.resize(X_N, X_N);
        Eigen::MatrixXd U;
        U.resize(Z_N, Z_N);
        U << Q_ax, 0, 0,
            0, Q_ay, 0,
            0, 0, Q_az;
        Eigen::MatrixXd G;
        G.resize(X_N, Z_N);
        G << pow(dt, 3) / 6.0f, 0, 0,
            0.5f * pow(dt, 2), 0, 0,
            dt, 0, 0,
            0, pow(dt, 3) / 6.0f, 0,
            0, 0.5f * pow(dt, 2), 0,
            0, dt, 0,
            0, 0, pow(dt, 3) / 6.0f,
            0, 0, 0.5f * pow(dt, 2),
            0, 0, dt;
        Q = G * U * G.transpose();
        return Q;
    };

    auto u_r = [this](const Eigen::Matrix<double, Z_N, 1>& z)
    {
        Eigen::Matrix<double, Z_N, Z_N> r;

        r << noise_.r_x * std::abs(z(0)), 0, 0,
            0, noise_.r_y * std::abs(z(1)), 0,
            0, 0, noise_.r_z * std::abs(z(2));
        return r;
    };

    Eigen::Matrix<double, X_N, X_N> P0;
    P0.setIdentity();
    kf_ = std::make_unique<RobotStateKF>(cal_A(0.05), H, P0, u_q, u_r);

    target_state = Eigen::VectorXd::Zero(X_N);
}

void MotionModel::Translate::getParam()
{
    global_node::Parameter->get_parameter("translate.q_x", noise_.s2qx);
    global_node::Parameter->get_parameter("translate.q_y", noise_.s2qy);
    global_node::Parameter->get_parameter("translate.q_z", noise_.s2qz);

    global_node::Parameter->get_parameter("translate.r_x", noise_.r_x);
    global_node::Parameter->get_parameter("translate.r_y", noise_.r_y);
    global_node::Parameter->get_parameter("translate.r_z", noise_.r_z);

    global_node::Parameter->get_parameter("translate.max_match_distance", max_match_distance_);
    //std::cout << noise_ << std::endl;
}

rm_interfaces::msg::Measurement MotionModel::Translate::update(rm_interfaces::msg::Armors& same_num_armors)
{
    if (first_frame_)
    {
        reset(same_num_armors.header.stamp, same_num_armors.armors[0]);
        dt_ = 0.05;
    }
    else
    {
        rclcpp::Time time = same_num_armors.header.stamp;
        dt_ = (time - last_time_).seconds();
        last_time_ = time;
    }
    kf_->Aset(cal_A(dt_));

    rm_interfaces::msg::Armor tracked_armor;
    // KF predict
    Eigen::VectorXd kf_prediction = kf_->predict();
    auto predicted_position = getArmorPositionFromState(kf_prediction);
    target_state = kf_prediction;
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    for (const auto& armor : same_num_armors.armors)
    {
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();

        if (position_diff < min_position_diff)
        {
            // Find the closest armor
            min_position_diff = position_diff;
            // yaw_diff = abs(orientationToYaw(armor.pose.orientation) - kf_prediction(6));
            tracked_armor = armor;
        }
    }

    if (min_position_diff <= max_match_distance_)
    {
        auto p = tracked_armor.pose.position;
        measurement = Eigen::Vector3d(p.x, p.y, p.z);
        target_state = kf_->update(measurement);
        overflow_count_ = std::max(0,overflow_count_-1);
    }
    else
    {
        overflow_count_++;
        if (overflow_count_ > 5)
        {
            reset(global_node::Clock->time(), tracked_armor);
            // No matched armor found
            FYT_WARN("armor_solver", "[Translate] Diff! Reset");
        }
    }

    rm_interfaces::msg::Measurement measurement_msg;
    measurement_msg.x = measurement(0);
    measurement_msg.y = measurement(1);
    measurement_msg.z = measurement(2);
    measurement_msg.yaw = 0;
    return measurement_msg;
}

void MotionModel::Translate::reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor& armor)
{
    last_time_ = time;

    double xa = armor.pose.position.x;
    double ya = armor.pose.position.y;
    double za = armor.pose.position.z;
    last_yaw_ = 0;
    double yaw = KalmanCommon::orientationToYaw(armor.pose.orientation);
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;

    // Set initial position at 0.2m behind the target
    target_state = Eigen::VectorXd::Zero(X_N);
    target_state(0) = xa;
    target_state(2) = 0;
    target_state(3) = ya;
    target_state(5) = 0;
    target_state(6) = za;
    target_state(8) = 0;

    kf_->setState(target_state);
}

Eigen::MatrixXd MotionModel::Translate::getState()
{
    return target_state;
}

rm_interfaces::msg::Target MotionModel::Translate::predict()
{
    rclcpp::Time now = global_node::Clock->time();
    auto duration = now - last_time_;
    kf_->Aset(cal_A(dt_));
    last_time_ = now;
    target_state = kf_->predict();

    rm_interfaces::msg::Target target;
    target.position.x = target_state(0);
    target.velocity.x = target_state(1);
    target.acceleration.x = target_state(2);
    target.position.y = target_state(3);
    target.velocity.y = target_state(4);
    target.acceleration.y = target_state(5);
    target.position.z = target_state(6);
    target.velocity.z = target_state(7);
    target.acceleration.z = target_state(8);
    target.yaw = 0;
    target.v_yaw = 0;
    target.radius_1 = 0;
    target.radius_2 = 0;
    target.d_zc = 0;
    target.armors_num = 1;
    return target;
}

void MotionModel::Translate::resetFromSensor(const geometry_msgs::msg::Pose& pose)
{
    target_state(0) = pose.position.x;
    target_state(1) = 0;
    target_state(2) = 0;
    target_state(3) = pose.position.y;
    target_state(4) = 0;
    target_state(5) = 0;
    target_state(6) = pose.position.z;
    target_state(7) = 0;
 target_state(8) = 0;
    kf_->setState(target_state);

}

Eigen::Vector3d MotionModel::Translate::getArmorPositionFromState(const Eigen::VectorXd& x)
{
    // Calculate predicted position of the current armor
    double xa = x(0), ya = x(3), za = x(6);
    return Eigen::Vector3d(xa, ya, za);
}

double MotionModel::Translate::orientationToYaw(const geometry_msgs::msg::Quaternion& q)
{
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // Make yaw change continuous (-pi~pi to -inf~inf)
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
}
