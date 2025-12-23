//
// Created by lbw on 25-5-14.
//

#ifndef TRANSLATE_H
#define TRANSLATE_H
#include <rm_utils/math/kalman_filter.hpp>
#include <armor_solver/kalman_pool/object_base.h>
#include "armor_solver/kalman_pool/common.h"

namespace MotionModel
{
    class Translate : public ObjectBase
    {
        constexpr static int X_N = 9;
        constexpr static int Z_N = 3;

        struct Noise
        {
            double s2qx = 0.05;
            double s2qy = 0.05;
            double s2qz = 0.05;

            double r_x = 4e-4;
            double r_y = 4e-4;
            double r_z = 9e-4;
            double r_yaw = 5e-3;

            friend std::ostream& operator<<(std::ostream& os, const Noise& noise)
            {
                os << "s2qx: " << noise.s2qx << "\t";
                os << "s2qy: " << noise.s2qy << "\t";
                os << "s2qz: " << noise.s2qz << "\t";

                os << "r_x: " << noise.r_x << "\t";
                os << "r_y: " << noise.r_y << "\t";
                os << "r_z: " << noise.r_z << "\t";
                return os;
            }
        };

        using RobotStateKF = KalmanFilter<X_N, Z_N>;


    public:
        Translate(const std::string& id);
        ~Translate() override;
        void init() override;
        void getParam();

        rm_interfaces::msg::Measurement update(rm_interfaces::msg::Armors& same_num_armors) override;
        void reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor& armor);

        Eigen::MatrixXd getState() override;
        rm_interfaces::msg::Target predict() override;

        void resetFromSensor(const geometry_msgs::msg::Pose& pose) override;
        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);
        double orientationToYaw(const geometry_msgs::msg::Quaternion& q);
    private:
        double dt_{0.0};
        Noise noise_;
        double max_match_distance_{};
        double last_yaw_ = 0;

        std::unique_ptr<KalmanFilter<X_N,Z_N>> kf_;

        Eigen::MatrixXd target_state;
        Eigen::MatrixXd measurement;

        int overflow_count_ = 0;

        std::function<Eigen::MatrixXd(double)> cal_A;
    };
}


#endif //TRANSLATE_H
