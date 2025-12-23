//
// Created by lbw on 25-4-15.
//

#ifndef FIX_TOP_H
#define FIX_TOP_H

#include "armor_solver/kalman_pool/common.h"
#include "armor_solver/kalman_pool/object_base.h"
#include <armor_solver/kalman_pool/extended_kalman_filter.hpp>

namespace MotionModel
{
    class FixTop : public ObjectBase
    {
    public:
        constexpr static int X_N = 7;
        constexpr static int Z_N = 4;
        constexpr static int NZ_N = 5;

        struct Predict
        {
            //xc,yc,zc,yaw,v_yaw,r,d_z
            //0, 1, 2, 3, 4,    5, 6
            explicit Predict(double dt)
                : dt(dt)
            {
            }

            template <typename T>
            void operator()(const T x0[X_N], T x1[X_N])
            {
                for (int i = 0; i < X_N; i++)
                {
                    x1[i] = x0[i];
                }

                // v_yaw
                // angular velocity
                x1[3] += x0[4] * dt;
            }

            double dt;
        };

        struct sMeasure
        {
            template <typename T>
            void operator()(const T x[X_N], T z[Z_N])
            {
                z[0] = x[0] - ceres::cos(x[6]) * x[8];
                z[1] = x[1] - ceres::sin(x[6]) * x[8];
                z[2] = x[2] + x[6];
                z[3] = x[3];
            }
        };

        struct dMeasure
        {
            template <typename T>
            void operator()(const T x[X_N], T z[NZ_N])
            {
                z[0] = x[0] - ceres::cos(x[6]) * x[8];
                z[1] = x[1] - ceres::sin(x[6]) * x[8];
                z[2] = x[2] + x[6];
                z[3] = x[3];
                z[4] = x[5];
            }
        };

        struct Noise
        {
            double s2qx = 0.05;
            double s2qy = 0.05;
            double s2qz = 0.05;
            double s2qyaw = 1.0;
            double s2qr = 80.0;
            double s2qd_zc = 80.0;

            double r_x = 4e-4;
            double r_y = 4e-4;
            double r_z = 9e-4;
            double r_yaw = 5e-3;

            friend std::ostream& operator<<(std::ostream& os, const Noise& noise)
            {
                os << "s2qx: " << noise.s2qx << "\t";
                os << "s2qy: " << noise.s2qy << "\t";
                os << "s2qz: " << noise.s2qz << "\t";
                os << "s2qyaw: " << noise.s2qyaw << "\t";
                os << "s2qr: " << noise.s2qr << "\t";
                os << "s2qd_zc" << noise.s2qd_zc << std::endl;
                os << "r_x: " << noise.r_x << "\t";
                os << "r_y: " << noise.r_y << "\t";
                os << "r_z: " << noise.r_z << "\t";
                os << "r_yaw: " << noise.r_yaw << std::endl;
                return os;
            }
        };

        using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, sMeasure>;

        explicit FixTop(const std::string& ID);
        ~FixTop() override;

        void init() override;
        void getParam();

        rm_interfaces::msg::Measurement update(rm_interfaces::msg::Armors& same_num_armors) override;
        Eigen::MatrixXd getState() override;
        rm_interfaces::msg::Target predict() override;

        void reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor& armor);
        void resetFromSensor(const geometry_msgs::msg::Pose& pose) override;

        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);
        double orientationToYaw(const geometry_msgs::msg::Quaternion& q);
        void handleArmorJump(const rm_interfaces::msg::Armor& current_armor);

        bool armors_to_center(const rm_interfaces::msg::Armors& armors_msg, Eigen::Vector2d& center);

    private:
        double dt_{0.0};
        Noise noise_;
        double max_match_distance_{};
        double max_match_yaw_diff_{};

        std::unique_ptr<myself::ExtendedKalmanFilter<X_N, Z_N, Predict, sMeasure>> ekf_;

        Predict* predict_ = nullptr;
        sMeasure* smeasure_ = nullptr;
        dMeasure* dmeasure_ = nullptr;

        Eigen::VectorXd measurement;
        Eigen::MatrixXd target_state;

    public:
        // To store another pair of armors message
        double d_za{}, another_r{};

        // To store offset relative to the reference plane
        double d_zc{};
        double last_yaw_ = 0;
    };
}
#endif //FIX_TOP_H
