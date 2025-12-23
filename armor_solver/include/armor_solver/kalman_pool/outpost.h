//
// Created by lbw on 25-3-13.
//

#ifndef OUTPOST_H
#define OUTPOST_H
#include "armor_solver/kalman_pool/common.h"
#include "armor_solver/kalman_pool/object_base.h"

namespace Object
{
    class Outpost : public ObjectBase
    {
    public:
        constexpr static int X_N = 5;
        constexpr static int Z_N = 4;

        struct Predict
        {
            //xc,yc,zc,yaw,w
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

                // angular velocity
                x1[3] += x0[4] * dt;
            }

            double dt;
        };

        struct Measure
        {
            template <typename T>
            void operator()(const T x[X_N], T z[Z_N])
            {
                z[0] = x[0] - ceres::cos(x[3]) * 0.276;
                z[1] = x[1] - ceres::sin(x[3]) * 0.276;
                z[2] = x[2];
                z[3] = x[5];
            }
        };

        struct Noise
        {
            double s2qx = 0.05;
            double s2qy = 0.05;
            double s2qz = 0.05;
            double s2qyaw = 1.0;
            double s2qr = 80.0;
            double s2qw = 0.0001;

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
                os << "r_x: " << noise.r_x << "\t";
                os << "r_y: " << noise.r_y << "\t";
                os << "r_z: " << noise.r_z << "\t";
                os << "r_yaw: " << noise.r_yaw << std::endl;
                return os;
            }
        };

        using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;

        explicit Outpost(const std::string& ID);
        ~Outpost() override;

        void init() override;
        void getParam();

        rm_interfaces::msg::Measurement update(rm_interfaces::msg::Armors& same_num_armors) override;
        rm_interfaces::msg::Target getState() override;
        rm_interfaces::msg::Target predict() override;
        rm_interfaces::msg::Target predict(double dt_) override;

        void exportNoise(Noise& noise);

        void reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor & armor);
        void resetFromSensor(const geometry_msgs::msg::Pose& pose) override;

        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x);
        double orientationToYaw(const geometry_msgs::msg::Quaternion &q);
        void handleArmorJump(const rm_interfaces::msg::Armor &current_armor);
    private:
        double dt_{0.0};
        Noise noise_;
        double max_match_distance_;
        double max_match_yaw_diff_;
        double top_armor_pitch_;

        std::unique_ptr<RobotStateEKF> ekf_;

        Predict* predict_;
        Measure* measure_;

        Eigen::VectorXd measurement;
        Eigen::MatrixXd target_state;

        double last_yaw_ = 0;
        double w_pre;
    };


}

#endif //OUTPOST
