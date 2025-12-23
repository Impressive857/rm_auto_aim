//
// Created by lbw on 25-3-13.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <armor_solver/kalman_pool/object_base.h>
#include <vector>
#include <map>
#include <memory>
#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/msg/measurement.hpp>
#include <armor_solver/kalman_pool/observer.hpp>

namespace Object
{
    class Robot : public ObjectBase
    {
        struct YawResidual
        {
            YawResidual(double x1, double y1, double x2, double y2,
                        double yaw1, double yaw2, double mu)
                : mu_(mu),
                  x1_(x1), y1_(y1), x2_(x2), y2_(y2),
                  yaw1_(yaw1), yaw2_(yaw2)
            {
            }

            template <typename T>
            bool operator()(const T* const x, const T* const y, T* residual) const
            {
                // 1. 计算对于p1方位角残差
                T dx1 = x[0] - T(x1_);
                T dy1 = y[0] - T(y1_);
                T bearing1 = atan2(dy1, dx1 + T(1e-10)); // 避免除零
                T angle_error1 = bearing1 - T(yaw1_);
                T cost1 = ceres::pow(angle_error1, 2);

                // 2. 计算对于p2方位角残差
                T dx2 = x[0] - T(x2_);
                T dy2 = y[0] - T(y2_);
                T bearing2 = atan2(dy2, dx2 + T(1e-10)); // 避免除零
                T angle_error2 = bearing2 - T(yaw1_);
                T cost2 = ceres::pow(angle_error2, 2);

                // 3. 计算约束惩罚项
                T constraint = (x[0] - T(x1_)) * (x[0] - T(x2_)) +
                    (y[0] - T(y1_)) * (y[0] - T(y2_));
                T penalty = mu_ * ceres::pow(constraint, 2);

                // 4. 总残差 = 目标函数 + 约束惩罚
                residual[0] = cost1 + cost2 + penalty;
                return true;
            }

        private:
            const double mu_;
            const double x1_, y1_, x2_, y2_;
            const double yaw1_, yaw2_;
        };

        struct CenterResidual
        {
            CenterResidual(double xc, double yc, double weight)
                : xc_(xc), yc_(yc), weight_(weight)
            {
            }

            template <typename T>
            bool operator()(const T* const x, const T* const y, T* const residual) const
            {
                T center_x = ceres::pow(x[0] - T(xc_), 2.0);
                T center_y = ceres::pow(y[0] - T(yc_), 2.0);
                residual[0] = (center_x + center_y) * T(weight_);
                return true;
            }

        private:
            const double xc_, yc_;
            const double weight_;
        };

        struct LengthResidual
        {
            LengthResidual(double x1, double y1,
                double x2, double y2):
                x1_(x1), y1_(y1), x2_(x2), y2_(y2)
            {
            }
            template <typename T>
            bool operator()(const T* const x, const T* const y, T* const residual) const
            {
                T r1 = ceres::sqrt(ceres::pow(x[0] - T(x1_), 2.0) + ceres::pow(y[0] - T(y1_), 2.0));
                T r2 = ceres::sqrt(ceres::pow(y[0] - T(x2_), 2.0) + ceres::pow(y[0] - T(y2_), 2.0));
                if (r1 > 0.4 || r1 < 0.12 || r2 > 0.4 || r2 < 0.12)
                {
                    residual[0] = T(1.0e8);
                }
                else
                {
                    residual[0] = T(0.0);
                }
                return true;
            }

            const double x1_, y1_;
            const double x2_, y2_;
        };

    public:
        explicit Robot(const std::string& ID);
        ~Robot() override;

        void init() override;

        rm_interfaces::msg::Measurement update(rm_interfaces::msg::Armors& same_num_armors) override;
        rm_interfaces::msg::Target getState() override;
        rm_interfaces::msg::Target predict() override;
        rm_interfaces::msg::Target predict(double dt_) override;

        void reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor& armor);
        void resetFromSensor(const geometry_msgs::msg::Pose& pose) override;

        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);
        double orientationToYaw(const geometry_msgs::msg::Quaternion& q);
        void handleArmorYawJump(const rm_interfaces::msg::Armor& current_armor);
        void handleArmorPositionJump(const rm_interfaces::msg::Armor& current_armor);

        bool armors_to_center(const rm_interfaces::msg::Armors& armors_msg, Eigen::Vector2d& center);

    private:
        double dt_{};
        std::unique_ptr<Observer> observer_;

        Eigen::VectorXd measurement;
        Eigen::MatrixXd target_state;
        double max_match_distance_{};
        double max_match_yaw_diff_{};
        double r_avg[2];//0是低板，1是高板
        double r_count;

        bool first_observe; //当前id的车在同一场比赛中第一次被观测到
        bool just_reset; //刚刚重置了状态，等待判断是否半径对应出错
        bool radius_error; //半径对应出错-当前追踪对应1，另一个对应0

        void updateTranslate(rm_interfaces::msg::Armor& tracked_armor,
                             Eigen::MatrixXd position_only_predict_yaw,
                             rm_interfaces::msg::Measurement& measurement_msg);

        void iteration_solver(double& x, double& y,
                              double x1, double y1,
                              double x2, double y2,
                              double yaw1, double yaw2,
                              Eigen::Vector2d kalman_center);

    public:
        // To store another pair of armors message
        double d_za{}, another_r{};

        // To store offset relative to the reference plane
        double d_zc{};
        double last_yaw_ = 0;
    };
}

#endif //ROBOT_H
