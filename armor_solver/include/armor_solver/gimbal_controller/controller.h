//
// Created by lbw on 25-3-13.
//

#ifndef TRAJECTORY_H
#define TRAJECTORY_H
//std
#include <functional>
#include <utility>
#include <map>
#include <Eigen/Dense>
//ros
#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
//project
#include "armor_solver/armor_solver_common.h"
#include "armor_solver/kalman_pool/kalman_pool.h"
#include <armor_solver/gimbal_controller/target_adviser.h>
#include <armor_solver/gimbal_controller/controller_tools.h>
#include "rm_utils/logger/log.hpp"
#include <rm_utils/math/trajectory_compensator.hpp>
#include <rm_utils/math/manual_compensator.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/msg/point2d.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "armor_solver/tinympc/tiny_api.hpp"

namespace ckyf
{
    namespace auto_aim
    {
        constexpr double DT = 0.01;
        constexpr int HALF_HORIZON = 50;
        constexpr int HORIZON = HALF_HORIZON * 2;
        using Trajectory = Eigen::Matrix<double, 4, HORIZON>;
        class Controller
        {
        public:
            struct Delay
            {
                double control_delay;
                double prediction_delay;
                double response_delay;
                double trigger_delay;
                double command_delay;
            };

            struct Offset
            {
                double h; /*!水平方向偏置*/
                double v; /*!垂直方向偏置*/
                std::vector<std::string> angle_str; /*!角度偏置*/
            };

            struct Threshold
            {
                double max_tracking_v_yaw;
                double min_switching_v_yaw;
                double side_angle;
                int transfer_thresh;
                double shooting_range_w;
                double shooting_range_h;
                double shooting_range_large_w;
                double shooting_range_large_h;
                double shooting_threshold;
            };
            struct Plan
            {
                bool control;
                bool fire;
                float target_yaw;
                float target_pitch;
                float yaw;
                float yaw_vel;
                float yaw_acc;
                float pitch;
                float pitch_vel;
                float pitch_acc;
            };
            enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;

            enum PredictMode
            {
                PredictNone = 0x00, PredictYaw = 0x01, PredictMove = 0x02, PredictAll = 0x03
            } predict_mode;

            Controller();
            ~Controller() = default;
            void init();
            void setup_yaw_solver();
            void setup_pitch_solver();

            void registerGimbalCmdPub(std::function<void(rm_interfaces::msg::GimbalCmd)> publish_GimbalCmd_);

            void registerTargetPub(std::function<void(rm_interfaces::msg::Target)> publish_Target)
            {
                publish_target_ = publish_Target;
            }

            void registerMeasurementPub(
                std::function<void(rm_interfaces::msg::Measurement)> publish_Measurement)
            {
                publish_measurement_ = publish_Measurement;
            }

            void transCmd();
            void registerTimer(const std::function<rclcpp::Time()>& timer);
            void setDelay(const Delay& delay);
            void setOffset(const Offset& offset);
            void setThreshold(const Threshold& threshold);
            void update_operator_offset(const rm_interfaces::msg::Point2d& operator_offset);
            const Eigen::Vector3d target_xyz() const;
            const double target_distance2d() const;
            const double target_distance3d() const;
            double limit_rad(double angle);

            std::optional<std::pair<double, double>> trajectory(double v0, double d, double h);


            void linkKalmanPool(std::shared_ptr<KalmanPool> kalman_pool) { kalman_pool_ = kalman_pool; }

            std::string target_id;
        private:
            std::string target_frame_;
            rm_interfaces::msg::Target target_;
            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
            std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

            TinySolver* yaw_solver_;
            TinySolver* pitch_solver_;

            //目标选择
            TargetAdviser* adviser_{ nullptr };
            std::shared_ptr<KalmanPool> kalman_pool_{ nullptr };

            std::array<double, 3> rpy_{};
            std::function<rclcpp::Time()> timer_;
            std::function<void(rm_interfaces::msg::GimbalCmd)> publish_GimbalCmd_;
            std::function<void(rm_interfaces::msg::Target)> publish_target_;
            std::function<void(rm_interfaces::msg::Measurement)> publish_measurement_;
            std::unique_ptr<fyt::TrajectoryCompensator> trajectory_compensator_{ nullptr };
            std::unique_ptr<fyt::ManualCompensator> manual_compensator_;
            Delay delay_{};
            Offset offset_;
            Threshold threshold_{};
            rm_interfaces::msg::Point2d operator_offset_;
            int overflow_count_{};

            double side_angle_ = M_PI / 2; //弧度
            double cosTheta_ = 0;
            double frequency_ = 10.0;
            double wild_coefficient_ = 3.0;

            double vx, vy, vz, v_yaw;
            double r;

            Tools::SigmaCalculator<10> sigma_vx_;
            Tools::SigmaCalculator<10> sigma_vy_;

            double top_freq_;
            double top_ampl_;
            double jump_time_;

            double last_cmd_yaw_;

            double yaw_offset_;
            double pitch_offset_;


            //判断准心与目标重合度
            [[nodiscard]] bool isOnTarget(double cur_yaw,
                double cur_pitch,
                double target_yaw,
                double target_pitch,
                double distance) const noexcept;

            void calcYawAndPitch(const Eigen::Vector3d& p,
                double& yaw,
                double& pitch) const noexcept;

            Eigen::Vector3d getBestArmorPositions(const Eigen::Vector3d& target_center,
                Eigen::Vector3d& really_armor_pos,
                double target_yaw,
                double r1,
                double r2,
                double d_height,
                size_t armors_num,
                double controller_delay,
                double response_delay) noexcept;

            Eigen::Vector3d getBestArmorPositions2(const Eigen::Vector3d& target_center,
                double target_yaw,
                double r1,
                double r2,
                double d_height,
                size_t armors_num,
                double controller_delay
            ) noexcept;
            int getArmorIdWithDelay(const Eigen::Vector3d& target_center,
                double target_yaw,
                double r1,
                double r2,
                double d_height,
                size_t armors_num) noexcept;
            Plan getPlan();
            Eigen::Matrix<double, 2, 1> aim();
            Trajectory get_trajectory(const double yaw0);

        };

        inline void Controller::registerGimbalCmdPub(
            std::function<void(rm_interfaces::msg::GimbalCmd)> publish_GimbalCmd)
        {
            publish_GimbalCmd_ = std::move(publish_GimbalCmd);
        }

        inline void Controller::registerTimer(const std::function<rclcpp::Time()>& timer)
        {
            timer_ = timer;
        }

        inline void Controller::setDelay(const Delay& delay)
        {
            delay_ = delay;
        }

        inline void Controller::setThreshold(const Threshold& threshold)
        {
            threshold_ = threshold;
            threshold_.side_angle *= M_PI / 180.; //转换成弧度
        }

        inline void Controller::update_operator_offset(const rm_interfaces::msg::Point2d& operator_offset)
        {
            operator_offset_ = operator_offset;
        }
    }
}
#endif //TRAJECTORY_H