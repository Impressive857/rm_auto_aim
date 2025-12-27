#ifndef _PLANNER_HPP_
#define _PLANNER_HPP_

//
#include "armor_solver/armor_solver_common.h"
#include "target_predictor.hpp"
#include "clock/clock.h"
#include "parameter/parameter.h"
#include <rm_utils/math/trajectory_compensator.hpp>
#include <rm_utils/math/manual_compensator.hpp>


// 3rdparty
#include <armor_solver/tinympc/tiny_api.hpp>
#include <angles/angles.h>

// ros
#include "rm_interfaces/msg/target.hpp"

// Eigen
#include <Eigen/Dense>

namespace ckyf {
    namespace auto_aim {
        class Planner {
        public:
            static constexpr double DT = 0.01;
            static constexpr int HALF_HORIZON = 50;
            static constexpr int HORIZON = HALF_HORIZON * 2;
            static constexpr double G = 9.7833;
            static constexpr double MAX_PITCH_ACC = 100;
            static constexpr double MAX_YAW_ACC = 100;
            using Trajectory = Eigen::Matrix<double, 4, HORIZON>;
        public:
            enum class TrackMode {
                TRACK_ARMOR = 0,
                TRACK_CENTER = 1
            };
            struct Plan {
                double yaw;
                double pitch;
            };
            struct Delay
            {
                double control_delay;
                double prediction_delay;
                double response_delay;
                double trigger_delay;
                double command_delay;
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
        public:
            Planner();
            ~Planner();
            bool init();
            void set_target(const rm_interfaces::msg::Target& target) noexcept;
            void set_track_mode(TrackMode track_mode) noexcept;
            TrackMode get_track_mode() const noexcept;
            Plan get_plan();
        private:
            void setup_yaw_solver();
            void setup_pitch_solver();
            Trajectory get_trajectory(const double yaw0);
            std::pair<double, double> cal_yaw_pitch(const Eigen::Vector3d& p) const noexcept;
            std::pair<double, double> cal_yaw_pitch(const Eigen::Vector4d& p) const noexcept;
        private:
            std::unique_ptr<fyt::TrajectoryCompensator> m_trajectory_compensator{ nullptr };
            std::unique_ptr<fyt::ManualCompensator> m_manual_compensator;
            TinySolver* m_yaw_solver;
            TinySolver* m_pitch_solver;
            TargetPredictor m_target_predictor;
            int m_overflow_count;
            TrackMode m_track_mode;
            Delay m_delay;
            Threshold m_threshold;
            const Eigen::MatrixXd m_A{ {1, DT}, {0, 1} };
            const Eigen::MatrixXd m_B{ {0}, {DT} };
            const Eigen::VectorXd m_f{ {0, 0} };
            Eigen::Matrix<double, 2, 1> m_Q;
            Eigen::Matrix<double, 1, 1> m_R;
        };
    }
}

#endif // ! _PLANNER_HPP_