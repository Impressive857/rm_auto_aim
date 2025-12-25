#ifndef _PLANNER_HPP_
#define _PLANNER_HPP_

//
#include "armor_solver/armor_solver_common.h"
#include "target_predictor.hpp"
#include "clock/clock.h"
#include "parameter/parameter.h"

// 3rdparty
#include "armor_solver/tinympc/tiny_api.hpp"

// ros
#include "rm_interfaces/msg/target.hpp"

// Eigen
#include <Eigen/Dense>

namespace ckyf {
    namespace auto_aim {
        class Planner {
        public:
            static constexpr double DT = 0.01;
            static constexpr int MIN_HALF_HORIZON = 50;
            static constexpr int MIN_HORIZON = MIN_HALF_HORIZON * 2;
            static constexpr int MAX_HORIZON = 200;
            static constexpr double G = 9.7833;
            static constexpr double MAX_PITCH_ACC = 100;
            static constexpr double MAX_YAW_ACC = 100;
            using Trajectory = Eigen::Matrix<double, 4, -1>;
        public:
            struct Plan {
                double yaw;
                double pitch;
            };
        public:
            Planner();
            ~Planner();
            bool init();
            bool set_target(const rm_interfaces::msg::Target& target);
            Plan get_plan();
        private:
            void setup_yaw_solver();
            void setup_pitch_solver();
            void set_solver_horizon(int horizion);
            void set_yaw_solver_horizon(int horizion);
            void set_pitch_solver_horizon(int horizion);
            Trajectory get_trajectory(const double yaw0);
            std::optional<std::pair<double, double>> trajectory(const double v0, const double d, const double h);
            std::pair<double, double> aim(const std::tuple<double, double, double>& target_xyz);
        private:
            TinySolver* m_yaw_solver;
            TinySolver* m_pitch_solver;
            TargetPredictor m_target_predictor;
            int m_current_horizon;
            int m_current_half_horizon;
            double m_yaw_offset;
            double m_pitch_offset;
            double m_bullet_speed;
            const Eigen::MatrixXd m_A{ {1, DT}, {0, 1} };
            const Eigen::MatrixXd m_B{ {0}, {DT} };
            const Eigen::VectorXd m_f{ {0, 0} };
            Eigen::Matrix<double, 2, 1> m_Q;
            Eigen::Matrix<double, 1, 1> m_R;
        };
    }
}

#endif // ! _PLANNER_HPP_