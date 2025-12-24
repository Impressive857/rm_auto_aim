#ifndef _PLANNER_HPP_
#define _PLANNER_HPP_

//
#include "armor_solver_common.h"
#include "target_predictor.hpp"
#include "clock/clock.h"
#include "parameter/parameter.h"

// 3rdparty
#include "tinympc/tiny_api.hpp"

// ros

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
            using Trajectory = Eigen::Matrix<double, 4, HORIZON>;
        public:
            struct Plan {
                double yaw;
                double pitch;
            };
        public:
            Planner();
            ~Planner();
            bool init();
            Plan get_plan();
        private:
            void setup_yaw_solver();
            void setup_pitch_solver();
            Trajectory get_trajectory(const double yaw0);
            std::optional<std::pair<double, double>> trajectory(const double v0, const double d, const double h);
            std::pair<double, double> aim(const std::tuple<double, double, double>& target_xyz);
        private:
            TinySolver* m_yaw_solver;
            TinySolver* m_pitch_solver;
            TargetPredictor m_target_predictor;
            double m_yaw_offset;
            double m_pitch_offset;
            double m_bullet_speed;
        };
    }
}

#endif // ! _PLANNER_HPP_