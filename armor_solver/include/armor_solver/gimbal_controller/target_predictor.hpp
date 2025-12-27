#ifndef _TARGET_PREDICTOR_HPP_
#define _TARGET_PREDICTOR_HPP_

//
#include "armor_solver/armor_solver_common.h"

// std
#include <numeric>

// ros
#include <rm_interfaces/msg/target.hpp>

// Eigen
#include <Eigen/Dense>

namespace ckyf
{
    namespace auto_aim
    {
        class TargetPredictor {
        public:
            TargetPredictor();
            TargetPredictor(const rm_interfaces::msg::Target& target);
            void predict_xyz(double dt);
            void predict_yaw(double dt);
            void predict(double dt);
            double target_dist_2d() const;
            double target_dist_3d() const;
            double nearest_armor_dist_2d() const;
            double nearest_armor_dist_3d() const;
            Eigen::Vector3d get_best_armor_position() const noexcept;
            void set_target(const rm_interfaces::msg::Target& target);
            bool has_target() const;
            ~TargetPredictor() = default;
        public:
            Eigen::Vector3d target_pos_;
            Eigen::Vector3d target_v_;
            double target_yaw_;
            double target_v_yaw_;
        private:
            int get_armor_id() const noexcept;
        private:
            double m_r1;
            double m_r2;
            double m_d_height;
            size_t m_armor_num;
            bool m_has_target;
        };
    }
}

#endif // ! _TARGET_PREDICTOR_HPP_