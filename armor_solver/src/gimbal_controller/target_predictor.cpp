#include "armor_solver/gimbal_controller/target_predictor.hpp"

#include <iostream>

namespace ckyf::auto_aim
{
    TargetPredictor::TargetPredictor() {
        m_has_target = false;
        m_armor_num = 0;
    }

    TargetPredictor::TargetPredictor(const rm_interfaces::msg::Target& target) {
        set_target(target);
    }

    void TargetPredictor::predict_xyz(double dt) {
        target_pos_ += dt * target_v_;

    }
    void TargetPredictor::predict_yaw(double dt) {
        target_yaw_ += dt * target_v_yaw_;
        target_yaw_ = limit_rad(target_yaw_);
    }

    void TargetPredictor::predict(double dt) {
        predict_xyz(dt);
        predict_yaw(dt);
    }

    double TargetPredictor::target_dist_2d() const {
        return target_pos_.head<2>().norm();
    }

    double TargetPredictor::target_dist_3d() const {
        return target_pos_.head<3>().norm();
    }

    double TargetPredictor::nearest_armor_dist_2d() const {
        return get_best_armor_position().head<2>().norm();
    }

    double TargetPredictor::nearest_armor_dist_3d() const {
        return get_best_armor_position().head<3>().norm();
    }

    void TargetPredictor::set_target(const rm_interfaces::msg::Target& target) {
        target_pos_ << target.position.x, target.position.y, target.position.z;

        target_v_ << target.velocity.x, target.velocity.y, target.velocity.z;

        target_yaw_ = target.yaw;

        target_v_yaw_ = target.v_yaw;

        m_r1 = target.radius_1;
        m_r2 = target.radius_2;

        m_armor_num = static_cast<size_t>(target.armors_num);

        m_d_height = target.d_height;

        m_has_target = true;
    }

    bool TargetPredictor::has_target() const {
        return m_has_target;
    }

    Eigen::Vector3d TargetPredictor::get_best_armor_position() const noexcept
    {
        int i = get_armor_id();

        double r = 0;
        double target_dz = 0.;
        double temp_yaw = target_yaw_ + i * (2 * PI / m_armor_num);
        if (m_armor_num == 4)
        {
            bool is_current_pair = i % 2 == 0;
            r = is_current_pair ? m_r1 : m_r2;
            target_dz = is_current_pair ? 0 : m_d_height;
        }
        else
        {
            r = m_r1;
            target_dz = 0;
        }

        return target_pos_ + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
    }

    int TargetPredictor::get_armor_id() const noexcept
    {
        auto armor_positions = std::vector<Eigen::Vector3d>(m_armor_num, Eigen::Vector3d::Zero());
        auto armors_vec = std::vector<Eigen::Vector2d>(m_armor_num, Eigen::Vector2d::Zero());
        // Calculate the position of each armor
        bool is_current_pair = true;
        double r = 0., target_dz = 0.;
        Eigen::Vector3d target_center(target_pos_.x(), target_pos_.y(),target_pos_.z());
        for (size_t i = 0; i < m_armor_num; i++)
        {
            double temp_yaw = target_yaw_ + i * (2 * PI / m_armor_num);
            if (m_armor_num == 4)
            {
                r = is_current_pair ? m_r1 : m_r2;
                target_dz = is_current_pair ? 0 : m_d_height;
                is_current_pair = !is_current_pair;
            }
            else
            {
                r = m_r1;
                target_dz = 0;
            }
            armor_positions[i] =
                target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
            Eigen::Vector2d armor_vec;
            armor_vec << armor_positions[i](0) - target_center(0), armor_positions[i](1) - target_center(1);
            armors_vec[i] = armor_vec;
        }

        Eigen::Vector2d center_vec;
        center_vec << -target_center(0), -target_center(1);
        double maxcosTheta = -2.0;
        double argmax = 0;
        for (int i = 0; i < m_armor_num; i++)
        {
            double cosTheta = center_vec.dot(armors_vec[i]) / center_vec.norm() / armors_vec[i].norm();
            if (cosTheta > maxcosTheta)
            {
                argmax = i;
                maxcosTheta = cosTheta;
            }
        }

        // cosTheta_ = maxcosTheta;
        return argmax;
    }
}
