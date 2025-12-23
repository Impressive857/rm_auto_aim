#include "armor_solver/gimbal_controller/target_predictor.hpp"

#include <iostream>

namespace ckyf::auto_aim
{
    TargetPredictor::TargetPredictor() {
        m_has_target = false;
        m_armor_num = 0;
        m_X = Eigen::VectorXd::Zero(11);
    }

    TargetPredictor::TargetPredictor(const rm_interfaces::msg::Target& target) {
        set_target(target);
    }


    void TargetPredictor::predict(double dt) {
        m_X[0] += dt * m_X[1];
        m_X[2] += dt * m_X[3];
        m_X[4] += dt * m_X[5];
        m_X[6] += dt * m_X[7];
        m_X[6] = limit_rad(m_X[6]);
    }

    std::tuple<double, double, double> TargetPredictor::target_xyz() const {
        return { m_X[0], m_X[2], m_X[4] };
    }

    std::tuple<double, double, double, double> TargetPredictor::target_xyza() const {
        return { m_X[0], m_X[2], m_X[4], m_X[6] };
    }

    double TargetPredictor::target_dist_2d() const {
        return square_sum_sqrt(m_X[0], m_X[2]);
    }

    double TargetPredictor::target_dist_3d() const {
        return square_sum_sqrt(m_X[0], m_X[2], m_X[4]);
    }

    double TargetPredictor::nearest_armor_dist_2d() const {
        auto [x, y, z] = nearest_armor_xyz();
        return square_sum_sqrt(x, y);
    }

    double TargetPredictor::nearest_armor_dist_3d() const {
        auto [x, y, z] = nearest_armor_xyz();
        return square_sum_sqrt(x, y, z);
    }

    bool TargetPredictor::set_target(const rm_interfaces::msg::Target& target) {
        try {
            double x = target.position.x;
            double y = target.position.y;
            double z = target.position.z;

            double v_x = target.velocity.x;
            double v_y = target.velocity.y;
            double v_z = target.velocity.z;

            double yaw = target.yaw;

            double v_yaw = target.v_yaw;

            double radius = target.radius_2;

            m_armor_num = static_cast<size_t>(target.armors_num);

            double l = target.radius_2 - target.radius_1;
            double h = target.d_height;

            // x vx y vy z vz a w r l h
            // a: angle
            // w: angular velocity
            // l: r2 - r1
            // h: z2 - z1
            Eigen::VectorXd X0{ {x, v_x, y, v_y, z, v_z, yaw, v_yaw, radius, l, h} };  //初始化预测量
            m_X = X0;

            m_has_target = true;
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return false;
        }

        return true;
    }

    std::tuple<double, double, double, double> TargetPredictor::cal_armor_xyza(const size_t idx) const {
        double yaw = limit_rad(m_X[6] + idx * 2 * PI / m_armor_num);
        int use_l_h = (m_armor_num == 4) && (idx == 1 || idx == 3);

        double r = (use_l_h) ? m_X[8] + m_X[9] : m_X[8];
        double x = m_X[0] - r * std::cos(yaw);
        double y = m_X[2] - r * std::sin(yaw);
        double z = (use_l_h) ? m_X[4] + m_X[10] : m_X[4];
        return { x, y, z, yaw };
    }

    // 计算出装甲板中心的坐标（考虑长短轴）
    std::tuple<double, double, double> TargetPredictor::cal_armor_xyz(const size_t idx) const {
        auto [x, y, z, yaw] = cal_armor_xyza(idx);
        return { x,y,z };
    }

    std::tuple<double, double, double, double> TargetPredictor::nearest_armor_xyza() const {
        double x, y, z, yaw;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < m_armor_num; ++i) {
            auto [temp_x, temp_y, temp_z, temp_yaw] = cal_armor_xyza(i);
            double temp_dist = square_sum_sqrt(temp_x, temp_y, temp_z);
            if (temp_dist < min_dist) {
                x = temp_x;
                y = temp_y;
                z = temp_z;
                yaw = temp_yaw;
                min_dist = temp_dist;
            }
        }
        return { x,y,z,yaw };
    }

    std::tuple<double, double, double> TargetPredictor::nearest_armor_xyz() const {
        auto [x, y, z, yaw] = nearest_armor_xyza();
        return { x,y,z };
    }

    bool TargetPredictor::has_target() const {
        return m_has_target;
    }

    double TargetPredictor::limit_rad(double angle) const {
        while (angle > PI) angle -= 2 * PI;
        while (angle <= -PI) angle += 2 * PI;
        return angle;
    }
}
