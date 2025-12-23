#ifndef _TARGET_PREDICTOR_HPP_
#define _TARGET_PREDICTOR_HPP_

// std
#include <tuple>
#include <numeric>

// ros
#include <rm_interfaces/msg/target.hpp>

// Eigen
#include <Eigen/Dense>

namespace ckyf
{
    namespace auto_aim
    {
        template <typename... _Args, typename = typename std::enable_if_t<(std::is_arithmetic_v<_Args> && ...)>>
        auto square_sum(const _Args... args) {
            return ((args * args) + ...);
        }
        template <typename... _Args, typename = typename std::enable_if_t<(std::is_arithmetic_v<_Args> && ...)>>
        auto square_sum_sqrt(const _Args... args) {
            return std::sqrt(square_sum(args...));
        }
        class TargetPredictor {
        public:
            TargetPredictor();
            TargetPredictor(const rm_interfaces::msg::Target& target);
            void predict(double dt);
            double target_dist_2d() const;
            double target_dist_3d() const;
            double nearest_armor_dist_2d() const;
            double nearest_armor_dist_3d() const;
            std::tuple<double, double, double> target_xyz() const;
            std::tuple<double, double, double, double> target_xyza() const;
            std::tuple<double, double, double> nearest_armor_xyz() const;
            std::tuple<double, double, double, double> nearest_armor_xyza() const;
            bool set_target(const rm_interfaces::msg::Target& target);
            bool has_target() const;
            ~TargetPredictor() = default;
        private:
            double limit_rad(double angle) const;
            std::tuple<double, double, double> cal_armor_xyz(const size_t idx) const;
            std::tuple<double, double, double, double> cal_armor_xyza(const size_t idx) const;
        private:
            size_t m_armor_num;
            bool m_has_target;
            Eigen::VectorXd m_X;
            const double PI = 3.1415926;
        };
    }
}

#endif // ! _TARGET_PREDICTOR_HPP_