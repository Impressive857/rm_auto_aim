//
// Created by lbw on 25-5-2.
//

#ifndef ARMOR_SOLVER_COMMON_H
#define ARMOR_SOLVER_COMMON_H
//ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/image.hpp>
//project
#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/measurement.hpp"
#include "rm_interfaces/msg/operator.hpp"
#include "rm_interfaces/msg/enemy_position.hpp"
#include "rm_interfaces/msg/enemy_positions.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include <rm_interfaces/srv/enemy_strategist.hpp>
#include <rm_interfaces/msg/debug_back.hpp>

namespace ckyf
{
    namespace auto_aim
    {
        constexpr double PI = 3.141592653589793;

        static constexpr double rad2degree(const double rad) {
            return rad * 180 / PI;
        }

        static constexpr double degree2rad(const double degree) {
            return degree * PI / 180;
        }

        template <typename... _Args, typename = typename std::enable_if_t<(std::is_arithmetic_v<_Args> && ...)>>
        static constexpr auto square_sum(const _Args... args) {
            return ((args * args) + ...);
        }
        template <typename... _Args, typename = typename std::enable_if_t<(std::is_arithmetic_v<_Args> && ...)>>
        static constexpr auto square_sum_sqrt(const _Args... args) {
            return std::sqrt(square_sum(args...));
        }

        enum class StrategyMode
        {
            Discretionary, //自瞄自主决定
            Obey, //听从外部建议
            Shutdown, //关闭自瞄
            Invalid, //非法情况
        };

        struct TargetAdvice
        {
            std_msgs::msg::Header header;
            bool has_target_advice;
            StrategyMode mode;
            std::string id;
        };

        inline std::string StrategyModeToStr(StrategyMode mode)
        {
            switch (mode)
            {
            case StrategyMode::Discretionary:
                return "Discretionary";
            case StrategyMode::Obey:
                return "Obey";
            case StrategyMode::Shutdown:
                return "Shutdown";
            case StrategyMode::Invalid:
                return "Invalid";
            default:
                return "Error";
            }
        }

        struct ArmorGroup
        {
            std_msgs::msg::Header header;
            std::map<std::string, rm_interfaces::msg::Armors> armor_group;
            std::map<std::string, rm_interfaces::msg::Target> temp_lost_group;
        };

        struct ShootTarget
        {
            std_msgs::msg::Header header;
            bool has_target;
            std::string id;
        };

        static double limit_rad(double angle) {
            while (angle > PI) angle -= 2 * PI;
            while (angle <= -PI) angle += 2 * PI;
            return angle;
        }
    }
}
#endif //ARMOR_SOLVER_COMMON_H
