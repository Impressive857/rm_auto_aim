//
// Created by lbw on 25-3-13.
//

#ifndef OUTPOST_H
#define OUTPOST_H
#include <armor_solver/kalman_pool/observer.hpp>
#include "armor_solver/kalman_pool/common.h"
#include "armor_solver/kalman_pool/object_base.h"

namespace Object
{
    class Outpost : public ObjectBase
    {
    public:
        explicit Outpost(const std::string& ID);
        ~Outpost() override;

        void init() override;

        rm_interfaces::msg::Measurement update(rm_interfaces::msg::Armors& same_num_armors) override;
        rm_interfaces::msg::Target getState() override;
        rm_interfaces::msg::Target predict() override;

        void reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor& armor);
        void resetFromSensor(const geometry_msgs::msg::Pose& pose) override;

        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);
        double orientationToYaw(const geometry_msgs::msg::Quaternion& q);
        void handleArmorYawJump(const rm_interfaces::msg::Armor& current_armor);
        void handleArmorPositionJump(const rm_interfaces::msg::Armor& current_armor);

    private:
        double dt_{};
        std::unique_ptr<Observer> observer_;

        Eigen::VectorXd measurement;
        Eigen::MatrixXd target_state;
        double max_match_distance_{};
        double max_match_yaw_diff_{};

        void updateTranslate(rm_interfaces::msg::Armor& tracked_armor,
                             Eigen::MatrixXd position_only_predict_yaw,
                             rm_interfaces::msg::Measurement& measurement_msg);

    public:
        double last_yaw_ = 0;
    };
}

#endif //OUTPOST
