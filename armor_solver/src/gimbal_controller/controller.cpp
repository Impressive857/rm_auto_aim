//
// Created by lbw on 25-3-13.
//

#include <armor_solver/gimbal_controller/controller.h>
#include "visualization/visualization.h"
#include <parameter/parameter.h>
#include <clock/clock.h>

#include <data_center/data_recorder.h>
#include <spdlog/common.h>
using namespace std::chrono_literals;

namespace ckyf::auto_aim
{
    Controller::Controller(): state()
    {
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(global_node::Clock->get_clock());
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
        adviser_ = TargetAdviser::getInstance();

        init();
    }

    void Controller::init()
    {
        std::string compenstator_type;
        global_node::Parameter->get_parameter("solver.compensator_type", compenstator_type);
        if (trajectory_compensator_ == nullptr)
            trajectory_compensator_ = fyt::CompensatorFactory::createCompensator(compenstator_type);
        global_node::Parameter->get_parameter("solver.iteration_times", trajectory_compensator_->iteration_times);
        global_node::Parameter->get_parameter("solver.bullet_speed", trajectory_compensator_->velocity);
        global_node::Parameter->get_parameter("solver.gravity", trajectory_compensator_->gravity);
        global_node::Parameter->get_parameter("solver.resistance", trajectory_compensator_->resistance);
        global_node::Parameter->get_parameter("target_frame", target_frame_);
        global_node::Parameter->get_parameter("solver.side_angle", side_angle_);
        global_node::Parameter->get_parameter("solver.shoot_frequency", frequency_);
        global_node::Parameter->get_parameter("solver.wild_coefficient", wild_coefficient_);
        trajectory_compensator_->iteration_times = 20;
        trajectory_compensator_->velocity = 22;
        trajectory_compensator_->gravity = 9.8;
        trajectory_compensator_->resistance = 0.001;
        if (manual_compensator_ == nullptr)
            manual_compensator_ = std::make_unique<fyt::ManualCompensator>();
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
        setup_yaw_solver();
        setup_pitch_solver();
    }

    void Controller::setup_yaw_solver()
    {
        auto max_yaw_acc = 50;
        Eigen::MatrixXd A{{1, DT}, {0, 1}};
        Eigen::MatrixXd B{{0}, {DT}};
        Eigen::VectorXd f{{0, 0}};
        Eigen::Matrix<double, 2, 1> Q(9e6, 0.0);
        Eigen::Matrix<double, 1, 1> R(1);
        tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
        tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

        yaw_solver_->settings->max_iter = 10;
    }

    void Controller::setup_pitch_solver()
    {
        auto max_pitch_acc = 100;
        Eigen::MatrixXd A{{1, DT}, {0, 1}};
        Eigen::MatrixXd B{{0}, {DT}};
        Eigen::VectorXd f{{0, 0}};
        Eigen::Matrix<double, 2, 1> Q(9e6,0);
        Eigen::Matrix<double, 1, 1> R(1);
        tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
        tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

        pitch_solver_->settings->max_iter = 10;
    }
    void Controller::setOffset(const Offset& offset)
    {
        offset_ = offset;
        manual_compensator_->updateMapFlow(offset_.angle_str);
    }

    void Controller::transCmd()
    {
        target_id = adviser_->getShootTarget();

        global_node::Visualization->debug_target(target_id + " [Invalid]");
        if (!kalman_pool_->isExist(target_id) && target_id != NoTarget)
        {
            sigma_vx_.reset();
            sigma_vy_.reset();
            return;
        }

        if (target_id == NoTarget ||
            (kalman_pool_->getTrackState(target_id) != KalmanCommon::Detect_Tracking &&
                kalman_pool_->getTrackState(target_id) != KalmanCommon::Detect_Temp_Lost &&
                kalman_pool_->getTrackState(target_id) != KalmanCommon::Sensing_Detecting))
        {
            rm_interfaces::msg::GimbalCmd gimbal_cmd;
            gimbal_cmd.id = -1;
            gimbal_cmd.pitch = 0;
            gimbal_cmd.yaw = 0;
            sigma_vx_.reset();
            sigma_vy_.reset();
            publish_GimbalCmd_(gimbal_cmd);
            return;
        }

        target_ = kalman_pool_->predict(target_id);
        target_predictor_.set_target(target_);
        target_.header.frame_id = target_frame_;
        target_.header.stamp = global_node::Clock->time();
        //发送target和measurement
        publish_target_(target_);
        kalman_pool_->measurements[target_.id].header = target_.header;
        publish_measurement_(kalman_pool_->measurements[target_.id]);
        global_node::Visualization->debug_target(target_id + " [Shoot]");
        global_node::Visualization->debug_target_marker(target_);
        //tf变换
        try
        {
            //获取云台当前的RPY
            auto gimbal_tf =
                tf2_buffer_->lookupTransform("odom", "gimbal_link", tf2::TimePointZero);
            auto msg_q = gimbal_tf.transform.rotation;

            tf2::Quaternion tf_q;
            tf2::fromMsg(msg_q, tf_q);
            tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
            rpy_[1] = -rpy_[1];

            //target的变换
            geometry_msgs::msg::PoseStamped ps;
            ps.header = target_.header;
            ps.pose.position = target_.position;
            try
            {
                target_.position = tf2_buffer_->transform(ps, "odom").pose.position;
            }
            catch (const tf2::TransformException& ex)
            {
                FYT_ERROR("armor_solver", "Transform Error: {}", ex.what());
            }
        }
        catch (tf2::TransformException& ex)
        {
            FYT_ERROR("armor_solver", "{}", ex.what());
            return;
        }

        //大致估计弹道后的装机板位置以及yaw转动速度
        Eigen::Vector<double, 3> target_pos;
        target_pos << target_.position.x, target_.position.y, target_.position.z;
        double fly_time = trajectory_compensator_->getFlyingTime(target_pos);

        rclcpp::Time now_time = timer_();
        rclcpp::Time img_time = target_.header.stamp;
        rclcpp::Duration duration = now_time - img_time;
        double delay = duration.seconds() + delay_.prediction_delay; //算法
        delay += fly_time; //飞行时间


        vx = target_.velocity.x;
        vy = target_.velocity.y;
        vz = target_.velocity.z;
        v_yaw = target_.v_yaw;


        //计算速度样本方差（无偏）
        sigma_vx_.push(vx);
        sigma_vy_.push(vy);
        global_node::Visualization->debug_user.sigma_vx = sigma_vx_.getSigma();
        global_node::Visualization->debug_user.sigma_vy = sigma_vy_.getSigma();

        //静止噪声抑制
        if (std::abs(vx) < 0.5 && v_yaw > 5.0)vx = 0;
        if (std::abs(vy) < 0.5 && v_yaw > 5.0)vy = 0;
        vz = 0;

        double target_yaw = target_.yaw + v_yaw * delay;
        double delay_tr = delay + delay_.response_delay;


        target_pos.x() += vx * delay_tr + 0.5 * delay_tr * delay_tr * target_.acceleration.x;
        target_pos.y() += vy * delay_tr + 0.5 * delay_tr * delay_tr * target_.acceleration.y;
        target_pos.z() += vz * delay_tr + 0.5 * delay_tr * delay_tr * target_.acceleration.z;

        global_node::Visualization->debug_user.debug16 = vy * delay_tr + 0.5 * delay_tr * delay_tr * target_.
            acceleration.y;

        // real-Armor-position
        auto real_armor_position = getBestArmorPositions2(
            target_pos, target_yaw,
            target_.radius_1, target_.radius_2,
            target_.d_height,
            target_.armors_num,
            delay_.control_delay);
        double distance = real_armor_position.norm();

        // Calculate real_yaw, real_pitch
        double real_yaw, real_pitch;
        calcYawAndPitch(real_armor_position, real_yaw, real_pitch);

        // Choose the best armor to shoot
        Eigen::Vector3d v_vec(vx, vy, 0);
        double pre_offset = delay_.command_delay;

        auto cmd_armor_position = getBestArmorPositions2(
            target_pos + v_vec * pre_offset, target_yaw,
            target_.radius_1, target_.radius_2,
            target_.d_height,
            target_.armors_num,
            delay_.control_delay);

        // Calculate cmd_yaw, cmd_pitch
        double cmd_yaw, cmd_pitch;
        calcYawAndPitch(cmd_armor_position, cmd_yaw, cmd_pitch);

        // Initialize gimbal_cmd
        rm_interfaces::msg::GimbalCmd gimbal_cmd;
        gimbal_cmd.header = target_.header;
        top_freq_ = 4 * 0.5 * v_yaw / M_PI;

        //计算中心yaw
        double center_yaw, center_pitch;
        calcYawAndPitch(target_pos, center_yaw, center_pitch);


        gimbal_cmd.distance = distance;
        top_ampl_ = 2 * atan2(r, sqrt(target_pos.x() * target_pos.x() + target_pos.y() * target_pos.y()));

        // Compensate angle by angle_offset_map
        auto angle_offset = manual_compensator_->angleHardCorrect(target_pos.head(2).norm(), target_pos.z());

        double pitch_offset = angle_offset[0] * M_PI / 180;
        double yaw_offset = angle_offset[1] * M_PI / 180;

        cmd_pitch = cmd_pitch + pitch_offset;
        cmd_yaw = angles::normalize_angle(cmd_yaw + yaw_offset);

        real_pitch = real_pitch + pitch_offset;
        real_yaw = angles::normalize_angle(real_yaw + yaw_offset);

        center_pitch = center_pitch + pitch_offset;
        center_yaw = angles::normalize_angle(center_yaw + yaw_offset);
        double bounder_yaw = center_yaw + 0.5 * top_ampl_;
        jump_time_ = std::abs(bounder_yaw - rpy_[2]) * top_freq_;


        //debug
        visualization::Offset offset{};
        offset.distance = target_pos.head(2).norm();
        offset.height = target_pos.z();
        offset.pitch_offset = angle_offset[0];
        offset.yaw_offset = angle_offset[1];
        global_node::Visualization->debug_offset(offset);

        gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], real_yaw, real_pitch, distance);
        switch (state)
        {
        case TRACKING_ARMOR:
            {
                if (std::abs(v_yaw) > threshold_.max_tracking_v_yaw)
                {
                    overflow_count_++;
                }
                else
                {
                    overflow_count_ = std::max(0, overflow_count_ - 1);
                }

                if (overflow_count_ > threshold_.transfer_thresh)
                {
                    state = TRACKING_CENTER;
                }
                global_node::Visualization->debug_tracking_mode(
                    std::string("tracking_armor") + " [" +
                    std::to_string(v_yaw) + "]");
                break;
            }
        case TRACKING_CENTER:
            {
                if (std::abs(v_yaw) < threshold_.min_switching_v_yaw)
                {
                    overflow_count_++;
                }
                else
                {
                    overflow_count_ = std::max(0, overflow_count_ - 1);
                }

                if (overflow_count_ > threshold_.transfer_thresh)
                {
                    state = TRACKING_ARMOR;
                    overflow_count_ = 0;
                }

                double temp_yaw, temp_pitch;

                //计算yaw 不添加多余delay
                calcYawAndPitch(target_pos, cmd_yaw, temp_pitch);

                //计算pitch 添加controller_delay
                cmd_armor_position = getBestArmorPositions2(target_pos, target_yaw,
                    target_.radius_1, target_.radius_2,
                    target_.d_height,
                    target_.armors_num, delay_.control_delay);
                calcYawAndPitch(cmd_armor_position, temp_yaw, cmd_pitch);

                //依据角度的cos值判定
                Eigen::Vector3d target_pos_tracking_center;
                double target_yaw_tracking_center;
                target_pos_tracking_center.x() = target_pos.x() + vx * delay_.trigger_delay;
                target_pos_tracking_center.y() = target_pos.y() + vy * delay_.trigger_delay;
                target_pos_tracking_center.z() = target_pos.z() + vz * delay_.trigger_delay;
                target_yaw_tracking_center = target_yaw + v_yaw * delay_.trigger_delay;
                getBestArmorPositions2(
                    target_pos_tracking_center, target_yaw_tracking_center,
                    target_.radius_1, target_.radius_2,
                    target_.d_height,
                    target_.armors_num, delay_.trigger_delay);
                double shoot_angle_window = threshold_.shooting_threshold / (target_.radius_1 + 0.0001);
                if (cosTheta_ > cos(shoot_angle_window))
                {
                    double shoot_time_window = shoot_angle_window / v_yaw;
                    int shoot_frequency = (int)(wild_coefficient_ * shoot_time_window / (1 / frequency_));
                    gimbal_cmd.fire_advice = shoot_frequency < 1 ? 1 : shoot_frequency;
                    gimbal_cmd.fire_advice *= 10;
                    if (gimbal_cmd.fire_advice > 20)
                        gimbal_cmd.fire_advice = 20;
                }
                else
                    gimbal_cmd.fire_advice = 0;
                global_node::Visualization->debug_user.debug17 = cosTheta_;
                global_node::Visualization->debug_tracking_mode(
                    std::string("tracking_center") + " [" +
                    std::to_string(v_yaw) + "]");
                break;
            }
        }


        gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
        gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;
        auto [plan_yaw,plan_pitch] = get_plan();
        gimbal_cmd.tj_yaw = plan_yaw;
        gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
        gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;
        real_yaw = real_yaw * 180.0 / M_PI;
        real_pitch = -real_pitch * 180.0 / M_PI;
        global_node::Visualization->debug_user.real_pitch = real_pitch;
        global_node::Visualization->debug_user.real_yaw = real_yaw;
        global_node::Visualization->debug_user.position_diff = target_.position_diff;
        if (target_.id == "1")
        {
            gimbal_cmd.id = 1;
        }
        else if (target_.id == "2")
        {
            gimbal_cmd.id = 2;
        }
        else if (target_.id == "3")
        {
            gimbal_cmd.id = 3;
        }
        else if (target_.id == "4")
        {
            gimbal_cmd.id = 4;
        }
        else if (target_.id == "5")
        {
            gimbal_cmd.id = 5;
        }
        else if (target_.id == "sentry")
        {
            gimbal_cmd.id = 6;
        }
        else if (target_.id == "outpost")
        {
            gimbal_cmd.id = 7;
        }
        else if (target_.id == "base")
        {
            gimbal_cmd.id = 8;
        }
        else
        {
            gimbal_cmd.id = 0;
        }


        Eigen::Matrix2d R;
        R << std::cos(rpy_[2]), std::sin(rpy_[2]),
            -std::sin(rpy_[2]), std::cos(rpy_[2]);
        Eigen::Vector2d target_velocity_2d;
        target_velocity_2d << vx, vy;
        Eigen::Vector2d target_linear_velocity_2d;
        target_linear_velocity_2d << v_yaw * r * cos(target_yaw), v_yaw * r * sin(target_yaw);

        Eigen::Vector2d tangental_velocity_2d = R * (target_velocity_2d + target_linear_velocity_2d);
        double feed_forward_w_radius = -tangental_velocity_2d(1) / distance;
        double feed_forward_w_angle = feed_forward_w_radius * 180 / M_PI;
        gimbal_cmd.top_freq = feed_forward_w_angle; //前馈
        gimbal_cmd.shoot_freq = 20;

        //电控前馈处理
        gimbal_cmd.top_ampl = top_ampl_;
        if (
            std::abs(gimbal_cmd.yaw - last_cmd_yaw_) > 2 ||
            (gimbal_cmd.jump_time < 5 && gimbal_cmd.jump_time > 0)) //防止丢包，连续发送5帧
        {
            gimbal_cmd.jump_time += 1;
        }
        else
        {
            gimbal_cmd.jump_time = 0;
        }
        last_cmd_yaw_ = gimbal_cmd.yaw;
        publish_GimbalCmd_(gimbal_cmd);

        global_node::Visualization->debug_shoot_marker(gimbal_cmd.pitch, gimbal_cmd.yaw);
    }

    bool Controller::isOnTarget(const double cur_yaw,
                                const double cur_pitch,
                                const double target_yaw,
                                const double target_pitch,
                                const double distance) const noexcept
    {
        // Judge whether to shoot
        //double v_coff = std::abs(v_yaw) * 0.01 + 1;
        double shooting_range_yaw = std::abs(atan2(threshold_.shooting_range_w / 2, distance));
        double shooting_range_pitch = std::abs(atan2(threshold_.shooting_range_h / 2, distance));
        if (target_.id == "1" || target_.id == "base")
        {
            shooting_range_yaw = std::abs(atan2(threshold_.shooting_range_large_w / 2, distance));
            shooting_range_pitch = std::abs(atan2(threshold_.shooting_range_large_h / 2, distance));
        }

        // Limit the shooting area to 1 degree to avoid not shooting when distance is
        // too large
        shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
        shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
        if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
            std::abs(cur_pitch - target_pitch) < shooting_range_pitch)
        {
            return true;
        }
        return false;
    }

    void Controller::calcYawAndPitch(const Eigen::Vector3d& p,
                                     double& yaw,
                                     double& pitch) const noexcept
    {
        // Calculate yaw and pitch
        yaw = atan2(p.y(), p.x());
        pitch = atan2(p.z(), p.head(2).norm());
        if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch))
        {
            pitch = temp_pitch;
        }
    }

    Eigen::Vector3d Controller::getBestArmorPositions2(const Eigen::Vector3d& target_center_without_delay,
                                                       double target_yaw_without_delay,
                                                       double r1,
                                                       double r2,
                                                       double d_height,
                                                       size_t armors_num,
                                                       double controller_delay) noexcept
    {
        Eigen::Vector3d v(vx, vy, vz);
        Eigen::Vector3d a(target_.acceleration.x, target_.acceleration.y, target_.acceleration.z);
        Eigen::Vector3d target_center_with_delay = target_center_without_delay
            + v * controller_delay + 0.5 * controller_delay * controller_delay * a;
        double target_yaw_with_delay = target_yaw_without_delay + v_yaw * controller_delay;

        int i = getArmorIdWithDelay(target_center_with_delay, target_yaw_with_delay, r1, r2, d_height, armors_num);

        r = 0;
        double target_dz = 0.;
        double temp_yaw = target_yaw_without_delay + i * (2 * M_PI / armors_num);
        if (armors_num == 4)
        {
            bool is_current_pair = i % 2 == 0;
            r = is_current_pair ? r1 : r2;
            target_dz = is_current_pair ? 0 : d_height;
        }
        else
        {
            r = r1;
            target_dz = 0;
        }

        return target_center_without_delay + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
    }

    int Controller::getArmorIdWithDelay(const Eigen::Vector3d& target_center, double target_yaw, double r1, double r2,
                                        double d_height, size_t armors_num) noexcept
    {
        auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
        auto armors_vec = std::vector<Eigen::Vector2d>(armors_num, Eigen::Vector2d::Zero());
        // Calculate the position of each armor
        bool is_current_pair = true;
        double r = 0., target_dz = 0.;
        for (size_t i = 0; i < armors_num; i++)
        {
            double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
            if (armors_num == 4)
            {
                r = is_current_pair ? r1 : r2;
                target_dz = is_current_pair ? 0 : d_height;
                is_current_pair = !is_current_pair;
            }
            else
            {
                r = r1;
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
        for (int i = 0; i < armors_num; i++)
        {
            double cosTheta = center_vec.dot(armors_vec[i]) / center_vec.norm() / armors_vec[i].norm();
            if (cosTheta > maxcosTheta)
            {
                argmax = i;
                maxcosTheta = cosTheta;
            }
        }

        global_node::Visualization->debug_user.max_costheta = maxcosTheta;
        cosTheta_ = maxcosTheta;
        return argmax;
    }
    std::pair<double, double> Controller::get_plan()
    {

        // 1. Predict fly_time
        double dist = target_predictor_.nearest_armor_dist_2d();
        auto [target_x, target_y,target_z,target_yaw] = target_predictor_.target_xyza();
        //FYT_INFO("armor_solver", "dist 2d : {:.2f}", std::sqrt(target_x * target_x + target_y * target_y));
        double bullet_speed = trajectory_compensator_->velocity;
        if (bullet_speed < 10 || bullet_speed > 25) {
            bullet_speed = 22;
        }
        auto bullet_traj_opt = trajectory(bullet_speed, dist, target_z);
        if (!bullet_traj_opt.has_value()) {
            FYT_ERROR("armor_solver", "Can't solve bullet");
            return {};
        }
        auto [bullet_pitch, bullet_fly_time] = *bullet_traj_opt;
        target_predictor_.predict(bullet_fly_time);
        auto armor_xyz = target_predictor_.nearest_armor_xyz();

        // 2. Get trajectory
        double yaw0 = 0;
        Trajectory traj;
        try {
            yaw0 = aim(armor_xyz)(0);
            traj = get_trajectory(yaw0);
        }
        catch (const std::exception& e) {
            FYT_ERROR("armor_solver", "Unsolvable target {:.2f}", bullet_speed);
            return {};
        }

        // 3. Solve yaw
        Eigen::VectorXd x0(2);
        x0 << traj(0, 0), traj(1, 0);
        tiny_set_x0(yaw_solver_, x0);

        yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
        tiny_solve(yaw_solver_);

        // 4. Solve pitch
        x0 << traj(2, 0), traj(3, 0);
        tiny_set_x0(pitch_solver_, x0);

        pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
        tiny_solve(pitch_solver_);

        // plan.target_yaw = limit_rad(traj(0, HALF_HORIZON) + yaw0);
        // plan.target_pitch = traj(2, HALF_HORIZON);
        double yaw = limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
        // plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
        // plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
        double pitch = pitch_solver_->work->x(0, HALF_HORIZON);
        // plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
        // plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

        auto shoot_offset_ = 2;
        // plan.fire =
        //     std::hypot(
        //         traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
        //         traj(2, HALF_HORIZON + shoot_offset_) -
        //         pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < (1 / frequency_);
        return {yaw*180.0/M_PI, pitch*180.0/M_PI}; // 同济的yaw和我们是反的
    }
    double Controller::limit_rad(double angle)
    {
        while (angle > CV_PI) angle -= 2 * CV_PI;
        while (angle <= -CV_PI) angle += 2 * CV_PI;
        return angle;
    }
    std::optional<std::pair<double, double>> Controller::trajectory(const double v0, const double d, const double h)
    {
        constexpr double g = 9.7833;
        auto a = g * d * d / (2 * v0 * v0);
        auto b = -d;
        auto c = a + h;
        auto delta = b * b - 4 * a * c;

        if (delta < 0) {
            std::cout<<"d:"<<d<<"v0:"<<v0<<"h:"<<h<<std::endl;
            return std::nullopt;
        }

        auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
        auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
        auto pitch_1 = std::atan(tan_pitch_1);
        auto pitch_2 = std::atan(tan_pitch_2);
        auto t_1 = d / (v0 * std::cos(pitch_1));
        auto t_2 = d / (v0 * std::cos(pitch_2));

        double pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
        double fly_time = (t_1 < t_2) ? t_1 : t_2;

        return std::pair<double,double>{ pitch, fly_time };
    }

    Eigen::Matrix<double, 2, 1> Controller::aim(const std::tuple<double,double,double>& target_xyz)
    {
        double bullet_speed = trajectory_compensator_->velocity;
        auto [x,y,z] = target_xyz;
        double dist = sqrt(x * x + y * y);

        auto azim = std::atan2(y, x);
        auto bullet_traj_opt = trajectory(bullet_speed, dist, z);
        if (!bullet_traj_opt.has_value()) throw std::runtime_error("Unsolvable bullet trajectory!");

        auto [bullet_pitch, bullet_fly_time] = *bullet_traj_opt;

        return { limit_rad(azim + yaw_offset_), -bullet_pitch - pitch_offset_ };
    }


    Trajectory Controller::get_trajectory(const double yaw0)
    {
        Trajectory traj;
        
        target_predictor_.predict(-DT * (HALF_HORIZON + 1));
        auto armor_xyz = target_predictor_.nearest_armor_xyz();

        auto yaw_pitch_last = aim(armor_xyz);

        target_predictor_.predict(DT * (HALF_HORIZON + 1));  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
        armor_xyz = target_predictor_.nearest_armor_xyz();
        auto yaw_pitch = aim(armor_xyz);

        for (int i = 0; i < HORIZON; i++) {
            target_predictor_.predict(DT);
            armor_xyz = target_predictor_.nearest_armor_xyz();
            auto yaw_pitch_next = aim(armor_xyz);

            auto yaw_vel = limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
            auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

            traj.col(i) << limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

            yaw_pitch_last = yaw_pitch;
            yaw_pitch = yaw_pitch_next;
        }

        return traj;
    }
}

