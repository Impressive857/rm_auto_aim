#include "armor_solver/gimbal_controller/planner.hpp"

namespace ckyf::auto_aim {

    Planner::Planner() {
        init();
    }

    Planner::~Planner() {

    };

    bool Planner::init() {
        setup_yaw_solver();
        setup_pitch_solver();

        double Q_0, Q_1, R;
        global_node::Parameter->get_parameter("planner.plan_Q(0)", Q_0);
        global_node::Parameter->get_parameter("planner.plan_Q(1)", Q_1);
        global_node::Parameter->get_parameter("planner.plan_R", R);

        std::string compenstator_type;
        global_node::Parameter->get_parameter("solver.compensator_type", compenstator_type);
        if (nullptr == m_trajectory_compensator)
            m_trajectory_compensator = fyt::CompensatorFactory::createCompensator(compenstator_type);
        global_node::Parameter->get_parameter("solver.iteration_times", m_trajectory_compensator->iteration_times);
        global_node::Parameter->get_parameter("solver.bullet_speed", m_trajectory_compensator->velocity);
        global_node::Parameter->get_parameter("solver.gravity", m_trajectory_compensator->gravity);
        global_node::Parameter->get_parameter("solver.resistance", m_trajectory_compensator->resistance);

        m_trajectory_compensator->iteration_times = 20;
        m_trajectory_compensator->velocity = 22;
        m_trajectory_compensator->gravity = 9.8;
        m_trajectory_compensator->resistance = 0.001;

        global_node::Parameter->get_parameter("solver.controller_delay", m_delay.control_delay);
        global_node::Parameter->get_parameter("solver.prediction_delay", m_delay.prediction_delay);
        global_node::Parameter->get_parameter("solver.response_delay", m_delay.response_delay);
        global_node::Parameter->get_parameter("solver.trigger_delay", m_delay.trigger_delay);
        global_node::Parameter->get_parameter("solver.command_delay", m_delay.command_delay);

        global_node::Parameter->get_parameter("solver.max_tracking_v_yaw", m_threshold.max_tracking_v_yaw);
        global_node::Parameter->get_parameter("solver.min_switching_v_yaw", m_threshold.min_switching_v_yaw);
        global_node::Parameter->get_parameter("solver.side_angle", m_threshold.side_angle);
        global_node::Parameter->get_parameter("solver.shoot_threshold", m_threshold.shooting_threshold);
        m_threshold.transfer_thresh = 5;
        global_node::Parameter->get_parameter("solver.shooting_range_width", m_threshold.shooting_range_h);
        global_node::Parameter->get_parameter("solver.shooting_range_height", m_threshold.shooting_range_w);
        global_node::Parameter->get_parameter("solver.shooting_range_large_width", m_threshold.shooting_range_large_w);
        global_node::Parameter->get_parameter("solver.shooting_range_large_height", m_threshold.shooting_range_large_h);

        if (nullptr == m_manual_compensator)
            m_manual_compensator = std::make_unique<fyt::ManualCompensator>();

        m_Q << Q_0, Q_1;
        m_R << R;

        m_track_mode = TrackMode::TRACK_ARMOR;

        m_overflow_count = 0;
    }

    void Planner::set_target(const rm_interfaces::msg::Target& target) noexcept {
        m_target_predictor.set_target(target);
    }

    void Planner::set_track_mode(TrackMode track_mode) noexcept {
        m_track_mode = track_mode;
    };

    Planner::TrackMode Planner::get_track_mode() const noexcept {
        return m_track_mode;
    };

    void Planner::setup_yaw_solver()
    {
        tiny_setup(&m_yaw_solver, m_A, m_B, m_f, m_Q.asDiagonal(), m_R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        m_yaw_solver->settings->max_iter = 10;

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -MAX_YAW_ACC);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, MAX_YAW_ACC);
        tiny_set_bound_constraints(m_yaw_solver, x_min, x_max, u_min, u_max);
    }

    void Planner::setup_pitch_solver()
    {
        tiny_setup(&m_pitch_solver, m_A, m_B, m_f, m_Q.asDiagonal(), m_R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        m_pitch_solver->settings->max_iter = 10;

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -MAX_PITCH_ACC);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, MAX_PITCH_ACC);
        tiny_set_bound_constraints(m_pitch_solver, x_min, x_max, u_min, u_max);
    }

    Planner::Plan Planner::get_plan() {
        Eigen::Vector3d& target_pos = m_target_predictor.target_pos_;
        Eigen::Vector3d& target_v = m_target_predictor.target_v_;
        double target_v_yaw = m_target_predictor.target_v_yaw_;

        double fly_time = m_trajectory_compensator->getFlyingTime(target_pos);

        // rclcpp::Time now_time = timer_();
        // rclcpp::Time img_time = target_.header.stamp;
        // rclcpp::Duration duration = now_time - img_time;
        // double delay = duration.seconds() + m_delay.prediction_delay; //算法
        double delay = m_delay.prediction_delay;
        delay += fly_time; //飞行时间

        //静止噪声抑制
        if (std::abs(target_v.x()) < 0.5 && target_v_yaw > 5.0)target_v.x() = 0;
        if (std::abs(target_v.y()) < 0.5 && target_v_yaw > 5.0)target_v.y() = 0;
        target_v.z() = 0;

        double delay_tr = delay + m_delay.response_delay;

        m_target_predictor.predict_xyz(delay_tr);
        m_target_predictor.predict_yaw(delay);

        // Choose the best armor to shoot
        m_target_predictor.predict_xyz(m_delay.command_delay);

        Eigen::Vector3d armor_position_0 = m_target_predictor.get_best_armor_position();

        auto [yaw_0, pitch_0] = cal_yaw_pitch(armor_position_0);

        // Compensate angle by angle_offset_map
        auto angle_offset = m_manual_compensator->angleHardCorrect(target_pos.head(2).norm(), target_pos.z());

        double pitch_offset = degree2rad(angle_offset[0]);
        double yaw_offset = degree2rad(angle_offset[1]);

        switch (m_track_mode) {
        case TrackMode::TRACK_ARMOR: {
            if (std::abs(target_v_yaw) > m_threshold.max_tracking_v_yaw)
            {
                m_overflow_count++;
            }
            else
            {
                m_overflow_count = std::max(0, m_overflow_count - 1);
            }

            if (m_overflow_count > m_threshold.transfer_thresh)
            {
                m_track_mode = TrackMode::TRACK_CENTER;
            }
            break;
        }
        case TrackMode::TRACK_CENTER: {
            if (std::abs(target_v_yaw) < m_threshold.min_switching_v_yaw)
            {
                m_overflow_count++;
            }
            else
            {
                m_overflow_count = std::max(0, m_overflow_count - 1);
            }

            if (m_overflow_count > m_threshold.transfer_thresh)
            {
                m_track_mode = TrackMode::TRACK_ARMOR;
                m_overflow_count = 0;
            }

            double temp_yaw, temp_pitch;

            //计算yaw 不添加多余delay
            std::tie(yaw_0, temp_pitch) = cal_yaw_pitch(target_pos);

            //计算pitch 添加controller_delay
            m_target_predictor.predict(m_delay.control_delay);
            Eigen::Vector3d cmd_armor_position = m_target_predictor.get_best_armor_position();
            std::tie(temp_yaw, pitch_0) = cal_yaw_pitch(cmd_armor_position);
            break;
        }
        }

        Trajectory traj = get_trajectory(yaw_0);

        Eigen::VectorXd x0(2);
        x0 << traj(0, 0), traj(1, 0);
        tiny_set_x0(m_yaw_solver, x0);

        m_yaw_solver->work->Xref = traj.block(0, 0, 2, HORIZON);
        tiny_solve(m_yaw_solver);

        x0 << traj(2, 0), traj(3, 0);
        tiny_set_x0(m_pitch_solver, x0);

        m_pitch_solver->work->Xref = traj.block(2, 0, 2, HORIZON);
        tiny_solve(m_pitch_solver);

        // plan.target_yaw = limit_rad(traj(0, HALF_HORIZON) + yaw0);
        // plan.target_pitch = traj(2, HALF_HORIZON);
        double yaw = limit_rad(m_yaw_solver->work->x(0, HALF_HORIZON) + yaw_0 + yaw_offset);
        // plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
        // plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
        double pitch = m_pitch_solver->work->x(0, HALF_HORIZON) + pitch_offset;
        // plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
        // plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

        // auto shoot_offset_ = 2;
        // plan.fire =
        //     std::hypot(
        //         traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
        //         traj(2, HALF_HORIZON + shoot_offset_) -
        //         pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < (1 / frequency_);
        Plan plan{
            .yaw = rad2degree(yaw),
            .pitch = rad2degree(pitch)
        };
        return plan;
    }

    Planner::Trajectory Planner::get_trajectory(const double yaw0)
    {
        Trajectory traj;

        m_target_predictor.predict(-DT * (HALF_HORIZON + 1));
        auto armor_xyz = m_target_predictor.get_best_armor_position();

        auto [yaw_last, pitch_last] = cal_yaw_pitch(armor_xyz);

        m_target_predictor.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
        armor_xyz = m_target_predictor.get_best_armor_position();
        auto [yaw, pitch] = cal_yaw_pitch(armor_xyz);

        for (int i = 0; i < HORIZON; i++) {
            m_target_predictor.predict(DT);
            armor_xyz = m_target_predictor.get_best_armor_position();
            auto [yaw_next, pitch_next] = cal_yaw_pitch(armor_xyz);

            auto yaw_vel = limit_rad(yaw_next - yaw_last) / (2 * DT);
            auto pitch_vel = (pitch_next - pitch_last) / (2 * DT);

            traj.col(i) << limit_rad(yaw - yaw0), yaw_vel, pitch, pitch_vel;

            yaw_last = yaw;
            pitch_last = pitch;

            yaw = yaw_next;
            pitch = pitch_next;
        }

        return traj;
    }

    std::pair<double, double> Planner::cal_yaw_pitch(const Eigen::Vector3d& p) const noexcept
    {
        return { atan2(p.y(), p.x()),atan2(p.z(), p.head(2).norm()) };
    }

    std::pair<double, double> Planner::cal_yaw_pitch(const Eigen::Vector4d& p) const noexcept
    {
        return { atan2(p.y(), p.x()),atan2(p.z(), p.head(2).norm()) };
    }
}