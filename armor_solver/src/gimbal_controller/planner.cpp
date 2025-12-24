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

        global_node::Parameter->get_parameter("solver.bullet_speed", m_bullet_speed);
    }

    bool Planner::set_target(const rm_interfaces::msg::Target& target) {
        return m_target_predictor.set_target(target);
    }

    void Planner::setup_yaw_solver()
    {
        tiny_setup(&m_yaw_solver, m_A, m_B, m_f, m_Q.asDiagonal(), m_R.asDiagonal(), 1.0, 2, 1, MIN_HORIZON, 0);

        m_yaw_solver->settings->max_iter = 10;

        set_yaw_solver_horizon(MIN_HORIZON);
    }

    void Planner::setup_pitch_solver()
    {
        tiny_setup(&m_pitch_solver, m_A, m_B, m_f, m_Q.asDiagonal(), m_R.asDiagonal(), 1.0, 2, 1, MIN_HORIZON, 0);

        m_pitch_solver->settings->max_iter = 10;

        set_pitch_solver_horizon(MIN_HORIZON);
    }

    void Planner::set_solver_horizon(int horizon) {
        set_yaw_solver_horizon(horizon);
        set_pitch_solver_horizon(horizon);
    }

    void Planner::set_yaw_solver_horizon(int horizion) {
        horizion = (0 == horizion % 2) ? horizion : horizion + 1;
        horizion = (horizion > MIN_HORIZON) ? horizion : MIN_HORIZON;

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, horizion, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, horizion, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, horizion - 1, -MAX_YAW_ACC);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, horizion - 1, MAX_YAW_ACC);
        tiny_set_bound_constraints(m_yaw_solver, x_min, x_max, u_min, u_max);
    }

    void Planner::set_pitch_solver_horizon(int horizion) {
        horizion = (0 == horizion % 2) ? horizion : horizion + 1;
        horizion = (horizion > MIN_HORIZON) ? horizion : MIN_HORIZON;

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, horizion, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, horizion, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, horizion - 1, -MAX_PITCH_ACC);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, horizion - 1, MAX_PITCH_ACC);
        tiny_set_bound_constraints(m_pitch_solver, x_min, x_max, u_min, u_max);
    }

    Planner::Plan Planner::get_plan() {
        // 1. Predict fly_time
        double dist = m_target_predictor.nearest_armor_dist_2d();
        auto [target_x, target_y, target_z, target_yaw] = m_target_predictor.target_xyza();
        //FYT_INFO("armor_solver", "dist 2d : {:.2f}", std::sqrt(target_x * target_x + target_y * target_y));
        auto bullet_traj_opt = trajectory(m_bullet_speed, dist, target_z);
        if (!bullet_traj_opt.has_value()) {
            // FYT_ERROR("armor_solver", "Can't solve bullet");
            return {};
        }
        auto [bullet_pitch, bullet_fly_time] = *bullet_traj_opt;
        m_target_predictor.predict(bullet_fly_time);
        auto armor_xyz = m_target_predictor.nearest_armor_xyz();

        // 2. Get trajectory
        double yaw0 = 0;
        Trajectory traj;
        try {
            yaw0 = aim(armor_xyz).first;
            traj = get_trajectory(yaw0);
        }
        catch (const std::exception& e) {
            // FYT_ERROR("armor_solver", "Unsolvable target {:.2f}", bullet_speed);
            return {};
        }

        // 3. Solve yaw
        Eigen::VectorXd x0(2);
        x0 << traj(0, 0), traj(1, 0);
        tiny_set_x0(m_yaw_solver, x0);

        m_yaw_solver->work->Xref = traj.block(0, 0, 2, MIN_HORIZON);
        tiny_solve(m_yaw_solver);

        // 4. Solve pitch
        x0 << traj(2, 0), traj(3, 0);
        tiny_set_x0(m_pitch_solver, x0);

        m_pitch_solver->work->Xref = traj.block(2, 0, 2, MIN_HORIZON);
        tiny_solve(m_pitch_solver);

        // plan.target_yaw = limit_rad(traj(0, HALF_HORIZON) + yaw0);
        // plan.target_pitch = traj(2, HALF_HORIZON);
        double yaw = limit_rad(m_yaw_solver->work->x(0, MIN_HALF_HORIZON) + yaw0);
        // plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
        // plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
        double pitch = m_pitch_solver->work->x(0, MIN_HALF_HORIZON);
        // plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
        // plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

        auto shoot_offset_ = 2;
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

        m_target_predictor.predict(-DT * (MIN_HALF_HORIZON + 1));
        auto armor_xyz = m_target_predictor.nearest_armor_xyz();

        auto [yaw_last, pitch_last] = aim(armor_xyz);

        m_target_predictor.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
        armor_xyz = m_target_predictor.nearest_armor_xyz();
        auto [yaw, pitch] = aim(armor_xyz);

        for (int i = 0; i < MIN_HORIZON; i++) {
            m_target_predictor.predict(DT);
            armor_xyz = m_target_predictor.nearest_armor_xyz();
            auto [yaw_next, pitch_next] = aim(armor_xyz);

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

    std::optional<std::pair<double, double>> Planner::trajectory(const double v0, const double d, const double h)
    {
        auto a = G * d * d / (2 * v0 * v0);
        auto b = -d;
        auto c = a + h;
        auto delta = b * b - 4 * a * c;

        if (delta < 0) {
            std::cout << "d:" << d << "v0:" << v0 << "h:" << h << std::endl;
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

        return std::pair<double, double>{ pitch, fly_time };
    }

    std::pair<double, double> Planner::aim(const std::tuple<double, double, double>& target_xyz)
    {
        auto [x, y, z] = target_xyz;
        double dist = sqrt(x * x + y * y);

        auto azim = std::atan2(y, x);
        auto bullet_traj_opt = trajectory(m_bullet_speed, dist, z);
        if (!bullet_traj_opt.has_value()) throw std::runtime_error("Unsolvable bullet trajectory!");

        auto [bullet_pitch, bullet_fly_time] = *bullet_traj_opt;

        return { limit_rad(azim + m_yaw_offset), -bullet_pitch - m_pitch_offset };
    }
}