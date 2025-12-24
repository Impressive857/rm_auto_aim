//
// Created by lbw on 25-5-4.
//

#include <parameter/parameter.h>

ckyf::parameter::ArmorSolverParam::ArmorSolverParam(const rclcpp::NodeOptions& options):Node("armor_solver_parameter",options)
{
    FYT_REGISTER_LOGGER("armor_solver_param","~/fyt2024-log", INFO);
    declare_params();
}

void ckyf::parameter::ArmorSolverParam::declare_params()
{
    this->declare_parameter("debug", true);
    this->declare_parameter("target_frame","odom");
    this->declare_parameter("object_id",std::vector<std::string>({
                                       "1", "2", "3", "4", "sentry", "outpost", "base"
                                   }));
    //深度补偿
    this->declare_parameter("depth_compensation.k",0.054371);
    this->declare_parameter("depth_compensation.b",-0.048);
    this->declare_parameter("height_compensation.k",0.023);
    this->declare_parameter("height_compensation.b",0.0);
    //声明机器人的后端参数
    //机器人的
    this->declare_parameter("robot.q_x",50.0);
    this->declare_parameter("robot.q_y",50.0);
    this->declare_parameter("robot.q_z",50.0);
    this->declare_parameter("robot.r_x",0.005);
    this->declare_parameter("robot.r_y",0.005);
    this->declare_parameter("robot.r_z",0.005);

    this->declare_parameter("robot.q_yaw",1.0);
    this->declare_parameter("robot.r_yaw",5.0e-4);

    this->declare_parameter("robot.q_sx",0.05);
    this->declare_parameter("robot.q_sy",0.05);
    this->declare_parameter("robot.q_sz",0.05);
    this->declare_parameter("robot.q_syaw",1.0);
    this->declare_parameter("robot.q_sr",80.0);
    this->declare_parameter("robot.q_sdz",800.0);
    this->declare_parameter("robot.rd_x",1.0);
    this->declare_parameter("robot.rd_y",1.0);
    this->declare_parameter("robot.rd_z",1.0);
    this->declare_parameter("robot.rd_r",1.0);
    this->declare_parameter("robot.rd_dz",1.0);

    this->declare_parameter("robot.q_ro",0.3);
    this->declare_parameter("robot.r_ro",0.3);

    this->declare_parameter("robot.max_match_distance", 0.4);
    this->declare_parameter("robot.max_match_yaw_diff", 0.7);

    //声明前哨站的后端参数
    this->declare_parameter("outpost.q_x", 1.0);
    this->declare_parameter("outpost.q_y", 1.0);
    this->declare_parameter("outpost.q_z", 1.0);
    this->declare_parameter("outpost.q_yaw", 0.005);
    this->declare_parameter("outpost.q_w", 0.01); //0.01
    this->declare_parameter("outpost.q_r", 1e-5);
    this->declare_parameter("outpost.r_x", 1e-5);
    this->declare_parameter("outpost.r_y", 1e-5);
    this->declare_parameter("outpost.r_z", 1e-5);
    this->declare_parameter("outpost.r_yaw", 5e-2);
    this->declare_parameter("outpost.max_match_distance", 2.2);
    this->declare_parameter("outpost.max_match_yaw_diff", 1.5);
    this->declare_parameter("outpost.top_armor_pitch", 0.2);

    //声明controller参数
    this->declare_parameter("solver.compensator_type", "ideal");
    this->declare_parameter("solver.iteration_times", 20);
    this->declare_parameter("solver.bullet_speed", 22.0);
    this->declare_parameter("solver.gravity", 9.8);
    this->declare_parameter("solver.resistance", 0.001);

    this->declare_parameter("solver.prediction_delay", 0.02);
    this->declare_parameter("solver.controller_delay", 0.02);
    this->declare_parameter("solver.response_delay", 0.02);
    this->declare_parameter("solver.trigger_delay", 0.01);
    this->declare_parameter("solver.command_delay", 0.01);
    this->declare_parameter("horizontal_offset", 0.0);
    this->declare_parameter("vertical_offset", 0.0);
    this->declare_parameter<std::vector<std::string>>("solver.angle_offset", {"-1.0 10.0 -1.0 10.0 0.0 0.0"});
    this->declare_parameter("solver.max_tracking_v_yaw", 60.0);
    this->declare_parameter("solver.min_switching_v_yaw", 50.0);
    this->declare_parameter("solver.side_angle", 20.0);
    this->declare_parameter("solver.shooting_range_width", 0.135);
    this->declare_parameter("solver.shooting_range_height", 0.135);
    this->declare_parameter("solver.shooting_range_large_width",0.225);
    this->declare_parameter("solver.shooting_range_large_height",0.135);

    this->declare_parameter("solver.shoot_threshold",0.8);//Arc
    this->declare_parameter("solver.shoot_frequency",20.0);
    this->declare_parameter("solver.wild_coefficient",3.0);

    //外部目标建议
    this->declare_parameter("adviser.drop_time",5.0);

    this->declare_parameter("robot.plan_R",1.0);
    this->declare_parameter("planner.plan_Q(0)",1e3);
    this->declare_parameter("planner.plan_Q(1)",10);
    FYT_INFO("armor_solver_param", "All parameters declared");
}


#include <register_node_macro.h>
REGISTER_NODE(ckyf::parameter::ArmorSolverParam)
