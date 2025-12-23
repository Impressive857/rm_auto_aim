//
// Created by lbw on 25-3-10.
//

#include "armor_solver/armor_solver_node.h"
#include <clock/clock.h>
#include <data_center/data_recorder.h>
#include <parameter/parameter.h>
#include <tf2_eigen/tf2_eigen.hpp>
using namespace std::chrono_literals;

ckyf::auto_aim::ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : Node("armor_solver", options)
{
    FYT_REGISTER_LOGGER("armor_solver", "~/fyt2024-log", INFO);
    FYT_INFO("armor_solver", "Starting ArmorSolverNode");

    std::thread init([this]()
    {
        do
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        while (global_node::Clock == nullptr || global_node::Parameter == nullptr);

        global_node::Parameter->get_parameter("debug", debug_);
        global_node::Parameter->get_parameter("detect_white", detect_white);
        subscription_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto subscription_options = rclcpp::SubscriptionOptions();
        subscription_options.callback_group = subscription_callback_group_;

        //初始化tf2组件
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(global_node::Clock->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        global_node::Parameter->get_parameter("target_frame", target_frame_);
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

        //主相机装甲板获取
        armors_sub_.subscribe(this, "armor_detector/armors",
                              rmw_qos_profile_sensor_data, subscription_options);
        armors_filter_ = std::make_shared<ArmorFilter>(armors_sub_,
                                                       *tf2_buffer_,
                                                       target_frame_,
                                                       10,
                                                       this->get_node_logging_interface(),
                                                       this->get_node_clock_interface(),
                                                       std::chrono::duration<int>(1));
        armors_filter_->registerCallback(&ArmorSolverNode::ArmorsCallback, this);

        controller_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                                                    std::bind(&ArmorSolverNode::ControllerTimerCallback, this)
                                                    , subscription_callback_group_);

        //发布初始化
        target_pub_ = this->create_publisher<Target>("armor_solver/target",
                                                     rclcpp::SensorDataQoS());
        measurement_pub_ = this->create_publisher<Measurement>("armor_solver/measurement",
                                                               rclcpp::SensorDataQoS());
        gimbal_cmd_pub_ = this->create_publisher<GimbalCmd>("armor_solver/cmd_gimbal",
                                                            rclcpp::SensorDataQoS());

        //设置模式服务
        set_mode_service_ = this->create_service<SetMode>("armor_solver/set_mode",
                                                          [this](SetMode::Request::SharedPtr req,
                                                                 SetMode::Response::SharedPtr res)
                                                          {
                                                              return this->SetModeCallback(req, res);
                                                          });

        set_params_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ArmorSolverNode::onSetParamsCallback, this, std::placeholders::_1));
        FYT_INFO("armor_solver", "ArmorSolverNode Started");
    });
    init.detach();
    FYT_INFO("armor_solver", "ArmorSolverNode Initial");
}

ckyf::auto_aim::ArmorSolverNode::~ArmorSolverNode() = default;

void ckyf::auto_aim::ArmorSolverNode::initKalmanPool()
{
    global_node::Parameter->get_parameter("outpost.top_armor_pitch", top_armor_pitch_);
    kalman_pool_ = std::make_shared<KalmanPool>();
    global_node::Parameter->get_parameter("object_id", id_vec_);
    for (const auto& id : id_vec_)
    {
        kalman_pool_->createObject(id == "outpost" ? KalmanCommon::Outpost : KalmanCommon::Robot, id);
        kalman_pool_->measurements[id] = Measurement();
    }

    multi_sensor_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                                 [this]()
                                                 {
                                                     auto sensor_detected = global_node::DataRecorder->getEnemies();
                                                     for (const auto& enemy : sensor_detected->enemies)
                                                     {
                                                         kalman_pool_->sensorDetect(enemy);
                                                     }
                                                 });

    initKalmanPoolDone = true;
}

void ckyf::auto_aim::ArmorSolverNode::initController()
{
    if (controller_ == nullptr)
        controller_ = std::make_unique<Controller>();
    controller_->registerGimbalCmdPub([this](GimbalCmd cmd) { gimbal_cmd_pub_->publish(cmd); });
    controller_->registerTargetPub([this](rm_interfaces::msg::Target msg) { target_pub_->publish(msg); });
    controller_->registerMeasurementPub([this](rm_interfaces::msg::Measurement msg)
    {
        if (debug_)
            measurement_pub_->publish(msg);
    });
    controller_->registerTimer([this] { return this->get_clock()->now(); });
    Controller::Offset input_offset;
    Controller::Threshold input_threshold{};
    global_node::Parameter->get_parameter("solver.controller_delay", controller_delay_.control_delay);
    global_node::Parameter->get_parameter("solver.prediction_delay", controller_delay_.prediction_delay);
    global_node::Parameter->get_parameter("solver.response_delay", controller_delay_.response_delay);
    global_node::Parameter->get_parameter("solver.trigger_delay", controller_delay_.trigger_delay);
    global_node::Parameter->get_parameter("horizontal_offset", input_offset.h);
    global_node::Parameter->get_parameter("vertical_offset", input_offset.v);
    global_node::Parameter->get_parameter<std::vector<std::string>>("solver.angle_offset", input_offset.angle_str);
    global_node::Parameter->get_parameter("solver.max_tracking_v_yaw", input_threshold.max_tracking_v_yaw);
    global_node::Parameter->get_parameter("solver.min_switching_v_yaw", input_threshold.min_switching_v_yaw);
    global_node::Parameter->get_parameter("solver.side_angle", input_threshold.side_angle);
    global_node::Parameter->get_parameter("solver.shoot_threshold", input_threshold.shooting_threshold);
    input_threshold.transfer_thresh = 5;
    global_node::Parameter->get_parameter("solver.shooting_range_width", input_threshold.shooting_range_h);
    global_node::Parameter->get_parameter("solver.shooting_range_height", input_threshold.shooting_range_w);
    controller_->setDelay(controller_delay_);
    controller_->setOffset(input_offset);
    controller_->setThreshold(input_threshold);
    controller_->linkKalmanPool(kalman_pool_);
    initControllerDone = true;
}


bool ckyf::auto_aim::ArmorSolverNode::ok()
{
    if (global_node::Parameter == nullptr)
        return false;
    if (!initKalmanPoolDone)
        initKalmanPool();
    if (!initControllerDone)
        initController();

    if (!rclcpp::ok())
    {
        FYT_FATAL("armor_solver", "ROS Service IS BROKEN DOWN");
        return false;
    }

    return true;
}

void ckyf::auto_aim::ArmorSolverNode::ArmorsCallback(const Armors::SharedPtr& armors_msg)
{
    if (!ok())
        return;
    //完成坐标转换，并且清除显然错误的装甲板
    armors_msg->armors.erase(
        std::remove_if(armors_msg->armors.begin(),
                       armors_msg->armors.end(),
                       [this,armors_msg](Armor& armor)
                       {
                           PoseStamped ps;
                           ps.header = armors_msg->header;
                           ps.pose = armor.pose;
                           try
                           {
                               auto camera_to_target = tf2_buffer_->lookupTransform(
                                   target_frame_, armors_msg->header.frame_id,
                                   armors_msg->header.stamp, rclcpp::Duration::from_seconds(0.01));

                               auto transform = tf2::transformToEigen(camera_to_target);
                               Eigen::Vector3d position_camera(armor.pose.position.x,
                                                               armor.pose.position.y,
                                                               armor.pose.position.z);

                               auto position_target = transform * position_camera;
                               armor.pose.position.x = position_target(0);
                               armor.pose.position.y = position_target(1);
                               armor.pose.position.z = position_target(2);

                               tf2::Quaternion rotation_q;
                               tf2::fromMsg(camera_to_target.transform.rotation, rotation_q);
                               tf2::Matrix3x3 rotation_matrix = tf2::Matrix3x3(rotation_q);

                               tf2::Quaternion origin_q;
                               tf2::fromMsg(armor.pose.orientation, origin_q);
                               tf2::Matrix3x3 origin_matrix = tf2::Matrix3x3(origin_q);
                               auto rotation_result = rotation_matrix * origin_matrix;
                               tf2::Quaternion target_q;
                               rotation_result.getRotation(target_q);
                               armor.pose.orientation.w = target_q.getW();
                               armor.pose.orientation.x = target_q.getX();
                               armor.pose.orientation.y = target_q.getY();
                               armor.pose.orientation.z = target_q.getZ();
                               // armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;


                               //获取云台当前的RPY
                               std::array<double, 3> rpy_;
                               auto gimbal_tf =
                                   tf2_buffer_->lookupTransform("odom", "gimbal_link", tf2::TimePointZero);
                               auto msg_q = gimbal_tf.transform.rotation;

                               tf2::Quaternion tf_q;
                               tf2::fromMsg(msg_q, tf_q);
                               tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
                               rpy_[1] = -rpy_[1]; //获取云台当前的RPY

                               double odom_yaw = KalmanCommon::orientationToYaw(armor.pose.orientation);
                               double theta = atan2(armor.pose.position.y, armor.pose.position.x);
                               double yaw = std::min(std::abs(odom_yaw - theta), 2 * M_PI - std::abs(odom_yaw - theta));
                               double depth = sqrt(
                                   armor.pose.position.x * armor.pose.position.x +
                                   armor.pose.position.y * armor.pose.position.y);
                               double k = 0.054371 * depth - 0.048;
                               // double k = 0; //TODO 涉及模拟器时需要取消深度补偿
                               armor.pose.position.x -= k * yaw * yaw * std::cos(theta);
                               armor.pose.position.y -= k * yaw * yaw * std::sin(theta);
                               armor.yaw_in_camera = yaw;
                           }
                           catch (const tf2::TransformException& ex)
                           {
                               FYT_ERROR("armor_solver", "Transform Error: {}", ex.what());
                           }
                           return valid_armor(armor);
                       }),
        armors_msg->armors.end());
    armors_msg->header.stamp = global_node::Clock->time(); /*! 使用中央时钟重设时间**/

    //1.装甲板建组
    auto not_detect = id_vec_;
    //为了保证时间戳和检测结果的已执行，只允许在这个函数中使用detect_armors_;
    armor_group_mutex_.lock();
    armor_group_ = build_armor_group(*armors_msg, not_detect);
    armor_group_mutex_.unlock();

    //2.将所有装甲板组送入kalman池
    push_kalman(armor_group_, not_detect);

    //3.询问目标建议，选车对接选车逻辑
    ask_for_target();

    //4.更新观测状态
    TargetAdviser::getInstance()->putDetectArmorGroup(armor_group_);
}

void ckyf::auto_aim::ArmorSolverNode::ControllerTimerCallback()
{
    //发送控制指令
    if (controller_ != nullptr && kalman_pool_ != nullptr)
    {
        controller_->transCmd();
    }
}

void ckyf::auto_aim::ArmorSolverNode::ask_for_target()
{
    //1.构造主相机检测的敌人信息
    for (auto& [id,armors] : armor_group_.armor_group)
    {
        //构造敌人状态，送入选车
        auto main_detect_enemy = armorsToEnemy(armors);
        global_node::DataRecorder->mainDetectEnemy(main_detect_enemy);
    }

    //2.temp_lost 也认为是已经观测到
    for (auto& [id, temp_lost_target] : armor_group_.temp_lost_group)
    {
        auto temp_lost_enemy = targetToEnemy(temp_lost_target);
        global_node::DataRecorder->mainDetectEnemy(temp_lost_enemy);
    }

    //3.获取外部攻击目标
    if (global_node::DataRecorder->serviceOn)
    {
        auto fut = global_node::DataRecorder->ask_for_strategy();

        auto fut_status = fut.wait_for(std::chrono::milliseconds(5));
        if (fut_status == std::future_status::ready)
        {
            auto response = fut.get();
            target_advice_.has_target_advice = response->err_code == 1; //0听自瞄，1强行选定，2关闭
            switch (response->err_code)
            {
            case 0:
                target_advice_.mode = StrategyMode::Discretionary;
                break;
            case 1:
                target_advice_.mode = StrategyMode::Obey;
                break;
            case 2:
                target_advice_.mode = StrategyMode::Shutdown;
                break;
            default:
                target_advice_.mode = StrategyMode::Invalid;
            }
            target_advice_.id = response->id;
            target_advice_.header.stamp = global_node::Clock->time();
            TargetAdviser::getInstance()->putExternAdvice(target_advice_);
        }
        else
        {
            target_advice_.mode = StrategyMode::Invalid;
            target_advice_.has_target_advice = false;
            target_advice_.header.stamp = global_node::Clock->time();
        }
        global_node::Visualization->debug_strategist(StrategyModeToStr(target_advice_.mode));
    }
    else
    {
        global_node::Visualization->debug_strategist("NoService");
    }
}

ckyf::auto_aim::ArmorGroup ckyf::auto_aim::ArmorSolverNode::build_armor_group(
    Armors& detect_armors, std::vector<std::string>& not_detect)
{
    //将相同id的装甲板建组
    std::map<std::string, rm_interfaces::msg::Armors> armors_groups;
    for (auto& detected_armor : detect_armors.armors)
    {
        not_detect.erase(
            std::remove_if(not_detect.begin(),
                           not_detect.end(),
                           [detected_armor](const std::string& id)
                           {
                               return id == detected_armor.number;
                           }),
            not_detect.end());
        //建组
        auto& armors_group = armors_groups[detected_armor.number];
        armors_group.header = detect_armors.header;
        armors_group.armors.push_back(detected_armor);
    }
    //主相机识别到的数据更新到recorder
    for (const auto& [id,armor_group] : armors_groups)
    {
        auto enemy = armorsToEnemy(armor_group);
        enemy.main = true;
        enemy.is_detected = true;
        enemy.header = armor_group.header;
        enemy.occlusion = false;
        global_node::DataRecorder->mainDetectEnemy(enemy);
    }
    return {detect_armors.header, armors_groups};
}

void ckyf::auto_aim::ArmorSolverNode::push_kalman(ArmorGroup& armors_groups, std::vector<std::string>& not_detect)
{
    //1.将主相机识别到的装甲板组送入卡尔曼池
    for (auto& armor_pair : armors_groups.armor_group)
    {
        auto& id = armor_pair.first;
        auto& detected_armors = armor_pair.second;
        if (detected_armors.armors.empty() || !is_valid_id(id))
            continue;
        const auto& detected_armor = detected_armors.armors.front();
        //筛选没有检测到的装甲板
        kalman_pool_->changeState(detected_armor.number, KalmanCommon::ExternState::Main_Detect);

        //当前装甲板已经被主相机检测到了，但是分为已经追踪和没有追踪两种情况，这一点在measure->update中会有体现
        auto measurement_msg = kalman_pool_->measure(id, detected_armors);
        auto& measurement = kalman_pool_->measurements[detected_armor.number];
        measurement = measurement_msg;
    }

    //2.丢失目标更新状态
    for (auto& armor_id : not_detect)
    {
        kalman_pool_->changeState(armor_id, KalmanCommon::ExternState::Main_Lost);
    }

    //3.TempLost状态也视为检测到
    for (const auto& id : not_detect)
    {
        if (is_valid_id(id))
        {
            auto track_state = kalman_pool_->getTrackState(id);
            if (track_state == KalmanCommon::Detect_Temp_Lost)
            {
                armors_groups.temp_lost_group[id] = kalman_pool_->getState(id);
            }
        }
    }

    //4.Debug
    if (debug_)
    {
        kalman_pool_->pushDebug();
    }
}

void ckyf::auto_aim::ArmorSolverNode::SetModeCallback(SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res)
{
    switch (req->mode)
    {
    case 0: /*!自瞄红*/
    case 2: /*!小符红*/
    case 4: /*!大符红*/
        detect_mode_ = RED;
        res->success = true;
        break;
    case 1: /*!自瞄蓝*/
    case 3: /*!小符蓝*/
    case 5: /*!大符蓝*/
        detect_mode_ = BLUE;
        res->success = true;
        break;
    default:
        res->success = false;
        res->message = std::string("[ERROR]Unknown DetectMode");
        break;
    }
}

rcl_interfaces::msg::SetParametersResult ckyf::auto_aim::ArmorSolverNode::onSetParamsCallback(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param : params)
    {
        if (param.get_name() == "solver.prediction_delay")
        {
            controller_delay_.prediction_delay = param.as_double();
            controller_->setDelay(controller_delay_);
        }
        else if (param.get_name() == "solver.controller_delay")
        {
            controller_delay_.control_delay = param.as_double();
            controller_->setDelay(controller_delay_);
        }
        else if (param.get_name() == "solver.response_delay")
        {
            controller_delay_.response_delay = param.as_double();
            controller_->setDelay(controller_delay_);
        }
        else if (param.get_name() == "solver.trigger_delay")
        {
            controller_delay_.trigger_delay = param.as_double();
            controller_->setDelay(controller_delay_);
        }
    }

    return result; // 返回校验结果
}

bool ckyf::auto_aim::ArmorSolverNode::is_valid_id(const std::string& id) const
{
    return std::find(id_vec_.begin(), id_vec_.end(), id) != id_vec_.end();
}

ckyf::auto_aim::ArmorSolverNode::EnemyPosition ckyf::auto_aim::ArmorSolverNode::armorsToEnemy(const Armors& armors)
{
    EnemyPosition enemy;
    enemy.main = true;
    enemy.header = armors.header;
    enemy.is_detected = true;

    if (armors.armors.size() == 1)
    {
        enemy.pose = armors.armors.at(0).pose;
    }
    else if (armors.armors.size() == 2)
    {
        //根据两块装甲板，求解其旋转中心的坐标
        auto& armor1 = armors.armors.at(0);
        auto& armor2 = armors.armors.at(1);
        double yaw1 = KalmanCommon::orientationToYaw(armor1.pose.orientation);
        double yaw2 = KalmanCommon::orientationToYaw(armor2.pose.orientation);
        double yaw = std::min(yaw1, yaw2);
        //魔法公式，其中有深刻的数学哲理
        Eigen::Matrix2d mat1;
        Eigen::Vector2d vec;
        mat1 << cos(yaw), -sin(yaw),
            sin(yaw), cos(yaw);
        vec << armor1.pose.position.x * cos(yaw) + armor1.pose.position.y * sin(yaw),
            armor2.pose.position.y * cos(yaw) - armor2.pose.position.x * sin(yaw);

        auto answer = mat1 * vec;
        enemy.pose.position.x = answer(0);
        enemy.pose.position.y = answer(1);
        enemy.pose.position.z = (armor1.pose.position.z + armor2.pose.position.z) / 2.;
    }
    return enemy;
}

ckyf::auto_aim::ArmorSolverNode::EnemyPosition ckyf::auto_aim::ArmorSolverNode::targetToEnemy(const Target& target)
{
    EnemyPosition enemy;
    enemy.main = true;
    enemy.header = target.header;
    enemy.header.stamp = global_node::Clock->time();
    enemy.is_detected = true;
    enemy.id = target.id;
    rclcpp::Time now = global_node::Clock->now();
    rclcpp::Time last = target.header.stamp;
    double dt = (now - last).seconds();
    enemy.pose.position.x = target.position.x + target.velocity.x * dt + 0.5 * target.acceleration.x * dt * dt;
    enemy.pose.position.y = target.position.y + target.velocity.y * dt + 0.5 * target.acceleration.y * dt * dt;
    enemy.pose.position.z = target.position.z + target.velocity.z * dt + 0.5 * target.acceleration.z * dt * dt;
    double yaw = target.yaw + target.v_yaw * dt;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    enemy.pose.orientation.x = quat.x();
    enemy.pose.orientation.y = quat.y();
    enemy.pose.orientation.z = quat.z();
    enemy.pose.orientation.w = quat.w();
    return enemy;
}

#include <register_node_macro.h>
REGISTER_NODE(ckyf::auto_aim::ArmorSolverNode)
