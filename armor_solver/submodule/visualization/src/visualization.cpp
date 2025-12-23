//
// Created by lbw on 25-3-16.
//

#include "visualization/visualization.h"
#include <parameter/parameter.h>
#include <clock/clock.h>

namespace ckyf::visualization
{
    Preset::Preset Visualization::backendRobotPreset = Preset::getPreset_1("odom");

    Visualization::Visualization(const rclcpp::NodeOptions& options)
        : Node("visualization", options)
    {
        std::thread init([this]()
        {
            do
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            while (global_node::Parameter == nullptr);
            global_node::Parameter->get_parameter("debug", turn_on_);
            global_node::Parameter->get_parameter("object_id", id_vec_);
            if (!turn_on_)
                return;

            panel_publisher = image_transport::create_publisher(this, "armor_solver/panel");
            debug_back_publisher_ = this->create_publisher<rm_interfaces::msg::DebugBack>("armor_solver/debug", 10);
            debug_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "armor_solver/markers", 10);
            debug_frontend_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "armor_solver/measure", 10);
            panel_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                [this]()
                {
                    publishPanel();
                    publishMarkers();
                });
        });
        init.detach();
    }

    Visualization::~Visualization()
    {
    }

    void Visualization::init()
    {
    }

    void Visualization::printAllDebug(const std::string& file, int path)
    {
        std::string msg = "[" + file + "] line:" + std::to_string(path);
        if (std::find(had_called_position.begin(), had_called_position.end(), msg) == had_called_position.end())
        {
            had_called_position.insert(msg);
        }
        else
            return;
        std::cout << "Before " << msg << ",\nthese positions below had called publish debug information:" << std::endl;
        for (const auto& position : debug_call_positions_)
        {
            std::cout << position << std::endl;
        }
        std::cout << "----------------------------------------------------" << std::endl;
    }

    void Visualization::debug_kalman_object_state(const std::string& id, KalmanCommon::Track_State state)
    {
        if (is_id_valid(id))
        {
            kalman_track_state_[id] = state;
        }
    }

    void Visualization::debug_kalman_motion_state(const std::string& id, const std::string& state)
    {
        kalman_motion_map_[id] = state;
    }

    void Visualization::debug_target(const std::string& id)
    {
        target_id_ = id;
    }

    void Visualization::debug_operator(Operator& op)
    {
        operator_ = op;
    }

    void Visualization::debug_strategist(const std::string& mode)
    {
        strategist_mode_ = mode;
    }

    void Visualization::debug_tracking_mode(const std::string& track)
    {
        tracking_mode_ = track;
    }

    void Visualization::debug_offset(const Offset& offset)
    {
        offset_ = offset;
    }

    void Visualization::debug_target_marker(Target target)
    {
        shoot_target_ = target;
    }

    void Visualization::debug_measure_center_marker(CenterMeasure& center)
    {
        center_measure_ = center;
    }

    void Visualization::debug_predicted_target_marker(Target target)
    {
        predicted_target_ = target;
    }

    void Visualization::debug_shoot_marker(double pitch, double yaw)
    {
        shoot_pitch_ = pitch / 180 * M_PI;
        shoot_yaw_ = yaw / 180 * M_PI;
    }


    void Visualization::publishPanel()
    {
        cv::Mat panel = cv::Mat::zeros(1440, 1920, CV_8UC3);
        std_msgs::msg::Header header;
        header.stamp = global_node::Clock->time();
        header.frame_id = "debug";
        //时间
        std::string time = fmt::format(
            "{}.{}", header.stamp.sec, header.stamp.nanosec);
        cv::putText(panel, time, cv::Point(10, 30),
                    cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(255, 255, 255), 1);

        //kalman池
        cv::putText(panel, "Kalman Pool:", cv::Point(100, 100),
                    cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(150, 150, 0), 2);
        int i = 1;
        for (const auto& [id,state] : kalman_track_state_)
        {
            std::string trackState = KalmanCommon::trackState_to_str(state);
            std::string info = fmt::format("{:>8}:  {:>15} | {}", id, trackState, kalman_motion_map_[id]);
            cv::putText(panel, info, cv::Point(0, 100 + 100 * i),
                        cv::FONT_HERSHEY_COMPLEX, 1.6,
                        state >= 3
                            ? cv::Scalar(0, 250, 150)
                            : cv::Scalar(0, 150, 150),
                        2);

            i++;
        }

        std::string target_info =
            fmt::format("Target->{}", target_id_);
        cv::putText(panel, target_info, cv::Point(100, 1000),
                    cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 200, 155), 2);

        std::string operator_lock = fmt::format(
            "Operator Lock:{}", operator_.right_press ? "ON" : "OFF");
        cv::putText(panel, operator_lock, cv::Point(800, 1000),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    operator_.right_press
                        ? cv::Scalar(0, 250, 150)
                        : cv::Scalar(250, 0, 150),
                    2);

        //策略debug
        std::string strategist_mode = fmt::format("Strategist Mode:{}", strategist_mode_);
        cv::putText(panel, strategist_mode, cv::Point(100, 1100),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    cv::Scalar(0, 250, 150), 2);

        //跟踪模式
        std::string tracking_mode = fmt::format("Tracking Mode:{}", tracking_mode_);
        cv::putText(panel, tracking_mode, cv::Point(100, 1200),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    cv::Scalar(100, 200, 0), 2);

        //偏置debug
        std::string offset_header = "Solver Offset:";
        cv::putText(panel, offset_header, cv::Point(1200, 200),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    cv::Scalar(0, 100, 250), 2);
        std::string distance_threshold = fmt::format("distance:{}", offset_.distance);
        cv::putText(panel, distance_threshold, cv::Point(1200, 300),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    cv::Scalar(0, 100, 250), 2);
        std::string height_threshold = fmt::format("height:{}", offset_.height);
        cv::putText(panel, height_threshold, cv::Point(1200, 400),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    cv::Scalar(0, 100, 250), 2);
        std::string offset_threshold_header = "pitch/yaw(degree)";
        cv::putText(panel, offset_threshold_header, cv::Point(1200, 500),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    cv::Scalar(0, 100, 250), 2);
        std::string offset_threshold = fmt::format("{}/{}", offset_.pitch_offset, offset_.yaw_offset);
        cv::putText(panel, offset_threshold, cv::Point(1200, 600),
                    cv::FONT_HERSHEY_COMPLEX, 1.2,
                    cv::Scalar(0, 100, 250), 2);

        panel_publisher.publish(cv_bridge::CvImage(header, "rgb8", panel).toImageMsg());

        debug_back_publisher_->publish(debug_user);
    }

    void Visualization::publishMarkers()
    {
        MarkerArray backend_robot_markers;
        MarkerArray measure_markers;
        //机器人中心
        auto center = backendRobotPreset.center;
        center.pose.position.x = shoot_target_.position.x;
        center.pose.position.y = shoot_target_.position.y;
        center.pose.position.z = shoot_target_.position.z;
        backend_robot_markers.markers.push_back(center);
        if (center_measure_.isDetect)
        {
            auto center_measure_marker = backendRobotPreset.center;
            center_measure_marker.color.b = 0.0;
            center_measure_marker.color.r = 1.0;
            center_measure_marker.color.g = 0.0;
            center_measure_marker.color.a = 1.0;
            center_measure_marker.pose.position.x = center_measure_.x;
            center_measure_marker.pose.position.y = center_measure_.y;
            measure_markers.markers.push_back(center_measure_marker);
        }
        else
        {
            auto center_measure_marker = backendRobotPreset.center;
            center_measure_marker.action = Marker::DELETE;
            measure_markers.markers.push_back(center_measure_marker);
        }

        const auto& center_position = center.pose.position;
        //机器人线速度
        auto velocity = backendRobotPreset.velocity;
        velocity.scale.x = sqrt(shoot_target_.velocity.x * shoot_target_.velocity.x +
            shoot_target_.velocity.y * shoot_target_.velocity.y);
        Eigen::Vector3d dir_velocity;
        dir_velocity << -shoot_target_.velocity.x, shoot_target_.velocity.y, 0;
        auto q_velocity = direction_to_quaternion(dir_velocity);
        velocity.pose.position = center_position;
        velocity.pose.orientation.x = q_velocity.getX();
        velocity.pose.orientation.y = q_velocity.getY();
        velocity.pose.orientation.z = q_velocity.getZ();
        velocity.pose.orientation.w = q_velocity.getW();
        backend_robot_markers.markers.push_back(velocity);

        double r = shoot_target_.radius_1;
        for (auto i = 0; i < shoot_target_.armors_num; i++)
        {
            auto armor = backendRobotPreset.armor;
            armor.ns = std::string("armor") + std::to_string(i + 1);
            armor.scale.y = 0.135;
            armor.pose.position = center_position;
            double temp_yaw = shoot_target_.yaw + 2 * i * M_PI / shoot_target_.armors_num;
            if (i % 2 == 0)
                r = shoot_target_.radius_1;
            else
                r = shoot_target_.radius_2;
            armor.pose.position.x += r * cos(temp_yaw);
            armor.pose.position.y += r * sin(temp_yaw);
            tf2::Quaternion q_armor;
            q_armor.setRPY(0, -15 * M_PI / 180.0, temp_yaw);
            armor.pose.orientation.x = q_armor.getX();
            armor.pose.orientation.y = q_armor.getY();
            armor.pose.orientation.z = q_armor.getZ();
            armor.pose.orientation.w = q_armor.getW();
            backend_robot_markers.markers.push_back(armor);
        }

        auto v_yaw = backendRobotPreset.angle_w;
        v_yaw.pose.position = center_position;
        v_yaw.scale.x = shoot_target_.v_yaw / 5.0;
        tf2::Quaternion q_v_yaw;
        q_v_yaw.setRPY(0, shoot_target_.v_yaw > 0 ? M_PI / 2.0 : -M_PI / 2.0, 0);
        v_yaw.pose.orientation.x = q_v_yaw.getX();
        v_yaw.pose.orientation.y = q_v_yaw.getY();
        v_yaw.pose.orientation.z = q_v_yaw.getZ();
        v_yaw.pose.orientation.w = q_v_yaw.getW();
        backend_robot_markers.markers.push_back(v_yaw);

        auto trajectory = backendRobotPreset.trajectory;
        tf2::Quaternion q_trajectory;
        q_trajectory.setRPY(0, shoot_pitch_, shoot_yaw_);
        trajectory.pose.orientation.x = q_trajectory.getX();
        trajectory.pose.orientation.y = q_trajectory.getY();
        trajectory.pose.orientation.z = q_trajectory.getZ();
        trajectory.pose.orientation.w = q_trajectory.getW();
        backend_robot_markers.markers.push_back(trajectory);


        debug_markers_publisher_->publish(backend_robot_markers);
        debug_frontend_publisher_->publish(measure_markers);
    }


    bool Visualization::is_id_valid(const std::string& robot_id)
    {
        return std::find(id_vec_.begin(), id_vec_.end(), robot_id) != id_vec_.end();
    }

    tf2::Quaternion Visualization::direction_to_quaternion(Eigen::Vector3d direction)
    {
        // 1. 归一化方向向量
        Eigen::Vector3d dir_normalized = direction.normalized();
        Eigen::Vector3d z_axis(0, 0, 1); // 基准向量：Z轴

        // 2. 计算旋转轴（叉积）和旋转角度（点积反余弦）
        Eigen::Vector3d axis = z_axis.cross(dir_normalized).normalized();
        double cos_theta = z_axis.dot(dir_normalized);
        double theta = std::acos(cos_theta);

        // 3. 构造四元数
        tf2::Quaternion q;
        tf2::Vector3 q_axis;
        q_axis.setX(axis.x());
        q_axis.setY(axis.y());
        q_axis.setZ(axis.z());
        q.setRotation(q_axis, theta); // 旋转轴 + 旋转角度
        q.normalize(); // 归一化确保单位四元数[4](@ref)
        return q;
    }
}

