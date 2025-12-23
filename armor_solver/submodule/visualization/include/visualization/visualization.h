//
// Created by lbw on 25-3-16.
//

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include "visualization/preset.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rm_interfaces/msg/debug_back.hpp>
#include <rm_interfaces/msg/operator.hpp>
#include <clock/clock.h>
#include <cv_bridge/cv_bridge.h>
#include <armor_solver/kalman_pool/common.h>

#include <rm_interfaces/msg/target.hpp>

namespace ckyf
{
    namespace visualization
    {
        struct Offset
        {
            double distance;
            double height;
            double yaw_offset;
            double pitch_offset;
        };

        struct CenterMeasure
        {
            bool isDetect;
            double x;
            double y;
        };

        class Visualization : public rclcpp::Node
        {
        private:
            static Preset::Preset backendRobotPreset;
            using Marker = visualization_msgs::msg::Marker;
            using MarkerArray = visualization_msgs::msg::MarkerArray;
            using Target = rm_interfaces::msg::Target;
            using DebugBack = rm_interfaces::msg::DebugBack;
            using Operator = rm_interfaces::msg::Operator;

        public:
            Visualization(const rclcpp::NodeOptions& options);
            Visualization(Visualization const&) = delete;
            Visualization& operator=(Visualization const&) = delete;
            ~Visualization() override;
            void init();
            void printAllDebug(const std::string& file, int path);

            //panel数据输入
            void debug_kalman_object_state(const std::string& id, KalmanCommon::Track_State state);
            void debug_kalman_motion_state(const std::string& id, const std::string& state);
            void debug_target(const std::string& id);
            void debug_operator(Operator& op);
            void debug_strategist(const std::string& mode);
            void debug_tracking_mode(const std::string& track);
            void debug_offset(const Offset& offset);

            //marker数据输入
            void debug_target_marker(Target target);
            void debug_measure_center_marker(CenterMeasure& center);
            void debug_predicted_target_marker(Target target);
            void debug_shoot_marker(double pitch,double yaw);

            //tool
            bool is_id_valid(const std::string& robot_id);
            static tf2::Quaternion direction_to_quaternion(Eigen::Vector3d direction);

            rm_interfaces::msg::DebugBack debug_user;

        private:
            bool turn_on_ = false;
            std::vector<std::string> id_vec_;

            std::set<std::string> debug_call_positions_;
            std::set<std::string> had_called_position;

            rclcpp::TimerBase::SharedPtr panel_timer_;
            image_transport::Publisher panel_publisher;
            void publishPanel();
            void publishMarkers();
            std::map<std::string, KalmanCommon::Track_State> kalman_track_state_;
            std::map<std::string, std::string> kalman_motion_map_;
            std::string target_id_;
            double shoot_yaw_;
            double shoot_pitch_;
            Operator operator_;
            std::string strategist_mode_;
            std::string tracking_mode_;
            Offset offset_{};
            Target shoot_target_;
            Target predicted_target_;
            CenterMeasure center_measure_;

            rclcpp::Publisher<rm_interfaces::msg::DebugBack>::SharedPtr debug_back_publisher_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_publisher_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_frontend_publisher_;
        };
    }
}

#include <register_node_macro.h>
REGISTER_NODE(ckyf::visualization::Visualization)

#include <global_node_map.hpp>
NODE_NAME_DEF(ckyf::visualization::Visualization, Visualization)
#endif //VISUALIZATION_H
