//
// Created by lbw on 25-3-10.
//

#ifndef ARMOR_SOLVER_NODE_H
#define ARMOR_SOLVER_NODE_H
//std
#include <Eigen/Dense>
#include <map>
//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
//project
#include <armor_solver/armor_solver_common.h>
#include <rm_utils/logger/log.hpp>
#include "gimbal_controller/controller.h"
#include "armor_solver/kalman_pool/kalman_pool.h"
#include "visualization/visualization.h"
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace ckyf
{
  namespace auto_aim
  {
    class ArmorSolverNode : public rclcpp::Node
    {
    public:
      enum DetectMode
      {
        RED,
        BLUE,
      };

    private:
      using Armor = rm_interfaces::msg::Armor;
      using Armors = rm_interfaces::msg::Armors;
      using ArmorFilter = tf2_ros::MessageFilter<Armors>;
      using Target = rm_interfaces::msg::Target;
      using GimbalCmd = rm_interfaces::msg::GimbalCmd;
      using Measurement = rm_interfaces::msg::Measurement;
      using Operator = rm_interfaces::msg::Operator;
      using TransformStamped = geometry_msgs::msg::TransformStamped;
      using PoseStamped = geometry_msgs::msg::PoseStamped;
      using Marker = visualization_msgs::msg::Marker;
      using MarkerArray = visualization_msgs::msg::MarkerArray;
      using DebugBack = rm_interfaces::msg::DebugBack;
      using EnemyPosition = rm_interfaces::msg::EnemyPosition;
      using EnemyPositions = rm_interfaces::msg::EnemyPositions;
      using SetMode = rm_interfaces::srv::SetMode;
      using EnemyStrategist = rm_interfaces::srv::EnemyStrategist;

    public:
      explicit ArmorSolverNode(const rclcpp::NodeOptions& options);
      ~ArmorSolverNode() override;
      void initKalmanPool();
      void initController();
      bool ok();
      bool is_valid_id(const std::string& id) const;

    private:
      //初始化完成
      bool initKalmanPoolDone = false;
      bool initControllerDone = false;
      bool debug_ = true;
      bool detect_white = false;

      //回调组
      rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
      rclcpp::CallbackGroup::SharedPtr service_callback_group_;

      //tf树组件
      std::string target_frame_;
      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
      std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

      //主相机获取装甲板姿态
      message_filters::Subscriber<Armors> armors_sub_;
      void ArmorsCallback(const Armors::SharedPtr& msg);
      std::shared_ptr<ArmorFilter> armors_filter_;
      bool valid_armor(const rm_interfaces::msg::Armor& armor);
      double top_armor_pitch_{};

      //全向感知
      rclcpp::TimerBase::SharedPtr multi_sensor_timer;

      //后端流程
      double depth_compensation_k_{};
      double depth_compensation_b_{};
      double height_compensation_k_{};
      double height_compensation_b_{};
      void ask_for_target();
      TargetAdvice target_advice_;
      ArmorGroup armor_group_;

      std::mutex armor_group_mutex_;
      ArmorGroup build_armor_group(Armors& detect_armors, std::vector<std::string>& not_detect);
      void push_kalman(ArmorGroup& armors_groups, std::vector<std::string>& not_detect);

      static EnemyPosition armorsToEnemy(const Armors& armors);
      static EnemyPosition targetToEnemy(const Target& target);

      //预测结果发布
      rclcpp::Publisher<Target>::SharedPtr target_pub_;
      rclcpp::Publisher<Measurement>::SharedPtr measurement_pub_;
      rclcpp::Publisher<GimbalCmd>::SharedPtr gimbal_cmd_pub_;
      std::unique_ptr<Controller> controller_{nullptr};
      rclcpp::TimerBase::SharedPtr controller_timer_;
      Controller::Delay controller_delay_;
      void ControllerTimerCallback();
      std::map<std::string, Measurement> measurement_;

      //对象卡尔曼池
      DetectMode detect_mode_;
      std::vector<std::string> id_vec_;
      std::shared_ptr<KalmanPool> kalman_pool_{nullptr};
      rclcpp::Service<SetMode>::SharedPtr set_mode_service_;
      void SetModeCallback(SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res);

      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_params_handle_;
      rcl_interfaces::msg::SetParametersResult onSetParamsCallback(const std::vector<rclcpp::Parameter>& params);

      //右键锁
      rclcpp::Subscription<Operator>::SharedPtr operator_sub_;
      void OperatorCallback(Operator msg);
    };

    inline bool ArmorSolverNode::valid_armor(const rm_interfaces::msg::Armor& armor)
    {
      bool height = abs(armor.pose.position.z) > 2;//高度不满足条件
      bool number = !is_valid_id(armor.number);//id不满足条件
      bool outpost = armor.number == "outpost" && KalmanCommon::orientationToPitch(armor.pose.orientation) >
        top_armor_pitch_;//前哨站顶部装甲
      return height || number || outpost;
    }
  }
}

#include <global_node_map.hpp>
NODE_NAME_DEF(ckyf::auto_aim::ArmorSolverNode,ArmorSolver)

#endif //ARMOR_SOLVER_NODE_H
