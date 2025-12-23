//
// Created by lbw on 25-3-13.
//

#include "armor_solver/kalman_pool/outpost.h"

#include <angles/angles.h>
#include <visualization/visualization.h>

#include "parameter/parameter.h"

#define Outpost_Radius 0.276
#define Outpost_Speed 0.8 * M_PI

Object::Outpost::Outpost(const std::string& ID): ObjectBase(ID)
{
  dt_ = 0.05;
  Outpost::init();
}

Object::Outpost::~Outpost()
{
}

void Object::Outpost::init()
{
  observer_ = std::make_unique<Observer>([this]() { return dt_; });
  Observer::Noise noise;
  global_node::Parameter->get_parameter("outpost.q_x", noise.q_x);
  global_node::Parameter->get_parameter("outpost.q_y", noise.q_y);
  global_node::Parameter->get_parameter("outpost.q_z", noise.q_z);
  global_node::Parameter->get_parameter("outpost.r_x", noise.r_x);
  global_node::Parameter->get_parameter("outpost.r_y", noise.r_y);
  global_node::Parameter->get_parameter("outpost.r_z", noise.r_z);

  global_node::Parameter->get_parameter("outpost.q_yaw", noise.q_yaw);
  global_node::Parameter->get_parameter("outpost.r_yaw", noise.r_yaw);

  global_node::Parameter->get_parameter("outpost.max_match_distance", max_match_distance_);
  global_node::Parameter->get_parameter("outpost.max_match_yaw_diff", max_match_yaw_diff_);
  observer_->setNoise(noise);
  measurement = Eigen::Vector4d(0, 0, 0, 0);
}

rm_interfaces::msg::Measurement Object::Outpost::update(rm_interfaces::msg::Armors& same_num_armors)
{
  rm_interfaces::msg::Measurement measurement_msg;
  ckyf::visualization::CenterMeasure center_measure{};
  center_measure.isDetect = false;
  if (first_frame_)
  {
    std::cout << "First Frame" << std::endl;
    reset(same_num_armors.header.stamp, same_num_armors.armors[0]);
    dt_ = 0.05;
  }
  else
  {
    rclcpp::Time time = same_num_armors.header.stamp;
    dt_ = (time - last_time_).seconds();
    last_time_ = time;
  }

  auto prediction_yaw = observer_->predictYaw();
  auto prediction_all = observer_->predictState();
  target_state = prediction_all;

  auto prediction_armor_position = getArmorPositionFromState(prediction_all);

  auto min_position_diff = DBL_MAX;
  auto min_yaw_diff = DBL_MAX;

  std::function calc_diff = [&]
  (const rm_interfaces::msg::Armor& armor, double& yaw_diff, double& position_diff)mutable
  {
    auto p = armor.pose.position;
    Eigen::Vector3d position_vec(p.x, p.y, p.z);
    position_diff = (position_vec - prediction_armor_position).norm();
    yaw_diff = orientationToYaw(armor.pose.orientation) - prediction_yaw(6);
  };

  rm_interfaces::msg::Armor tracked_armor;
  double measurement_yaw;
  if (same_num_armors.armors.size() == 1)
  {
    tracked_armor = same_num_armors.armors[0];
    calc_diff(tracked_armor, min_yaw_diff, min_position_diff);
  }
  else
  {
    std::sort(same_num_armors.armors.begin(), same_num_armors.armors.end(), [&](auto& armor1, auto& armor2) mutable
    {
      double position_diff1, position_diff2;
      double yaw_diff1, yaw_diff2;
      calc_diff(armor1, yaw_diff1, position_diff1);
      calc_diff(armor2, yaw_diff2, position_diff2);
      if (std::abs(position_diff1) < std::abs(position_diff2))
      {
        min_position_diff = position_diff1;
        min_yaw_diff = yaw_diff1;
        return true;
      }
      else
      {
        min_position_diff = position_diff2;
        min_yaw_diff = yaw_diff2;
        return false;
      }
    });
  }

  tracked_armor = same_num_armors.armors[0];
  measurement_yaw = orientationToYaw(tracked_armor.pose.orientation);
  measurement_msg.yaw = measurement_yaw;
  global_node::Visualization->debug_user.yaw_diff = std::abs(min_yaw_diff);
  global_node::Visualization->debug_user.position_diff = std::abs(min_position_diff);
  global_node::Visualization->debug_user.debug20 = tracked_armor.yaw_in_camera;
  if (std::abs(min_yaw_diff) < max_match_yaw_diff_ && min_position_diff < max_match_distance_)
  {
    Eigen::Matrix<double, 1, 1> yaw_measurement;
    yaw_measurement << measurement_yaw;
    observer_->updateYaw(yaw_measurement, tracked_armor.yaw_in_camera);
    updateTranslate(tracked_armor, prediction_yaw, measurement_msg);
  }
  else
  {
    if (std::abs(min_yaw_diff) > max_match_yaw_diff_ && same_num_armors.armors.size() == 1)
    {
      FYT_INFO("armor_solver", "[Observer]Armor Yaw Jumped!");
      handleArmorYawJump(tracked_armor);
    }
    else if (min_position_diff > max_match_distance_ && std::abs(min_yaw_diff) < max_match_yaw_diff_)
    {
      FYT_WARN("armor_solver", "[Observer]Armor Position Jumped!");
      handleArmorPositionJump(tracked_armor);
    }
    else
    {
      // reset(same_num_armors.header.stamp, tracked_armor);
      FYT_ERROR("armor_solver", "[Observer]Wrong!");
      std::cout << "min_yaw:" << min_yaw_diff << " min_position:" << min_position_diff << std::endl;
      // observer_->setState(prediction_all);
    }
  }


  target_state(8) = Outpost_Radius; //半径
  target_state(3) = 0;
  target_state(4) = 0;
  target_state(5) = 0;
  observer_->setState(target_state);
  measurement_msg.x = tracked_armor.pose.position.x;
  measurement_msg.y = tracked_armor.pose.position.y;
  measurement_msg.z = tracked_armor.pose.position.z;
  return measurement_msg;
}

rm_interfaces::msg::Target Object::Outpost::getState()
{
  rm_interfaces::msg::Target target;
  target.header.frame_id = "odom";
  target.header.stamp = last_time_;
  target.position.x = target_state(0);
  target.position.y = target_state(1);
  target.position.z = target_state(2);
  target.velocity.x = target_state(3);
  target.velocity.y = target_state(4);
  target.velocity.z = target_state(5);
  target.yaw = target_state(6);
  target.v_yaw = target_state(7);
  target.radius_1 = Outpost_Radius;
  target.radius_2 = Outpost_Radius;
  target.d_zc = 0;
  target.id = id;
  target.armors_num = 3;
  target.position_diff = 0.0;
  target.d_height = 0.0;
  return target;
}

rm_interfaces::msg::Target Object::Outpost::predict()
{
  rclcpp::Time now = global_node::Clock->time();
  auto duration = now - last_time_;
  dt_ = duration.seconds();
  last_time_ = now;
  target_state = observer_->predictState();

  rm_interfaces::msg::Target target;
  target.position.x = target_state(0);
  target.velocity.x = target_state(3);
  target.acceleration.x = 0;
  target.position.y = target_state(1);
  target.velocity.y = target_state(4);
  target.acceleration.y = 0;
  target.position.z = target_state(2);
  target.velocity.z = target_state(5);
  target.acceleration.y = 0;
  target.yaw = target_state(6);
  target.v_yaw = target_state(7);
  target.radius_1 = Outpost_Radius;
  target.radius_2 = Outpost_Radius;
  target.d_zc = 0.0;
  target.armors_num = 3;
  target.position_diff = 0.0;
  target.d_za = 0.0;
  target.d_height = 0.0; //当前kalman的z+d_height ==> 下一块的装甲板height
  return target;
}

void Object::Outpost::reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor& armor)
{
  last_time_ = time;

  double xa = armor.pose.position.x;
  double ya = armor.pose.position.y;
  double za = armor.pose.position.z;
  last_yaw_ = 0;
  double yaw = KalmanCommon::orientationToYaw(armor.pose.orientation);
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;

  // Set initial position at 0.2 m behind the target
  target_state = Eigen::VectorXd::Zero(Observer::X_A);
  double r;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  target_state << xc, yc, zc, 0, 0, 0, yaw, Outpost_Speed, Outpost_Radius, 0;

  observer_->setState(target_state);
}

void Object::Outpost::resetFromSensor(const geometry_msgs::msg::Pose& pose)
{
  target_state << pose.position.x, 0, pose.position.y, 0, pose.position.z, 0,
    KalmanCommon::orientationToYaw(pose.orientation), 0, 0.20, 0.0;
}

Eigen::Vector3d Object::Outpost::getArmorPositionFromState(const Eigen::VectorXd& x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(1), za = x(2) + x(9);
  double yaw = x(6), r = Outpost_Radius;
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

double Object::Outpost::orientationToYaw(const geometry_msgs::msg::Quaternion& q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

void Object::Outpost::handleArmorYawJump(const rm_interfaces::msg::Armor& current_armor)
{
  double last_yaw = target_state(6);
  double yaw = orientationToYaw(current_armor.pose.orientation);
  // Eigen::Vector3d p(current_armor.pose.position.x, current_armor.pose.position.y, current_armor.pose.position.z);
  if (abs(yaw - last_yaw) > 0.4)
  {
    // Armor angle also jumped, take this case as target spinning
    target_state(6) = yaw;
    FYT_DEBUG("armor_solver", "[Outpost]Armor Jump!");
  }
  observer_->setState(target_state);
}

void Object::Outpost::handleArmorPositionJump(const rm_interfaces::msg::Armor& current_armor)
{
  last_time_ = global_node::Clock->time();

  double xa = current_armor.pose.position.x;
  double ya = current_armor.pose.position.y;
  double za = current_armor.pose.position.z;
  double yaw = KalmanCommon::orientationToYaw(current_armor.pose.orientation);
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;

  // Set initial position at 0.2 m behind the target
  double r = Outpost_Radius;
  target_state(0) = xa + r * cos(yaw);
  target_state(1) = ya + r * sin(yaw);
  target_state(2) = za;
  target_state(6) = yaw;
  observer_->setState(target_state);
}

void Object::Outpost::updateTranslate(rm_interfaces::msg::Armor& tracked_armor,
                                      Eigen::MatrixXd position_only_predict_yaw,
                                      rm_interfaces::msg::Measurement& measurement_msg)
{
  const auto& p = tracked_armor.pose.position;

  double r = Outpost_Radius;
  double yaw = target_state(6);

  Eigen::Vector3d measured_center = Eigen::Vector3d(p.x + r * cos(yaw), p.y + r * sin(yaw), p.z);
  observer_->predictTranslate();
  target_state = observer_->updateTranslate(measured_center, tracked_armor.yaw_in_camera);
  measurement_msg.x = p.x;
  measurement_msg.y = p.y;
  measurement_msg.z = p.z;
}
