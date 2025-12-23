//
// Created by lbw on 25-3-13.
//

#include "armor_solver/kalman_pool/outpost.h"

#include <angles/angles.h>
#include <visualization/visualization.h>
#include <rm_utils/logger/log.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <parameter/parameter.h>

Object::Outpost::Outpost(const std::string& ID): ObjectBase(ID)
{
  w_pre = 0.0;
  Outpost::init();
}

Object::Outpost::~Outpost()
{
  delete predict_;
  delete measure_;
}

void Object::Outpost::init()
{
  getParam();

  predict_ = new Predict(0.005);
  measure_ = new Measure;

  auto u_q = [this]()
  {
    Eigen::Matrix<double, X_N, X_N> q;
    double t = dt_, x = noise_.s2qx, y = noise_.s2qy, z = noise_.s2qz, yaw = noise_.s2qw, r = noise_.s2qr;
    double q_x_x = pow(t, 4) / 4 * x;
    double q_y_y = pow(t, 4) / 4 * y;
    double q_z_z = pow(t, 4) / 4 * z;
    double q_yaw_yaw = pow(t, 4) / 4 * yaw, q_yaw_vyaw = pow(t, 3) / 2 * yaw,
           q_vyaw_vyaw = pow(t, 2) * yaw;
        // clang-format off
        //    xc     yc      zc     yaw         v_yaw
        q <<  q_x_x, 0,      0,     0,          0,
              0,     q_y_y,  0,     0,          0,
              0,     0,      q_z_z, 0,          0,
              0,     0,      0,     q_yaw_yaw,  q_yaw_vyaw,
              0,     0,      0,     q_yaw_vyaw, q_vyaw_vyaw;

    // clang-format on
    return q;
  };

  auto u_r = [this](const Eigen::Matrix<double, Z_N, 1>& z)
  {
    Eigen::Matrix<double, Z_N, Z_N> r;
        // clang-format off
        r << noise_.r_x * std::abs(z[0]), 0, 0, 0,
             0, noise_.r_y * std::abs(z[1]), 0, 0,
             0, 0, noise_.r_z * std::abs(z[2]), 0,
             0, 0, 0, noise_.r_yaw;
    // clang-format on
    return r;
  };

  Eigen::DiagonalMatrix<double, X_N> p0;
  p0.setIdentity();
  ekf_ = std::make_unique<RobotStateEKF>(predict_, measure_, u_q, u_r, p0);
}

void Object::Outpost::getParam()
{
  global_node::Parameter->get_parameter("outpost.q_x", noise_.s2qx);
  global_node::Parameter->get_parameter("outpost.q_y", noise_.s2qy);
  global_node::Parameter->get_parameter("outpost.q_z", noise_.s2qz);
  global_node::Parameter->get_parameter("outpost.q_yaw", noise_.s2qyaw);
  global_node::Parameter->get_parameter("outpost.q_w", noise_.s2qw);
  global_node::Parameter->get_parameter("outpost.q_r", noise_.s2qr);
  global_node::Parameter->get_parameter("outpost.r_x", noise_.r_x);
  global_node::Parameter->get_parameter("outpost.r_y", noise_.r_y);
  global_node::Parameter->get_parameter("outpost.r_z", noise_.r_z);
  global_node::Parameter->get_parameter("outpost.r_yaw", noise_.r_yaw);
  // std::cout << noise_ << std::endl;

  global_node::Parameter->get_parameter("outpost.max_match_distance", max_match_distance_);
  global_node::Parameter->get_parameter("outpost.max_match_yaw_diff", max_match_yaw_diff_);

  global_node::Parameter->get_parameter("outpost.top_armor_pitch", top_armor_pitch_);
}

rm_interfaces::msg::Measurement Object::Outpost::update(rm_interfaces::msg::Armors& same_num_armors)
{
  rm_interfaces::msg::Measurement measurement_msg;

  if (first_frame_)
  {
    reset(same_num_armors.header.stamp, same_num_armors.armors[0]);
    dt_ = 0.05;
  }
  else
  {
    rclcpp::Time time = same_num_armors.header.stamp;
    dt_ = (time - last_time_).seconds();
    last_time_ = time;
  }
  predict_->dt = dt_;
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf_->predict();

  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;

  rm_interfaces::msg::Armor tracked_armor;

  // Find the closest armor with the same id
  auto predicted_position = getArmorPositionFromState(ekf_prediction);
  double min_position_diff = DBL_MAX;
  double yaw_diff = DBL_MAX;
  double last_yaw = last_yaw_;
  double now_yaw = 0;
  for (const auto& armor : same_num_armors.armors)
  {
    // Only consider armors with the same id
    // Calculate the difference between the predicted position and the
    // current armor position
    auto p = armor.pose.position;
    Eigen::Vector3d position_vec(p.x, p.y, p.z);
    double position_diff = (predicted_position - position_vec).norm();
    if (position_diff < min_position_diff)
    {
      // Find the closest armor
      min_position_diff = position_diff;
      now_yaw = orientationToYaw(armor.pose.orientation);
      yaw_diff = abs(now_yaw - ekf_prediction(3));
      tracked_armor = armor;
    }
  }
  //前哨站不会出现同时检测到两个装甲板的情况
  // Check if the distance and yaw difference of closest armor are within the
  // threshold
  if (yaw_diff < max_match_yaw_diff_ && min_position_diff < max_match_distance_)
  {
    // Matched armor found
    auto p = tracked_armor.pose.position;
    // Update EKF
    double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
    measurement_msg.x = p.x;
    measurement_msg.y = p.y;
    measurement_msg.z = p.z;
    measurement_msg.yaw = measured_yaw;
    measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
    target_state = ekf_->update(measurement);
  }
  else if (yaw_diff > max_match_yaw_diff_)
  {
    // Matched armor not found, but there is only one armor with the same id
    // and yaw has jumped, take this case as the target is spinning and armor
    // jumped
    handleArmorJump(same_num_armors.armors[0]);
  }
  else
  {
    // No matched armor found
    FYT_WARN("armor_solver", "No matched armor found!");
  }


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
  target.yaw = target_state(3);
  target.v_yaw = target_state(4);
  target.radius_1 = 0.276;
  target.radius_2 = 0.276;
  target.d_zc = 0;
  target.id = id;
  target.armors_num = 3;
  return target;
}

rm_interfaces::msg::Target Object::Outpost::predict()
{
  rclcpp::Time now = global_node::Clock->time();
  auto duration = now - last_time_;
  predict_->dt = duration.seconds();
  last_time_ = now;
  target_state = ekf_->predict();

  rm_interfaces::msg::Target target;
  target.position.x = target_state(0);
  target.velocity.x = 0;
  target.acceleration.x = 0;
  target.position.y = target_state(1);
  target.velocity.y = 0;
  target.acceleration.y = 0;
  target.position.z = target_state(2);
  target.velocity.z = 0;
  target.acceleration.z = 0;
  target.yaw = target_state(3);
  target.v_yaw = target_state(4);
  target.radius_1 = 0.276;
  target.radius_2 = 0.276;
  target.d_zc = 0;

  target.armors_num = 3;
  return target;
}

rm_interfaces::msg::Target Object::Outpost::predict(double dt_)
{
  rclcpp::Time now = global_node::Clock->time();
  auto duration = now - last_time_;
  predict_->dt = duration.seconds();
  last_time_ = now;
  target_state = ekf_->predict();

  rm_interfaces::msg::Target target;
  target.position.x = target_state(0);
  target.velocity.x = 0;
  target.acceleration.x = 0;
  target.position.y = target_state(1);
  target.velocity.y = 0;
  target.acceleration.y = 0;
  target.position.z = target_state(2);
  target.velocity.z = 0;
  target.acceleration.z = 0;
  target.yaw = target_state(3);
  target.v_yaw = target_state(4);
  target.radius_1 = 0.276;
  target.radius_2 = 0.276;
  target.d_zc = 0;

  target.armors_num = 3;
  return target;
}
void Object::Outpost::exportNoise(Noise& noise)
{
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

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(X_N);
  double r = 0.276;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  target_state << xc, yc, zc, yaw, w_pre;

  ekf_->setState(target_state);
}

void Object::Outpost::resetFromSensor(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Vector<double, X_N> new_state;
  new_state << pose.position.x, pose.position.y, pose.position.z, KalmanCommon::orientationToYaw(pose.orientation),
    w_pre;
  target_state = new_state;
}

Eigen::Vector3d Object::Outpost::getArmorPositionFromState(const Eigen::VectorXd& x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(1), za = x(2);
  double yaw = x(3);
  double xa = xc - 0.276 * cos(yaw);
  double ya = yc - 0.276 * sin(yaw);
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

void Object::Outpost::handleArmorJump(const rm_interfaces::msg::Armor& current_armor)
{
  double last_yaw = target_state(3);
  double yaw = orientationToYaw(current_armor.pose.orientation);

  if (abs(yaw - last_yaw) > 0.4)
  {
    // Armor angle also jumped, take this case as target spinning
    target_state(3) = yaw;
    FYT_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_)
  {
    // If the distance between the current armor and the inferred armor is too
    // large, the state is wrong, reset center position and velocity in the
    // state
    double r = 0.276;
    target_state(0) = p.x + r * cos(yaw); // xc
    target_state(1) = p.y + r * sin(yaw); // yc
    target_state(2) = p.z; // zc
    FYT_WARN("armor_solver", "State wrong!");
  }

  ekf_->setState(target_state);
}

