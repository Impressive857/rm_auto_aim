//
// Created by lbw on 25-4-22.
//

#include "armor_solver/kalman_pool/motion_model/moving_top.h"

#include <angles/angles.h>
#include <visualization/visualization.h>
#include <rm_utils/logger/log.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <parameter/parameter.h>

#include "../../../submodule/clock/include/clock/clock.h"

MotionModel::MovingTop::MovingTop(const std::string& ID): ObjectBase(ID)
{
  MovingTop::init();
}

MotionModel::MovingTop::~MovingTop()
{
  delete predict_;
  delete smeasure_;
  delete dmeasure_;
}

void MotionModel::MovingTop::init()
{
  getParam();

  predict_ = new Predict(0.005);
  smeasure_ = new sMeasure;
  dmeasure_ = new dMeasure;
  target_state = Eigen::Vector<double,10>::Zero();
  auto u_q = [this]()
  {
    Eigen::Matrix<double, X_N, X_N> q;
    double t = dt_, x = noise_.s2qx, y = noise_.s2qy, z = noise_.s2qz,
           yaw = noise_.s2qyaw + 0.00 * target_state(7),
           r = noise_.s2qr,
           d_zc = noise_.s2qd_zc;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * z, q_vz_vz = pow(t, 2) * z;
    double q_yaw_yaw = pow(t, 4) / 4 * yaw, q_yaw_vyaw = pow(t, 3) / 2 * yaw,
           q_vyaw_vyaw = pow(t, 2) * yaw;
    double q_r = pow(t, 4) / 4 * r;
    double q_d_zc = pow(t, 4) / 4 * d_zc;
        // clang-format off
        //    xc      v_xc    yc      v_yc    zc      v_zc    yaw         v_yaw       r       d_za
        q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,          0,          0,      0,
              q_x_vx, q_vx_vx,0,      0,      0,      0,      0,          0,          0,      0,
              0,      0,      q_y_y,  q_y_vy, 0,      0,      0,          0,          0,      0,
              0,      0,      q_y_vy, q_vy_vy,0,      0,      0,          0,          0,      0,
              0,      0,      0,      0,      q_z_z,  q_z_vz, 0,          0,          0,      0,
              0,      0,      0,      0,      q_z_vz, q_vz_vz,0,          0,          0,      0,
              0,      0,      0,      0,      0,      0,      q_yaw_yaw,  q_yaw_vyaw, 0,      0,
              0,      0,      0,      0,      0,      0,      q_yaw_vyaw, q_vyaw_vyaw,0,      0,
              0,      0,      0,      0,      0,      0,      0,          0,          q_r,    0,
              0,      0,      0,      0,      0,      0,      0,          0,          0,      q_d_zc;

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
  ekf_ = std::make_unique<myself::ExtendedKalmanFilter<X_N, Z_N, Predict, sMeasure>>(predict_, smeasure_, u_q, u_r, p0);

  measurement = Eigen::Vector4d(0, 0, 0, 0);
}

void MotionModel::MovingTop::getParam()
{
  global_node::Parameter->get_parameter("moving_top.q_x", noise_.s2qx);
  global_node::Parameter->get_parameter("moving_top.q_y", noise_.s2qy);
  global_node::Parameter->get_parameter("moving_top.q_z", noise_.s2qz);
  global_node::Parameter->get_parameter("moving_top.q_yaw", noise_.s2qyaw);
  global_node::Parameter->get_parameter("moving_top.q_r", noise_.s2qr);
  global_node::Parameter->get_parameter("moving_top.q_zc", noise_.s2qd_zc);
  global_node::Parameter->get_parameter("moving_top.r_x", noise_.r_x);
  global_node::Parameter->get_parameter("moving_top.r_y", noise_.r_y);
  global_node::Parameter->get_parameter("moving_top.r_z", noise_.r_z);
  global_node::Parameter->get_parameter("moving_top.r_yaw", noise_.r_yaw);
  //std::cout << noise_ << std::endl;
  global_node::Parameter->get_parameter("moving_top.max_match_distance", max_match_distance_);
  global_node::Parameter->get_parameter("moving_top.max_match_y_diff", max_match_yaw_diff_);
}

rm_interfaces::msg::Measurement MotionModel::MovingTop::update(rm_interfaces::msg::Armors& same_num_armors)
{
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
      yaw_diff = orientationToYaw(armor.pose.orientation) - ekf_prediction(6);
      tracked_armor = armor;
    }
  }
  // Check if the distance and yaw difference of closest armor are within the
  // threshold
  if (min_position_diff < max_match_distance_ && (std::abs(yaw_diff) < max_match_yaw_diff_ || yaw_diff * target_state(7)
    > 0))
  {
    // Matched armor found
    auto p = tracked_armor.pose.position;
    // Update EKF
    double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
    Eigen::Vector2d center;
    if (armors_to_center(same_num_armors, center))
    {
      auto tracked_vec = Eigen::Vector2d(tracked_armor.pose.position.x, tracked_armor.pose.position.y);
      double r = (center - tracked_vec).norm();
      Eigen::Vector<double, 5> measurement__;
      measurement << p.x, p.y, p.z, measured_yaw;
      measurement__ << p.x, p.y, p.z, measured_yaw, r;
      auto u_r = [this](const Eigen::Matrix<double, NZ_N, 1>& z)
      {
        Eigen::Matrix<double, NZ_N, NZ_N> r;
        // clang-format off
        r << noise_.r_x * std::abs(z[0]), 0, 0, 0, 0,
             0, noise_.r_y * std::abs(z[1]), 0, 0, 0,
             0, 0, noise_.r_z * std::abs(z[2]), 0, 0,
             0, 0, 0, noise_.r_yaw,0,
             0, 0, 0, 0, 0.01;
        // clang-format on
        return r;
      };
      target_state = ekf_->update<dMeasure, NZ_N>(measurement__, dmeasure_, u_r);
    }
    else
    {
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf_->update(measurement);
    }
  }
  else if (same_num_armors.armors.size() == 1 && std::abs(yaw_diff) > max_match_yaw_diff_ && yaw_diff * target_state(7)
    < 0)
  {
    // Matched armor not found, but there is only one armor with the same id
    // and yaw has jumped, take this case as the target is spinning and armor
    // jumped
    handleArmorJump(same_num_armors.armors[0]);
  }
  else
  {
    // No matched armor found
    FYT_WARN("armor_solver", "[MovingTop]No matched armor found!");
  }


  // Prevent radius from spreading
  if (target_state(8) < 0.12)
  {
    target_state(8) = 0.12;
    ekf_->setState(target_state);
  }
  else if (target_state(8) > 0.4)
  {
    target_state(8) = 0.4;
    ekf_->setState(target_state);
  }
  rm_interfaces::msg::Measurement measurement_msg;
  measurement_msg.x = measurement(0);
  measurement_msg.y = measurement(1);
  measurement_msg.z = measurement(2);
  measurement_msg.yaw = measurement(3);
  return measurement_msg;
}

Eigen::MatrixXd MotionModel::MovingTop::getState()
{
  return target_state;
}

rm_interfaces::msg::Target MotionModel::MovingTop::predict()
{
  rclcpp::Time now = global_node::Clock->time();
  auto duration = now - last_time_;
  predict_->dt = duration.seconds();
  last_time_ = now;
  target_state = ekf_->predict();

  rm_interfaces::msg::Target target;
  target.position.x = target_state(0);
  target.velocity.x = target_state(1);
  target.acceleration.x = 0;
  target.position.y = target_state(2);
  target.velocity.y = target_state(3);
  target.acceleration.y = 0;
  target.position.z = target_state(4);
  target.velocity.z = target_state(5);
  target.acceleration.y = 0;
  target.yaw = target_state(6);
  target.v_yaw = target_state(7);
  target.radius_1 = target_state(8);
  target.radius_2 = another_r;
  target.d_zc = target_state(9);
  target.armors_num = 4;
  return target;
}


void MotionModel::MovingTop::reset(const rclcpp::Time& time, const rm_interfaces::msg::Armor& armor)
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
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  d_za = 0, d_zc = 0, another_r = r;
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  ekf_->setState(target_state);
}

void MotionModel::MovingTop::resetFromSensor(const geometry_msgs::msg::Pose& pose)
{
  target_state << pose.position.x, 0, pose.position.y, 0, pose.position.z, 0,
    KalmanCommon::orientationToYaw(pose.orientation), 3.48, 0.26, d_zc;
}

Eigen::Vector3d MotionModel::MovingTop::getArmorPositionFromState(const Eigen::VectorXd& x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

double MotionModel::MovingTop::orientationToYaw(const geometry_msgs::msg::Quaternion& q)
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

void MotionModel::MovingTop::handleArmorJump(const rm_interfaces::msg::Armor& current_armor)
{
  double last_yaw = target_state(6);
  double yaw = orientationToYaw(current_armor.pose.orientation);

  if (abs(yaw - last_yaw) > 0.4)
  {
    // Armor angle also jumped, take this case as target spinning
    target_state(6) = yaw;
    // Only 4 armors has 2 radius and height

    d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
    std::swap(target_state(8), another_r);
    d_zc = d_zc == 0 ? -d_za : 0;
    target_state(9) = d_zc;

    FYT_DEBUG("armor_solver", "[MovingTop]Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_)
  {
    // If the distance between the current armor and the inferred armor is too
    // large, the state is wrong, reset center position and velocity in the
    // state
    d_zc = 0;
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw); // xc
    target_state(1) = 0; // vxc
    target_state(2) = p.y + r * sin(yaw); // yc
    target_state(3) = 0; // vyc
    target_state(4) = p.z; // zc
    target_state(5) = 0; // vzc
    target_state(9) = d_zc; // d_zc
    FYT_WARN("armor_solver", "[MovingTop]State wrong!");
  }

  ekf_->setState(target_state);
}

bool MotionModel::MovingTop::armors_to_center(const rm_interfaces::msg::Armors& armors_msg, Eigen::Vector2d& center)
{
  if (armors_msg.armors.size() != 2)
    return false;
  const auto& xarmor1 = armors_msg.armors[0];
  const auto& xarmor2 = armors_msg.armors[1];
  double yaw1 = KalmanCommon::orientationToYaw(xarmor1.pose.orientation);
  double yaw2 = KalmanCommon::orientationToYaw(xarmor2.pose.orientation);
  if (abs(yaw1 - yaw2) > 1.6)
  {
    return false;
  }

  rm_interfaces::msg::Armor armor1, armor2;
  if (yaw1 < yaw2)
  {
    armor1 = armors_msg.armors[0];
    armor2 = armors_msg.armors[1];
  }
  else
  {
    armor1 = armors_msg.armors[1];
    armor2 = armors_msg.armors[0];
  }
  double x1, y1, x2, y2;
  x1 = armor1.pose.position.x;
  y1 = armor1.pose.position.y;
  x2 = armor2.pose.position.x;
  y2 = armor2.pose.position.y;

  Eigen::Matrix2d R1;
  R1 << cos(yaw1), -sin(yaw1),
    sin(yaw1), cos(yaw1);
  Eigen::Vector2d v;
  v << x1 * cos(yaw1) + y1 * sin(yaw1),
    x2 * cos(yaw2) - y2 * sin(yaw2);
  center = R1 * v;
  return true;
}
