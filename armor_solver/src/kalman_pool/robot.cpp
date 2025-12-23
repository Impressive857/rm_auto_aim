//
// Created by lbw on 25-3-13.
//

#include "armor_solver/kalman_pool/robot.h"

#include <ceres/ceres.h>
#include <angles/angles.h>
#include <visualization/visualization.h>

#include "parameter/parameter.h"

//迭代初始点选取算法
// #define Algorithm_Rotation_Matrix //基于等长半径计算
// #define Algorithm_Geometry
// #define Algorithm_Vector

Object::Robot::Robot(const std::string &ID): ObjectBase(ID) {
  dt_ = 0.05;
  r_avg[0] = 0.26;
  r_avg[1] = 0.26;
  r_count = 0;
  first_observe = true;
  Robot::init();
}

Object::Robot::~Robot() {
}

void Object::Robot::init() {
  observer_ = std::make_unique<Observer>([this]() { return dt_; });
  Observer::Noise noise;
  global_node::Parameter->get_parameter("robot.q_x", noise.q_x);
  global_node::Parameter->get_parameter("robot.q_y", noise.q_y);
  global_node::Parameter->get_parameter("robot.q_z", noise.q_z);
  global_node::Parameter->get_parameter("robot.r_x", noise.r_x);
  global_node::Parameter->get_parameter("robot.r_y", noise.r_y);
  global_node::Parameter->get_parameter("robot.r_z", noise.r_z);

  global_node::Parameter->get_parameter("robot.q_yaw", noise.q_yaw);
  global_node::Parameter->get_parameter("robot.r_yaw", noise.r_yaw);

  global_node::Parameter->get_parameter("robot.q_sx", noise.qs_x);
  global_node::Parameter->get_parameter("robot.q_sy", noise.qs_y);
  global_node::Parameter->get_parameter("robot.q_sz", noise.qs_z);
  global_node::Parameter->get_parameter("robot.q_syaw", noise.qs_yaw);
  global_node::Parameter->get_parameter("robot.q_sr", noise.qs_r);
  global_node::Parameter->get_parameter("robot.q_sdz", noise.qs_dz);
  global_node::Parameter->get_parameter("robot.rd_x", noise.rd_x);
  global_node::Parameter->get_parameter("robot.rd_y", noise.rd_y);
  global_node::Parameter->get_parameter("robot.rd_z", noise.rd_z);
  global_node::Parameter->get_parameter("robot.rd_r", noise.rd_r);
  global_node::Parameter->get_parameter("robot.rd_dz", noise.rd_dz);

  global_node::Parameter->get_parameter("robot.q_ro", noise.q_ro);
  global_node::Parameter->get_parameter("robot.r_ro", noise.r_ro);

  global_node::Parameter->get_parameter("robot.max_match_distance", max_match_distance_);
  global_node::Parameter->get_parameter("robot.max_match_yaw_diff", max_match_yaw_diff_);
  observer_->setNoise(noise);
  measurement = Eigen::Vector4d(0, 0, 0, 0);
}

rm_interfaces::msg::Measurement Object::Robot::update(rm_interfaces::msg::Armors &same_num_armors) {
  rm_interfaces::msg::Measurement measurement_msg;
  ckyf::visualization::CenterMeasure center_measure{};
  center_measure.isDetect = false;
  if (first_frame_) {
    std::cout << "First Frame" << std::endl;
    reset(same_num_armors.header.stamp, same_num_armors.armors[0]);
    first_observe = false;
    dt_ = 0.05;
  } else {
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

  std::function calc_diff = [&
      ](const rm_interfaces::msg::Armor &armor, double &yaw_diff, double &position_diff)mutable {
    auto p = armor.pose.position;
    Eigen::Vector3d position_vec(p.x, p.y, p.z);
    position_diff = (position_vec - prediction_armor_position).norm();
    yaw_diff = orientationToYaw(armor.pose.orientation) - prediction_yaw(6);
  };

  rm_interfaces::msg::Armor tracked_armor;
  double measurement_yaw;
  if (same_num_armors.armors.size() == 1) {
    tracked_armor = same_num_armors.armors[0];
    calc_diff(tracked_armor, min_yaw_diff, min_position_diff);
  } else {
    std::sort(same_num_armors.armors.begin(), same_num_armors.armors.end(), [&](auto &armor1, auto &armor2) mutable {
      double position_diff1, position_diff2;
      double yaw_diff1, yaw_diff2;
      calc_diff(armor1, yaw_diff1, position_diff1);
      calc_diff(armor2, yaw_diff2, position_diff2);
      if (std::abs(position_diff1) < std::abs(position_diff2)) {
        min_position_diff = position_diff1;
        min_yaw_diff = yaw_diff1;
        return true;
      } else {
        min_position_diff = position_diff2;
        min_yaw_diff = yaw_diff2;
        return false;
      }
    });
  }

  tracked_armor = same_num_armors.armors[0];
  measurement_yaw = orientationToYaw(tracked_armor.pose.orientation);
  measurement_msg.yaw = measurement_yaw;

  if (same_num_armors.armors.size() >= 2) {
    rm_interfaces::msg::Armor another_armor = same_num_armors.armors[1];
    // std::cout << "update for 2 armors" << std::endl;

    Eigen::Vector2d center_position(target_state(0), target_state(1));
    if (armors_to_center(same_num_armors, center_position)) {
      //同时观测两块装甲板
      Eigen::Vector2d p_armor1(same_num_armors.armors[0].pose.position.x, same_num_armors.armors[0].pose.position.y);
      Eigen::Vector2d p_armor2(same_num_armors.armors[1].pose.position.x, same_num_armors.armors[1].pose.position.y);
      // std::cout << p_armor1 << std::endl << p_armor2 << std::endl << "==============" << std::endl;
      double r[2];
      r[0] = (p_armor1 - center_position).norm();
      r[1] = (p_armor2 - center_position).norm();
      // std::cout << "armor1:" << p_armor1 << std::endl;
      // std::cout << "armor2:" << p_armor2 << std::endl;
      // std::cout << "center:" << center_position << std::endl;
      // std::cout << "r1: " << r[0] << ", " << r_avg[0] << ", " << r_count << std::endl;
      // std::cout << "r2: " << r[1] << ", " << r_avg[1] << ", " << r_count << std::endl;
      // std::cout << "*******************************" << std::endl;

      if (r[0] < 0.4 && r[0] > 0.12 && r[1] < 0.4 && r[1] > 0.12) {
        global_node::Visualization->debug_user.debug1 = r[0];
        global_node::Visualization->debug_user.debug2 = r[1];
        if (just_reset) {
          r_avg[0] = r[0];
          r_avg[1] = r[1];
          r_count = 1;
          just_reset = false;
        }
        //判断当前追踪的装甲板 0是低板，1是高板
        if (same_num_armors.armors[0].pose.position.z > same_num_armors.armors[1].pose.position.z) //armor0是高板
        {
          r_avg[0] = (r_avg[0] * r_count + r[1]) / (r_count + 1);
          r_avg[1] = (r_avg[1] * r_count + r[0]) / (r_count + 1);
        } else {
          r_avg[0] = (r_avg[0] * r_count + r[0]) / (r_count + 1);
          r_avg[1] = (r_avg[1] * r_count + r[1]) / (r_count + 1);
        }
        r_count = std::min(r_count + 1, 500.0);

        observer_->setState(target_state);

        center_measure.isDetect = true;
        center_measure.x = center_position(0);
        center_measure.y = center_position(1);
      } else {
        FYT_ERROR("armor_solver", "Invalid R Solved");
      }
    } else {
      FYT_WARN("armor_solver", "Observe two armors but not the same robot");
    }
  }

  if (std::abs(min_yaw_diff) < max_match_yaw_diff_ && min_position_diff < max_match_distance_) {
    Eigen::Matrix<double, 1, 1> yaw_measurement;
    yaw_measurement << measurement_yaw;
    observer_->updateYaw(yaw_measurement);
    updateTranslate(tracked_armor, prediction_yaw, measurement_msg);
  } else {
    if (std::abs(min_yaw_diff) > max_match_yaw_diff_ && same_num_armors.armors.size() == 1) {
      FYT_INFO("armor_solver", "[Observer]Armor Yaw Jumped!");
      handleArmorYawJump(tracked_armor);
    } else if (min_position_diff > max_match_distance_) {
      FYT_INFO("armor_solver", "[Observer]Armor Position Jumped!");
      handleArmorPositionJump(tracked_armor);
    } else {
      // reset(same_num_armors.header.stamp, tracked_armor);
      FYT_ERROR("armor_solver", "[Observer]Wrong!");
      std::cout << "min_yaw:" << min_yaw_diff << " min_position:" << min_position_diff << std::endl;
      observer_->setState(prediction_all);
    }
  }


  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    observer_->setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    observer_->setState(target_state);
  }

  if (target_state(5) > 0.5) {
    target_state(5) = 0.5;
    observer_->setState(target_state);
  } else if (target_state(5) < -0.5) {
    target_state(5) = -0.5;
    observer_->setState(target_state);
  }
  global_node::Visualization->debug_measure_center_marker(center_measure);
  return measurement_msg;
}

rm_interfaces::msg::Target Object::Robot::getState() {
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
  target.radius_1 = target_state(8);
  target.radius_2 = another_r;
  target.d_zc = target_state(9);
  target.id = id;
  target.armors_num = 4;
  return target;
}

rm_interfaces::msg::Target Object::Robot::predict() {
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
  target.radius_1 = target_state(8);
  target.radius_2 = another_r;
  target.d_zc = target_state(9);
  target.armors_num = 4;
  return target;
}

rm_interfaces::msg::Target Object::Robot::predict(double dt_) {

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
  target.radius_1 = target_state(8);
  target.radius_2 = another_r;
  target.d_zc = target_state(9);
  target.armors_num = 4;
  return target;
}
void Object::Robot::reset(const rclcpp::Time &time, const rm_interfaces::msg::Armor &armor) {
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
  if (first_observe) {
    another_r = r = 0.26;
    just_reset = false;
  } else {
    r = r_avg[0];
    another_r = r_avg[1];
    just_reset = true;
  }
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  d_za = 0, d_zc = 0;
  target_state << xc, yc, zc, 0, 0, 0, yaw, 0, r, d_zc;

  observer_->setState(target_state);
}

void Object::Robot::resetFromSensor(const geometry_msgs::msg::Pose &pose) {
  target_state << pose.position.x, 0, pose.position.y, 0, pose.position.z, 0,
      KalmanCommon::orientationToYaw(pose.orientation), 0, 0.20, d_zc;
}

Eigen::Vector3d Object::Robot::getArmorPositionFromState(const Eigen::VectorXd &x) {
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(1), za = x(2) + x(9);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

double Object::Robot::orientationToYaw(const geometry_msgs::msg::Quaternion &q) {
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

void Object::Robot::handleArmorYawJump(const rm_interfaces::msg::Armor &current_armor) {
  double last_yaw = target_state(6);
  double yaw = orientationToYaw(current_armor.pose.orientation);
  Eigen::Vector3d p(current_armor.pose.position.x, current_armor.pose.position.y, current_armor.pose.position.z);
  if (abs(yaw - last_yaw) > 0.4) {
    // Armor angle also jumped, take this case as target spinning
    target_state(6) = yaw;
    // Only 4 armors has 2 radius and height

    d_za = target_state(2) + target_state(9) - current_armor.pose.position.z;
    //判断装甲板的半径对应关系，半径是否需要交换

    if (d_za > 0) {
      target_state(8) = r_avg[0];
      another_r = r_avg[1];
    } else {
      target_state(8) = r_avg[1];
      another_r = r_avg[0];
    }

    d_zc = d_zc == 0 ? -d_za : 0;
    target_state(9) = d_zc;
    just_reset = false;
    FYT_DEBUG("armor_solver", "[Observer]Armor Jump!");
  }
  observer_->setState(target_state);
}

void Object::Robot::handleArmorPositionJump(const rm_interfaces::msg::Armor &current_armor) {
  last_time_ = global_node::Clock->time();

  double xa = current_armor.pose.position.x;
  double ya = current_armor.pose.position.y;
  double za = current_armor.pose.position.z;
  double yaw = KalmanCommon::orientationToYaw(current_armor.pose.orientation);
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;

  // Set initial position at 0.2 m behind the target
  double r = target_state(8);
  target_state(0) = xa + r * cos(yaw);
  target_state(1) = ya + r * sin(yaw);
  target_state(2) = za;
  d_za = 0, d_zc = 0;
  target_state(6) = yaw;
  observer_->setState(target_state);
}

bool Object::Robot::armors_to_center(const rm_interfaces::msg::Armors &armors_msg, Eigen::Vector2d &center) {
  auto yaw_in_camera1 = armors_msg.armors[0].yaw_in_camera;
  auto yaw_in_camera2 = armors_msg.armors[1].yaw_in_camera;
  double yaw1 = orientationToYaw(armors_msg.armors[0].pose.orientation);
  double yaw2 = orientationToYaw(armors_msg.armors[1].pose.orientation);
  double x1 = armors_msg.armors[0].pose.position.x, y1 = armors_msg.armors[0].pose.position.y;
  double x2 = armors_msg.armors[1].pose.position.x, y2 = armors_msg.armors[1].pose.position.y;
#ifdef Algorithm_Rotation_Matrix
  const auto& armor1 = armors_msg.armors[0];
  const auto& armor2 = armors_msg.armors[1];
  // std::cout << "yaw1: " << yaw1 << " yaw2: " << yaw2 << " delta: " << yaw2 - yaw1 << std::endl;
  if (abs(yaw1 - yaw2) > 1.8 || abs(yaw2 - yaw1) < 1.4)
  {
    return false;
  }

  double x1, y1, x2, y2;
  x1 = armor1.pose.position.x;
  y1 = armor1.pose.position.y;
  x2 = armor2.pose.position.x;
  y2 = armor2.pose.position.y;
  Eigen::Vector2d p1(x1, y1);
  Eigen::Vector2d p2(x2, y2);
  double theta = yaw2 - yaw1;

  Eigen::Matrix2d R;
  R << cos(theta), -sin(theta),
    sin(theta), cos(theta);

  auto R_I_inv = (R - Eigen::Matrix2d::Identity()).inverse();
  center = R_I_inv * (R * p1 - p2);

#endif

#ifdef Algorithm_Geometry
  Eigen::Vector2d v1(x1, x2);
  Eigen::Vector2d v2(cos(yaw1) * sin(yaw2), -cos(yaw2) * sin(yaw1));
  Eigen::Vector2d kalman_center = center;
  center.x() = (tan(yaw1) * x1 - tan(yaw2) * x2 + y2 - y1) / (tan(yaw1) - tan(yaw2));
  center.y() = sin(yaw1) == 0 ? center.x() : sin(yaw1) / cos(yaw1) * (center.x() - x1) + y1;


#endif

  iteration_solver(center.x(), center.y(), x1, y1, x2, y2, yaw1, yaw2, center);
  return true;
}

void Object::Robot::updateTranslate(rm_interfaces::msg::Armor &tracked_armor,
                                    Eigen::MatrixXd position_only_predict_yaw,
                                    rm_interfaces::msg::Measurement &measurement_msg) {
  const auto &p = tracked_armor.pose.position;
  Eigen::Vector3d armor_position(p.x, p.y, p.z);
  auto armor_position_kf = getArmorPositionFromState(position_only_predict_yaw);
  //Eigen::Vector3d measured_center = (armor_position - armor_position_kf) + position_only_predict_yaw.block(0, 0, 3, 1);
  double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
  double r = target_state(8);
  double yaw = target_state(6);
  double dz = target_state(9);
  Eigen::Vector3d measured_center = Eigen::Vector3d(p.x + r * cos(yaw), p.y + r * sin(yaw), p.z - dz);
  observer_->predictTranslate();
  target_state = observer_->updateTranslate(measured_center, tracked_armor.yaw_in_camera);
  global_node::Visualization->debug_user.debug10 = tracked_armor.yaw_in_camera;
  measurement_msg.x = p.x;
  measurement_msg.y = p.y;
  measurement_msg.z = p.z;

  global_node::Visualization->debug_user.debug4 = p.x;
  global_node::Visualization->debug_user.debug5 = -r * cos(yaw) + 2.3;
  global_node::Visualization->debug_user.debug6 = (r * cos(yaw) + p.x);
  global_node::Visualization->debug_user.debug7 = p.y;
  global_node::Visualization->debug_user.debug8 = -r * sin(yaw) - 0.1;
  global_node::Visualization->debug_user.debug9 = (r * sin(yaw) + p.y);
}

void Object::Robot::iteration_solver(double &x, double &y,
                                     double x1, double y1,
                                     double x2, double y2,
                                     double yaw1, double yaw2,
                                     Eigen::Vector2d kalman_center) {
  double mu_yaw = 0.1; //yaw约束的惩罚系数
  double mu_center = 10.0; //中心点约束的惩罚系数

  ceres::Problem problem;
  ceres::CostFunction *cost_for_point1 =
      new ceres::AutoDiffCostFunction<YawResidual, 1, 1, 1>(
        new YawResidual(x1, y1, x2, y2, yaw1, yaw2, mu_yaw));
  problem.AddResidualBlock(cost_for_point1, nullptr, &x, &y);

  ceres::CostFunction *cost_for_kalman =
      new ceres::AutoDiffCostFunction<CenterResidual, 1, 1, 1>(
        new CenterResidual(kalman_center.x(), kalman_center.y(), mu_center));
  problem.AddResidualBlock(cost_for_kalman, nullptr, &x, &y);

  ceres::CostFunction *cost_for_length =
      new ceres::AutoDiffCostFunction<LengthResidual, 1, 1, 1>(
        new LengthResidual(x1, y1, x2, y2));
  problem.AddResidualBlock(cost_for_length, nullptr, &x, &y);


  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 5;

  // 运行优化
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
}
