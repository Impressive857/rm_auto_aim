// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_utils/math/trajectory_compensator.hpp"
#include <iostream>
namespace fyt {
    bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                           double &pitch) const noexcept {
        double target_height = target_position(2);
        // The iterative_height is used to calculate angle in each iteration
        double iterative_height = target_height;
        double impact_height = 0;
        double distance =
                std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
        double angle = std::atan2(target_height, distance);
        double dh = 0;
        // Iterate to find the right angle, which makes the impact height equal to the
        // target height
        for (int i = 0; i < iteration_times; ++i) {
            angle = std::atan2(iterative_height, distance);
            if (std::abs(angle) > M_PI / 2.5) {
                break;
            }
            impact_height = calculateTrajectory(distance, angle);
            dh = target_height - impact_height;
            if (std::abs(dh) < 0.01) {
                break;
            }
            iterative_height += dh;
        }
        if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
            return false;
        }
        pitch = angle;
        return true;
    }

    std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
            double distance, double angle) const noexcept {
        std::vector<std::pair<double, double>> trajectory;

        if (distance < 0) {
            return trajectory;
        }

        for (double x = 0; x < distance; x += 0.03) {
            trajectory.emplace_back(x, calculateTrajectory(x, angle));
        }
        return trajectory;
    }

    double IdealCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
        double t = x / (velocity * cos(angle));
        double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
        return y;
    }

    double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
        double distance =
                sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
        double angle = atan2(target_position(2), distance);
        double t = distance / (velocity * cos(angle));
        return t;
    }

    double ResistanceCompensator::calculateTrajectory(const double x,
                                                      const double angle) const noexcept {
        double r = resistance < 1e-4 ? 1e-4 : resistance;
        double t = (exp(r * x) - 1) / (r * velocity * cos(angle));
        double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
        return y;
    }

    double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
        double r = resistance < 1e-4 ? 1e-4 : resistance;
        double distance =
                sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
        double angle = atan2(target_position(2), distance);
        double t = (exp(r * distance) - 1) / (r * velocity * cos(angle));
        return t;
    }

    struct Ballistic_Coefficients_t {
        float p00;
        float p10;
        float p01;
        float p20;
        float p11;
        float p02;
        float p30;
        float p21;
        float p12;
        float p03;
    };

//实际弹速25左右
    Ballistic_Coefficients_t ballTime = {0.01626, 0.03098, 0.0002764, 0.001613, 0.0002167,
                                         0.01009, -4.4e-05, 1.656e-05, -0.0007987, 4.406e-05};
    Ballistic_Coefficients_t ballAngle = {-0.002708, 0.4523, -0.001145, 0.004571, 0.003363,
                                          0.001887, 0.0001523, 0.0001772, 0.0002688, 0.0001827};
//实际弹速15.8左右
    Ballistic_Coefficients_t bigBallAngle = {-0.01569, 1.174, 0.01018, 0.00105, 0.01478,
                                             -0.0007071, 0.0008856, 0.001672, 0.002273, 0.0009303};
    Ballistic_Coefficients_t bigBallTime = {0.02227, 0.05283, 0.001351, 0.001727, 0.0006762,
                                            0.01437, -3.16e-05, 8.646e-05, -0.001054, 0.0001896};

    double PolyCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
        return 0.0;
    }

    double PolyCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
        float s2 = powf(target_position[0], 2) + powf(target_position[1], 2);
        float s = sqrtf(s2);
        float s3 = s * s2;
        float h = target_position[2];
        float h2 = h * h;
        float h3 = h2 * h;
        if (velocity > 20) {
            return ballTime.p00 + ballTime.p10 * s + ballTime.p01 * h +
                   ballTime.p20 * s2 + ballTime.p11 * s * h +
                   ballTime.p02 * h2 + ballTime.p30 * s3 + ballTime.p21 * s2 * h +
                   ballTime.p12 * s * h2 + ballTime.p03 * h3;
        } else {
            return bigBallTime.p00 + bigBallTime.p10 * s + bigBallTime.p01 * h +
                   bigBallTime.p20 * s2 + bigBallTime.p11 * s * h +
                   bigBallTime.p02 * h2 + bigBallTime.p30 * s3 + bigBallTime.p21 * s2 * h +
                   bigBallTime.p12 * s * h2 + bigBallTime.p03 * h3;
        }

    }

    bool PolyCompensator::compensate(const Eigen::Vector3d &target_position,
                                     double &pitch) const noexcept {

        float s2 = powf(target_position[0], 2) + powf(target_position[0], 2);
        float s = sqrtf(s2);
        float s3 = s * s2;
        float h = target_position[2];
        float h2 = h * h;
        float h3 = h2 * h;
        if (velocity > 20) {
            pitch = ballAngle.p00 + ballAngle.p10 * s + ballAngle.p01 * h +
                    ballAngle.p20 * s2 + ballAngle.p11 * s * h +
                    ballAngle.p02 * h2 + ballAngle.p30 * s3 + ballAngle.p21 * s2 * h +
                    ballAngle.p12 * s * h2 + ballAngle.p03 * h3 + atan2f(h, s) * 180 / M_PI;
        } else {
            pitch = bigBallAngle.p00 + bigBallAngle.p10 * s + bigBallAngle.p01 * h +
                    bigBallAngle.p20 * s2 + bigBallAngle.p11 * s * h +
                    bigBallAngle.p02 * h2 + bigBallAngle.p30 * s3 +
                    bigBallAngle.p21 * s2 * h +
                    bigBallAngle.p12 * s * h2 + bigBallAngle.p03 * h3 + atan2f(h, s) * 180 / M_PI;
        }
        return true;
    }

}  // namespace fyt
