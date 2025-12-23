/// Created by Chengfu Zou on 2024.1.19
// Copyright(C) FYT Vision Group. All rights resevred.
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

#include "rm_utils/math/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

namespace fyt {
PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix,
                     const std::vector<double> &distortion_coefficients,
                     cv::SolvePnPMethod method)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone())
, distortion_coefficients_(
    cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion_coefficients.data())).clone())
, method_(method) {}

void PnPSolver::setObjectPoints(const std::string &coord_frame_name,
                                const std::vector<cv::Point3f> &object_points) noexcept {
  object_points_map_[coord_frame_name] = object_points;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point) const noexcept {
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

double PnPSolver::calculateReprojectionError(const std::vector<cv::Point2f> &image_points,
                                             const cv::Mat &rvec,
                                             const cv::Mat &tvec,
                                             const std::string &coord_frame_name) const noexcept {
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto &object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points, rvec, tvec, camera_matrix_, distortion_coefficients_, reprojected_points);
    double error = 0;
    for (size_t i = 0; i < image_points.size(); ++i) {
      error += cv::norm(image_points[i] - reprojected_points[i]);
    }
    return error / image_points.size();
  } else {
    return 0;
  }
}

void PnPSolver::PointReprojection(const std::vector<cv::Point3f> &image_points,
                                             const cv::Mat &rvec,
                                             const cv::Mat &tvec,
                                            std::vector<cv::Point2f> &reprojected_points) const noexcept {
  cv::projectPoints(
    image_points, rvec, tvec, camera_matrix_, distortion_coefficients_, reprojected_points);
}

double PnPSolver::getBestYaw(const std::vector<cv::Point2f> &image_points,
                             const cv::Mat tvec,
                             const std::string &coord_frame_name) const noexcept {
    double pitch = 15.0 / 180 * M_PI;
    double roll = 0;
    double r = M_PI;
    double l = -M_PI;
    while (r-l > 0.0001)
    {
        double a = l + (r - l) / 3;
        double b = r - (r - l) / 3;
        double iou_a = calculateDist(image_points,tvec,roll,pitch,a,coord_frame_name);
        double iou_b = calculateDist(image_points,tvec,roll,pitch,b,coord_frame_name);
        if (iou_a > iou_b) l = a;
        else r = b;
    }
    double yaw = (r+l)/2;
    return yaw;
}

double PnPSolver::calculateDist(const std::vector<cv::Point2f> &image_points,
                               const cv::Mat tvec,
                               double roll, double pitch, double yaw,
                               const std::string &coord_frame_name) const noexcept{
    const auto &object_points = object_points_map_.at(coord_frame_name);
    // 新旋转矩阵
    cv::Mat R_new = rpyToR(roll,pitch,yaw);
    // 新旋转向量
    cv::Mat rvec_new;
    cv::Rodrigues(R_new,rvec_new);
    // 新相机坐标系的点
    std::vector<cv::Point2f> new_image_points;
    cv::projectPoints(object_points,rvec_new,tvec,
                      camera_matrix_,distortion_coefficients_,
                      new_image_points);

    double EuDist = 0.0;
    double min_dist = INT_MAX;
    for (int i = 0 ; i < image_points.size();i++){
        EuDist += std::sqrt(std::pow(image_points[i].x - new_image_points[i].x,2)
                            + std::pow(image_points[i].y - new_image_points[i].y,2));
        if (EuDist < min_dist){
            min_dist = EuDist;
        }
    }
    return min_dist;
}

/**@brief rpyToR
 * 将欧拉角转化为旋转矩阵
 * */
cv::Mat PnPSolver::rpyToR(double roll, double pitch, double yaw) const noexcept {
    cv::Mat R_roll = (cv::Mat_<double>(3,3) <<
            cos(roll), 0, sin(roll),
            0, 1, 0,
            -1*sin(roll),0, cos(roll)
    );

    cv::Mat R_pitch = (cv::Mat_<double>(3,3) <<
            1, 0, 0,
            0, cos(pitch), -1*sin(pitch),
            0, sin(pitch), cos(pitch)
    );

    cv::Mat R_yaw = (cv::Mat_<double>(3,3) <<
            cos(yaw), -1*sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1
    );
    return R_yaw*R_pitch*R_roll;
}

}  // namespace fyt
