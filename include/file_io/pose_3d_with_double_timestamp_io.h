//
// Created by amanda on 2/21/21.
//

#ifndef UT_VSLAM_POSE_3D_WITH_DOUBLE_TIMESTAMP_IO_H
#define UT_VSLAM_POSE_3D_WITH_DOUBLE_TIMESTAMP_IO_H

#include <base_lib/pose_reps.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>

namespace file_io {

struct Pose3DWithDoubleTimestamp {
  double timestamp_;
  double transl_x_;
  double transl_y_;
  double transl_z_;
  double quat_w_;
  double quat_x_;
  double quat_y_;
  double quat_z_;
};

void readPose3DWithDoubleTimestampLine(
    const std::string &line_in_file,
    Pose3DWithDoubleTimestamp &lidar_odom_est_data) {
  std::stringstream ss(line_in_file);
  std::vector<double> data;
  while (ss.good()) {
    std::string substr;
    getline(ss, substr, ' ');
    data.push_back(std::stod(substr));
  }
  lidar_odom_est_data.timestamp_ = data[0];
  lidar_odom_est_data.transl_x_ = data[1];
  lidar_odom_est_data.transl_y_ = data[2];
  lidar_odom_est_data.transl_z_ = data[3];
  lidar_odom_est_data.quat_w_ = data[4];
  lidar_odom_est_data.quat_x_ = data[5];
  lidar_odom_est_data.quat_y_ = data[6];
  lidar_odom_est_data.quat_z_ = data[7];
}

void readPose3DWithDoubleTimestampFromFile(
    const std::string &file_name,
    std::vector<Pose3DWithDoubleTimestamp> &odom_ests_in_abs_frame) {
  std::ifstream file_obj(file_name);
  std::string line;
  bool first_line = true;
  while (std::getline(file_obj, line)) {
    if (first_line) {
      first_line = false;
      continue;
    }

    Pose3DWithDoubleTimestamp lidar_odom_est_data;
    readPose3DWithDoubleTimestampLine(line, lidar_odom_est_data);
    odom_ests_in_abs_frame.emplace_back(lidar_odom_est_data);
  }
  if (first_line) {
    throw std::invalid_argument("Pose3d with double file was empty");
  }
}
}  // namespace file_io

#endif  // UT_VSLAM_POSE_3D_WITH_DOUBLE_TIMESTAMP_IO_H
