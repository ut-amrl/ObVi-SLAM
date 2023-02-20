//
// Created by amanda on 2/5/22.
//

#ifndef UT_VSLAM_POSE_3D_IO_H
#define UT_VSLAM_POSE_3D_IO_H

#include <base_lib/pose_utils.h>
#include <file_io/file_io_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

#include <fstream>

namespace file_io {

void readPose3dLine(const std::vector<std::string> &entries_in_file_line,
                    vslam_types_refactor::Pose3D<double> &pose3d) {
  size_t list_idx = 0;

  std::vector<double> data;
  for (size_t i = 0; i < 7; i++) {
    data.emplace_back(std::stod(entries_in_file_line[list_idx++]));
  }

  pose3d = vslam_types_refactor::Pose3D<double>(
      vslam_types_refactor::Position3d<double>(data[0], data[1], data[2]),
      vslam_types_refactor::Orientation3D<double>(
          Eigen::Quaterniond(data[6], data[3], data[4], data[5])));
}

void readPose3dsFromFile(const std::string &file_name,
                         std::vector<vslam_types_refactor::Pose3D<double>>
                             &pose_3ds_with_timestamp) {
  std::function<void(const std::vector<std::string> &,
                     vslam_types_refactor::Pose3D<double> &)>
      object_from_line_reader = readPose3dLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, pose_3ds_with_timestamp);
}

std::vector<std::string> convertPose3dToStringList(
    const pose::Pose3d &pose_3d_with_timestamp) {
  return {std::to_string(pose_3d_with_timestamp.first.x()),
          std::to_string(pose_3d_with_timestamp.first.y()),
          std::to_string(pose_3d_with_timestamp.first.z()),
          std::to_string(pose_3d_with_timestamp.second.x()),
          std::to_string(pose_3d_with_timestamp.second.y()),
          std::to_string(pose_3d_with_timestamp.second.z()),
          std::to_string(pose_3d_with_timestamp.second.w())};
}

std::vector<std::string> convertPose3DToStringList(
    const vslam_types_refactor::Pose3D<double> &pose_3d) {
  return convertPose3dToStringList(std::make_pair(
      pose_3d.transl_, Eigen::Quaterniond(pose_3d.orientation_)));
}

void writePose3dsWithTimestampToFile(
    const std::string &file_name,
    const std::vector<vslam_types_refactor::Pose3D<double>> &pose_3ds) {
  std::function<std::vector<std::string>(
      const vslam_types_refactor::Pose3D<double> &)>
      object_to_str_list_converter = convertPose3DToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"transl_x",
                               "transl_y",
                               "transl_z",
                               "quat_x",
                               "quat_y",
                               "quat_z",
                               "quat_w"},
                              pose_3ds,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_POSE_3D_IO_H
