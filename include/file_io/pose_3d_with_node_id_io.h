//
// Created by amanda on 2/5/22.
//

#ifndef UT_VSLAM_POSE_3D_WITH_NODE_ID_IO_H
#define UT_VSLAM_POSE_3D_WITH_NODE_ID_IO_H

#include <base_lib/pose_reps.h>
#include <file_io/file_io_utils.h>

#include <fstream>

namespace file_io {

void readPose3dWithNodeIdLine(
    const std::vector<std::string> &entries_in_file_line,
    std::pair<uint64_t, pose::Pose3d> &pose3d_with_node_id) {
  uint64_t node_id;
  size_t list_idx = 0;
  std::istringstream stream_node_id(entries_in_file_line[list_idx++]);
  stream_node_id >> node_id;

  std::vector<double> data;
  for (size_t i = 0; i < 7; i++) {
    data.emplace_back(std::stod(entries_in_file_line[list_idx++]));
  }

  pose::Pose3d pose_3d =
      std::make_pair(Eigen::Vector3d(data[0], data[1], data[2]),
                     Eigen::Quaterniond(data[6], data[3], data[4], data[5]));

  pose3d_with_node_id = std::make_pair(node_id, pose_3d);
}

void readPose3dsAndNodeIdFromFile(
    const std::string &file_name,
    std::vector<std::pair<uint64_t, pose::Pose3d>> &pose_3ds_with_node_id) {
  std::function<void(const std::vector<std::string> &,
                     std::pair<uint64_t, pose::Pose3d> &)>
      object_from_line_reader = readPose3dWithNodeIdLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, pose_3ds_with_node_id);
}

std::vector<std::string> convertPose3dWithNodeIdToStringList(
    const std::pair<uint64_t, pose::Pose3d> &pose_3d_with_node_id) {
  return {std::to_string(pose_3d_with_node_id.first),
          std::to_string(pose_3d_with_node_id.second.first.x()),
          std::to_string(pose_3d_with_node_id.second.first.y()),
          std::to_string(pose_3d_with_node_id.second.first.z()),
          std::to_string(pose_3d_with_node_id.second.second.x()),
          std::to_string(pose_3d_with_node_id.second.second.y()),
          std::to_string(pose_3d_with_node_id.second.second.z()),
          std::to_string(pose_3d_with_node_id.second.second.w())};
}

void writePose3dsWithNodeIdToFile(
    const std::string &file_name,
    const std::vector<std::pair<uint64_t, pose::Pose3d>>
        &pose_3ds_with_node_id) {
  std::function<std::vector<std::string>(
      const std::pair<uint64_t, pose::Pose3d> &)>
      object_to_str_list_converter = convertPose3dWithNodeIdToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"node_id",
                               "transl_x",
                               "transl_y",
                               "transl_z",
                               "quat_x",
                               "quat_y",
                               "quat_z",
                               "quat_w"},
                              pose_3ds_with_node_id,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_POSE_3D_WITH_NODE_ID_IO_H
