//
// Created by amanda on 7/18/23.
//

#ifndef UT_VSLAM_POSE_IO_UTILS_H
#define UT_VSLAM_POSE_IO_UTILS_H

#include <file_io/pose_3d_with_node_id_io.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

namespace file_io {
std::unordered_map<vslam_types_refactor::FrameId,
                   vslam_types_refactor::Pose3D<double>>
readRobotPosesFromFile(const std::string &file_name) {
  std::vector<std::pair<uint64_t, pose::Pose3d>> robot_poses_by_node_id;
  file_io::readPose3dsAndNodeIdFromFile(file_name, robot_poses_by_node_id);
  std::unordered_map<vslam_types_refactor::FrameId,
                     vslam_types_refactor::Pose3D<double>>
      robot_poses_by_node_num;
  for (const std::pair<uint64_t, pose::Pose3d> &pose_3d_with_frame :
       robot_poses_by_node_id) {
    vslam_types_refactor::Pose3D<double> pose(
        pose_3d_with_frame.second.first,
        vslam_types_refactor::Orientation3D<double>(
            pose_3d_with_frame.second.second));
    robot_poses_by_node_num[pose_3d_with_frame.first] = pose;
  }
  return robot_poses_by_node_num;
}
}  // namespace file_io

#endif  // UT_VSLAM_POSE_IO_UTILS_H
