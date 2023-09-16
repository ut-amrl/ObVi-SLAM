//
// Created by amanda on 2/5/22.
//

#ifndef UT_VSLAM_POSE_3D_WITH_TIMESTAMP_AND_WAYPOINT_ANNOTATION_IO_H
#define UT_VSLAM_POSE_3D_WITH_TIMESTAMP_AND_WAYPOINT_ANNOTATION_IO_H

#include <base_lib/pose_reps.h>
#include <base_lib/pose_utils.h>
#include <evaluation/trajectory_evaluation_utils.h>
#include <file_io/file_io_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

#include <fstream>

namespace file_io {

namespace {
static const size_t kWaypointIdIdx = 10;
}

void readPose3dWithWaypointInfoLine(
    const std::vector<std::string> &entries_in_file_line,
    std::pair<pose::Timestamp, vslam_types_refactor::PoseAndWaypointInfoForNode>
        &stamp_and_pose_info) {
  size_t list_idx = 0;

  uint32_t seconds;
  uint32_t nano_seconds;

  std::istringstream stream_seconds(entries_in_file_line[list_idx++]);
  stream_seconds >> seconds;

  std::istringstream stream_nseconds(entries_in_file_line[list_idx++]);
  stream_nseconds >> nano_seconds;

  vslam_types_refactor::PoseAndWaypointInfoForNode pose_info;

  bool lost = std::stoi(entries_in_file_line[list_idx++]) != 0;
  if (!lost) {
    std::vector<double> data;
    for (size_t i = 0; i < 7; i++) {
      data.emplace_back(std::stod(entries_in_file_line[list_idx++]));
    }

    vslam_types_refactor::Pose3D<double> pose_3d(
        vslam_types_refactor::Position3d<double>(data[0], data[1], data[2]),
        vslam_types_refactor::Orientation3D<double>(
            Eigen::Quaterniond(data[6], data[3], data[4], data[5])));
    pose_info.pose_ = pose_3d;
  } else {
    pose_info.pose_ = std::nullopt;
  }

  int waypoint_id = std::stoi(entries_in_file_line[kWaypointIdIdx]);
  if (waypoint_id < 0) {
    pose_info.waypoint_id_and_reversal_ = std::nullopt;
  } else {
    bool waypoint_reversed =
        std::stoi(entries_in_file_line[kWaypointIdIdx + 1]) != 0;
    pose_info.waypoint_id_and_reversal_ =
        std::make_pair(waypoint_id, waypoint_reversed);
  }

  stamp_and_pose_info =
      std::make_pair(std::make_pair(seconds, nano_seconds), pose_info);
}

void readPose3dsWithWaypointInfoFromFile(
    const std::string &file_name,
    std::vector<std::pair<pose::Timestamp,
                          vslam_types_refactor::PoseAndWaypointInfoForNode>>
        &stamp_and_pose_infos) {
  std::function<void(
      const std::vector<std::string> &,
      std::pair<pose::Timestamp,
                vslam_types_refactor::PoseAndWaypointInfoForNode> &)>
      object_from_line_reader = readPose3dWithWaypointInfoLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, stamp_and_pose_infos);
}

std::vector<std::string> convertPose3dWithWaypointInfoToStringList(
    const std::pair<pose::Timestamp,
                    vslam_types_refactor::PoseAndWaypointInfoForNode>
        &stamp_and_pose_info) {
  vslam_types_refactor::Position3d<double> pos_to_write;
  Eigen::Quaterniond quat_to_write;
  int lost_int;
  if (stamp_and_pose_info.second.pose_.has_value()) {
    lost_int = 0;
    pos_to_write = stamp_and_pose_info.second.pose_.value().transl_;
    quat_to_write = Eigen::Quaterniond(
        stamp_and_pose_info.second.pose_.value().orientation_);
  } else {
    lost_int = 1;
  }
  int reversed_int = 0;
  int waypoint_id = -1;
  if (stamp_and_pose_info.second.waypoint_id_and_reversal_.has_value()) {
    waypoint_id =
        stamp_and_pose_info.second.waypoint_id_and_reversal_.value().first;
    if (stamp_and_pose_info.second.waypoint_id_and_reversal_.value().second) {
      reversed_int = 1;
    }
  }

  return {std::to_string(stamp_and_pose_info.first.first),
          std::to_string(stamp_and_pose_info.first.second),
          std::to_string(lost_int),
          std::to_string(pos_to_write.x()),
          std::to_string(pos_to_write.y()),
          std::to_string(pos_to_write.z()),
          std::to_string(quat_to_write.x()),
          std::to_string(quat_to_write.y()),
          std::to_string(quat_to_write.z()),
          std::to_string(quat_to_write.w()),
          std::to_string(waypoint_id),
          std::to_string(reversed_int)};
}

void writePose3dWsithWaypointInfoToFile(
    const std::string &file_name,
    const std::vector<
        std::pair<pose::Timestamp,
                  vslam_types_refactor::PoseAndWaypointInfoForNode>>
        &stamp_and_pose_infos) {
  std::function<std::vector<std::string>(
      const std::pair<pose::Timestamp,
                      vslam_types_refactor::PoseAndWaypointInfoForNode> &)>
      object_to_str_list_converter = convertPose3dWithWaypointInfoToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"seconds",
                               "nanoseconds",
                               "lost",
                               "transl_x",
                               "transl_y",
                               "transl_z",
                               "quat_x",
                               "quat_y",
                               "quat_z",
                               "quat_w",
                               "waypoint_id",
                               "waypoint_reversed"},
                              stamp_and_pose_infos,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_POSE_3D_WITH_TIMESTAMP_AND_WAYPOINT_ANNOTATION_IO_H
