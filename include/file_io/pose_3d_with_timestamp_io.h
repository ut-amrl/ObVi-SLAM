//
// Created by amanda on 2/5/22.
//

#ifndef UT_VSLAM_POSE_3D_WITH_TIMESTAMP_IO_H
#define UT_VSLAM_POSE_3D_WITH_TIMESTAMP_IO_H

#include <base_lib/pose_reps.h>
#include <base_lib/pose_utils.h>
#include <file_io/file_io_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

#include <fstream>

namespace file_io {

void readPose3dWithTimestampLine(
    const std::vector<std::string> &entries_in_file_line,
    std::pair<pose::Timestamp, pose::Pose3d> &pose3d_with_timestamp) {
  size_t list_idx = 0;

  uint32_t seconds;
  uint32_t nano_seconds;

  std::istringstream stream_seconds(entries_in_file_line[list_idx++]);
  stream_seconds >> seconds;

  std::istringstream stream_nseconds(entries_in_file_line[list_idx++]);
  stream_nseconds >> nano_seconds;

  std::vector<double> data;
  for (size_t i = 0; i < 7; i++) {
    data.emplace_back(std::stod(entries_in_file_line[list_idx++]));
  }

  pose::Pose3d pose_3d =
      std::make_pair(Eigen::Vector3d(data[0], data[1], data[2]),
                     Eigen::Quaterniond(data[6], data[3], data[4], data[5]));

  pose3d_with_timestamp =
      std::make_pair(std::make_pair(seconds, nano_seconds), pose_3d);
}

void readOptionalPose3dWithTimestampLine(
    const std::vector<std::string> &entries_in_file_line,
    std::pair<pose::Timestamp,
              std::optional<vslam_types_refactor::Pose3D<double>>>
        &pose3d_with_timestamp) {
  size_t list_idx = 0;

  uint32_t seconds;
  uint32_t nano_seconds;

  std::istringstream stream_seconds(entries_in_file_line[list_idx++]);
  stream_seconds >> seconds;

  std::istringstream stream_nseconds(entries_in_file_line[list_idx++]);
  stream_nseconds >> nano_seconds;

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

    pose3d_with_timestamp =
        std::make_pair(std::make_pair(seconds, nano_seconds), pose_3d);
  } else {
    pose3d_with_timestamp =
        std::make_pair(std::make_pair(seconds, nano_seconds), std::nullopt);
  }
}

void readPose3dsAndTimestampsFromFile(
    const std::string &file_name,
    std::vector<std::pair<pose::Timestamp, pose::Pose3d>>
        &pose_3ds_with_timestamp) {
  std::function<void(const std::vector<std::string> &,
                     std::pair<pose::Timestamp, pose::Pose3d> &)>
      object_from_line_reader = readPose3dWithTimestampLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, pose_3ds_with_timestamp);
}

std::vector<std::string> convertPose3dWithTimestampToStringList(
    const std::pair<pose::Timestamp, pose::Pose3d> &pose_3d_with_timestamp) {
  return {std::to_string(pose_3d_with_timestamp.first.first),
          std::to_string(pose_3d_with_timestamp.first.second),
          std::to_string(pose_3d_with_timestamp.second.first.x()),
          std::to_string(pose_3d_with_timestamp.second.first.y()),
          std::to_string(pose_3d_with_timestamp.second.first.z()),
          std::to_string(pose_3d_with_timestamp.second.second.x()),
          std::to_string(pose_3d_with_timestamp.second.second.y()),
          std::to_string(pose_3d_with_timestamp.second.second.z()),
          std::to_string(pose_3d_with_timestamp.second.second.w())};
}

std::vector<std::string> convertOptionalPose3dWithTimestampToStringList(
    const std::pair<pose::Timestamp,
                    std::optional<vslam_types_refactor::Pose3D<double>>>
        &pose_3d_with_timestamp) {
  vslam_types_refactor::Position3d<double> pos_to_write;
  Eigen::Quaterniond quat_to_write;
  int lost_int;
  if (pose_3d_with_timestamp.second.has_value()) {
    lost_int = 0;
    pos_to_write = pose_3d_with_timestamp.second.value().transl_;
    quat_to_write =
        Eigen::Quaterniond(pose_3d_with_timestamp.second.value().orientation_);
  } else {
    lost_int = 1;
  }

  return {std::to_string(pose_3d_with_timestamp.first.first),
          std::to_string(pose_3d_with_timestamp.first.second),
          std::to_string(lost_int),
          std::to_string(pos_to_write.x()),
          std::to_string(pos_to_write.y()),
          std::to_string(pos_to_write.z()),
          std::to_string(quat_to_write.x()),
          std::to_string(quat_to_write.y()),
          std::to_string(quat_to_write.z()),
          std::to_string(quat_to_write.w())};
}

void writePose3dsWithTimestampToFile(
    const std::string &file_name,
    const std::vector<std::pair<pose::Timestamp, pose::Pose3d>>
        &pose_3ds_with_timestamp) {
  std::function<std::vector<std::string>(
      const std::pair<pose::Timestamp, pose::Pose3d> &)>
      object_to_str_list_converter = convertPose3dWithTimestampToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"seconds",
                               "nanoseconds",
                               "transl_x",
                               "transl_y",
                               "transl_z",
                               "quat_x",
                               "quat_y",
                               "quat_z",
                               "quat_w"},
                              pose_3ds_with_timestamp,
                              object_to_str_list_converter);
}

void writePose3dsWithTimestampToFile(
    const std::string &file_name,
    const std::vector<
        std::pair<pose::Timestamp, vslam_types_refactor::Pose3D<double>>>
        &pose_3ds_with_timestamp) {
  std::vector<std::pair<pose::Timestamp, pose::Pose3d>>
      old_pose_3ds_with_timestamp;
  for (const std::pair<pose::Timestamp, vslam_types_refactor::Pose3D<double>>
           &new_pose : pose_3ds_with_timestamp) {
    old_pose_3ds_with_timestamp.emplace_back(std::make_pair(
        new_pose.first,
        std::make_pair(new_pose.second.transl_,
                       Eigen::Quaterniond(new_pose.second.orientation_))));
  }
  writePose3dsWithTimestampToFile(file_name, old_pose_3ds_with_timestamp);
}

void readPose3dsWithTimestampFromFile(
    const std::string &file_name,
    std::vector<
        std::pair<pose::Timestamp, vslam_types_refactor::Pose3D<double>>>
        &pose_3ds_with_timestamp) {
  std::vector<std::pair<pose::Timestamp, pose::Pose3d>>
      old_pose_3ds_with_timestamp;
  readPose3dsAndTimestampsFromFile(file_name, old_pose_3ds_with_timestamp);
  for (const std::pair<pose::Timestamp, pose::Pose3d> &old_pose :
       old_pose_3ds_with_timestamp) {
    pose_3ds_with_timestamp.emplace_back(
        std::make_pair(old_pose.first,
                       vslam_types_refactor::Pose3D<double>(
                           old_pose.second.first,
                           vslam_types_refactor::Orientation3D<double>(
                               old_pose.second.second))));
  }
}

void readOptionalPose3dsWithTimestampFromFile(
    const std::string &file_name,
    std::vector<std::pair<pose::Timestamp,
                          std::optional<vslam_types_refactor::Pose3D<double>>>>
        &optional_pose_3ds_with_timestamp) {
  std::function<void(
      const std::vector<std::string> &,
      std::pair<pose::Timestamp,
                std::optional<vslam_types_refactor::Pose3D<double>>> &)>
      object_from_line_reader = readOptionalPose3dWithTimestampLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, optional_pose_3ds_with_timestamp);
}

void writeOptionalPose3dsWithTimestampToFile(
    const std::string &file_name,
    const std::vector<
        std::pair<pose::Timestamp,
                  std::optional<vslam_types_refactor::Pose3D<double>>>>
        &optional_pose_3ds_with_timestamp) {
  std::function<std::vector<std::string>(
      const std::pair<pose::Timestamp,
                      std::optional<vslam_types_refactor::Pose3D<double>>> &)>
      object_to_str_list_converter =
          convertOptionalPose3dWithTimestampToStringList;
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
                               "quat_w"},
                              optional_pose_3ds_with_timestamp,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_POSE_3D_WITH_TIMESTAMP_IO_H
