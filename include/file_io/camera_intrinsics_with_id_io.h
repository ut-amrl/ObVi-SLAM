//
// Created by amanda on 6/1/21.
//

#ifndef UT_VSLAM_CAMERA_INTRINSICS_WITH_ID_IO_H
#define UT_VSLAM_CAMERA_INTRINSICS_WITH_ID_IO_H

#include <file_io/file_io_utils.h>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace file_io {
struct CameraIntrinsicsWithId {
  uint64_t camera_id;
  uint64_t img_width;   // x dim
  uint64_t img_height;  // y dim
  double mat_00;
  double mat_01;
  double mat_02;
  double mat_10;
  double mat_11;
  double mat_12;
  double mat_20;
  double mat_21;
  double mat_22;
};

void readCameraIntrinsicsWithIdLine(
    const std::vector<std::string> &entries_in_file_line,
    CameraIntrinsicsWithId &camera_intrinsics_with_id) {
  size_t list_idx = 0;
  std::istringstream stream_cam_id(entries_in_file_line[list_idx++]);
  stream_cam_id >> camera_intrinsics_with_id.camera_id;

  std::istringstream stream_img_width(entries_in_file_line[list_idx++]);
  stream_img_width >> camera_intrinsics_with_id.img_width;

  std::istringstream stream_img_height(entries_in_file_line[list_idx++]);
  stream_img_height >> camera_intrinsics_with_id.img_height;

  camera_intrinsics_with_id.mat_00 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_01 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_02 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_10 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_11 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_12 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_20 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_21 =
      std::stod(entries_in_file_line[list_idx++]);
  camera_intrinsics_with_id.mat_22 =
      std::stod(entries_in_file_line[list_idx++]);
}

void readCameraIntrinsicsWithIdsFromFile(
    const std::string &file_name,
    std::vector<CameraIntrinsicsWithId> &camera_intrinsics_with_ids) {
  std::function<void(const std::vector<std::string> &,
                     CameraIntrinsicsWithId &)>
      object_from_line_reader = readCameraIntrinsicsWithIdLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, camera_intrinsics_with_ids);
}

std::vector<std::string> convertCameraIntrinsicsWithIdToStringList(
    const CameraIntrinsicsWithId &camera_intrinsics_with_id) {
  return {std::to_string(camera_intrinsics_with_id.camera_id),
          std::to_string(camera_intrinsics_with_id.img_width),
          std::to_string(camera_intrinsics_with_id.img_height),
          std::to_string(camera_intrinsics_with_id.mat_00),
          std::to_string(camera_intrinsics_with_id.mat_01),
          std::to_string(camera_intrinsics_with_id.mat_02),
          std::to_string(camera_intrinsics_with_id.mat_10),
          std::to_string(camera_intrinsics_with_id.mat_11),
          std::to_string(camera_intrinsics_with_id.mat_12),
          std::to_string(camera_intrinsics_with_id.mat_20),
          std::to_string(camera_intrinsics_with_id.mat_21),
          std::to_string(camera_intrinsics_with_id.mat_22)};
}

void writeCameraIntrinsicsWithIdsToFile(
    const std::string &file_name,
    const std::vector<CameraIntrinsicsWithId> &camera_intrinsics_with_ids) {
  std::function<std::vector<std::string>(const CameraIntrinsicsWithId &)>
      object_to_str_list_converter = convertCameraIntrinsicsWithIdToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"camera_id",
                               "img_width",
                               "img_height",
                               "mat_00",
                               "mat_01",
                               "mat_02",
                               "mat_10",
                               "mat_11",
                               "mat_12",
                               "mat_20",
                               "mat_21",
                               "mat_22"},
                              camera_intrinsics_with_ids,
                              object_to_str_list_converter);
}
}  // namespace file_io

#endif  // UT_VSLAM_CAMERA_INTRINSICS_WITH_ID_IO_H
