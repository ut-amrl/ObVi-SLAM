//
// Created by amanda on 6/1/21.
//

#ifndef UT_VSLAM_CAMERA_EXTRINSICS_WITH_ID_IO_H
#define UT_VSLAM_CAMERA_EXTRINSICS_WITH_ID_IO_H

#include <file_io/file_io_utils.h>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace file_io {
struct CameraExtrinsicsWithId {
  uint64_t camera_id;
  double transl_x;
  double transl_y;
  double transl_z;
  double quat_x;
  double quat_y;
  double quat_z;
  double quat_w;
};

void readCameraExtrinsicsWithIdLine(
    const std::vector<std::string> &entries_in_file_line,
    CameraExtrinsicsWithId &camera_extrinsics_with_id) {
  size_t list_idx = 0;
  std::istringstream stream_cam_id(entries_in_file_line[list_idx++]);
  stream_cam_id >> camera_extrinsics_with_id.camera_id;

  camera_extrinsics_with_id.transl_x =
      std::stod(entries_in_file_line[list_idx++]);
  camera_extrinsics_with_id.transl_y =
      std::stod(entries_in_file_line[list_idx++]);
  camera_extrinsics_with_id.transl_z =
      std::stod(entries_in_file_line[list_idx++]);
  camera_extrinsics_with_id.quat_x =
      std::stod(entries_in_file_line[list_idx++]);
  camera_extrinsics_with_id.quat_y =
      std::stod(entries_in_file_line[list_idx++]);
  camera_extrinsics_with_id.quat_z =
      std::stod(entries_in_file_line[list_idx++]);
  camera_extrinsics_with_id.quat_w =
      std::stod(entries_in_file_line[list_idx++]);
}

void readCameraExtrinsicsWithIdsFromFile(
    const std::string &file_name,
    std::vector<CameraExtrinsicsWithId> &camera_extrinsics_with_ids) {
  std::function<void(const std::vector<std::string> &,
                     CameraExtrinsicsWithId &)>
      object_from_line_reader = readCameraExtrinsicsWithIdLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, camera_extrinsics_with_ids);
}

std::vector<std::string> convertCameraExtrinsicsWithIdToStringList(
    const CameraExtrinsicsWithId &camera_extrinsics_with_id) {
  return {std::to_string(camera_extrinsics_with_id.camera_id),
          std::to_string(camera_extrinsics_with_id.transl_x),
          std::to_string(camera_extrinsics_with_id.transl_y),
          std::to_string(camera_extrinsics_with_id.transl_z),
          std::to_string(camera_extrinsics_with_id.quat_x),
          std::to_string(camera_extrinsics_with_id.quat_y),
          std::to_string(camera_extrinsics_with_id.quat_z),
          std::to_string(camera_extrinsics_with_id.quat_w)};
}

void writeCameraExtrinsicsWithIdsToFile(
    const std::string &file_name,
    const std::vector<CameraExtrinsicsWithId> &camera_extrinsics_with_ids) {
  std::function<std::vector<std::string>(const CameraExtrinsicsWithId &)>
      object_to_str_list_converter = convertCameraExtrinsicsWithIdToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"camera_id",
                               "transl_x",
                               "transl_y",
                               "transl_z",
                               "quat_x",
                               "quat_y",
                               "quat_z",
                               "quat_w"},
                              camera_extrinsics_with_ids,
                              object_to_str_list_converter);
}
}  // namespace file_io

#endif  // UT_VSLAM_CAMERA_EXTRINSICS_WITH_ID_IO_H
