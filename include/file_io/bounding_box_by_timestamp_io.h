//
// Created by amanda on 4/18/22.
//

#ifndef UT_VSLAM_BOUNDING_BOX_BY_TIMESTAMP_IO_H
#define UT_VSLAM_BOUNDING_BOX_BY_TIMESTAMP_IO_H

#include <file_io/file_io_utils.h>

#include <cstdint>
#include <string>

namespace file_io {

const uint64_t kDefaultCameraId = std::numeric_limits<uint64_t>::max();

struct BoundingBoxWithTimestampAndId {
  uint64_t ellipsoid_idx;

  uint64_t min_pixel_x;
  uint64_t min_pixel_y;
  uint64_t max_pixel_x;
  uint64_t max_pixel_y;

  /**
   * Semantic class of the detected bounding box
   *
   * TODO should this instead be an index? Should it store a set of
   * possible semantic classes with their likelihood?
   */
  std::string semantic_class;

  uint32_t seconds;
  uint32_t nano_seconds;

  /**
   * Id of the camera that captured this boundign box.
   */
  uint64_t camera_id;
};

struct BoundingBoxWithTimestamp {
  uint64_t min_pixel_x;
  uint64_t min_pixel_y;
  uint64_t max_pixel_x;
  uint64_t max_pixel_y;

  /**
   * Semantic class of the detected bounding box
   *
   * TODO should this instead be an index? Should it store a set of
   * possible semantic classes with their likelihood?
   */
  std::string semantic_class;

  uint32_t seconds;
  uint32_t nano_seconds;

  /**
   * Id of the camera that captured this boundign box.
   */
  uint64_t camera_id;
};

std::vector<std::string> convertBoundingBoxWithTimestampAndIdToStringList(
    const BoundingBoxWithTimestampAndId &bounding_box) {
  return {std::to_string(bounding_box.ellipsoid_idx),
          std::to_string(bounding_box.min_pixel_x),
          std::to_string(bounding_box.min_pixel_y),
          std::to_string(bounding_box.max_pixel_x),
          std::to_string(bounding_box.max_pixel_y),
          bounding_box.semantic_class,
          std::to_string(bounding_box.seconds),
          std::to_string(bounding_box.nano_seconds),
          std::to_string(bounding_box.camera_id)};
}

std::vector<std::string> convertBoundingBoxWithTimestampToStringList(
    const BoundingBoxWithTimestamp &bounding_box) {
  return {std::to_string(bounding_box.min_pixel_x),
          std::to_string(bounding_box.min_pixel_y),
          std::to_string(bounding_box.max_pixel_x),
          std::to_string(bounding_box.max_pixel_y),
          bounding_box.semantic_class,
          std::to_string(bounding_box.seconds),
          std::to_string(bounding_box.nano_seconds),
          std::to_string(bounding_box.camera_id)};
}

void writeBoundingBoxWithTimestampAndIdsToFile(
    const std::string &file_name,
    const std::vector<BoundingBoxWithTimestampAndId> &bounding_boxes) {
  std::function<std::vector<std::string>(const BoundingBoxWithTimestampAndId &)>
      object_to_str_list_converter =
          convertBoundingBoxWithTimestampAndIdToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"ellipsoid_idx",
                               "min_pixel_x",
                               "min_pixel_y",
                               "max_pixel_x",
                               "max_pixel_y",
                               "semantic_class",
                               "seconds",
                               "nano_seconds",
                               "camera_id"},
                              bounding_boxes,
                              object_to_str_list_converter);
}

void writeBoundingBoxWithTimestampsToFile(
    const std::string &file_name,
    const std::vector<BoundingBoxWithTimestamp> &bounding_boxes) {
  std::function<std::vector<std::string>(const BoundingBoxWithTimestamp &)>
      object_to_str_list_converter =
          convertBoundingBoxWithTimestampToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"min_pixel_x",
                               "min_pixel_y",
                               "max_pixel_x",
                               "max_pixel_y",
                               "semantic_class",
                               "seconds",
                               "nano_seconds",
                               "camera_id"},
                              bounding_boxes,
                              object_to_str_list_converter);
}

void readBoundingBoxWithTimestampAndIdLine(
    const std::vector<std::string> &entries_in_file_line,
    BoundingBoxWithTimestampAndId &bounding_box) {

  size_t list_idx = 0;
  std::istringstream ellipsoid_idx_stream(entries_in_file_line[list_idx++]);
  ellipsoid_idx_stream >> bounding_box.ellipsoid_idx;

  bounding_box.min_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.min_pixel_y = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_y = std::stod(entries_in_file_line[list_idx++]);

  bounding_box.semantic_class = entries_in_file_line[list_idx++];

  std::istringstream seconds_stream(entries_in_file_line[list_idx++]);
  seconds_stream >> bounding_box.seconds;

  std::istringstream nano_seconds_stream(entries_in_file_line[list_idx++]);
  nano_seconds_stream >> bounding_box.nano_seconds;

  std::istringstream cam_id_stream(entries_in_file_line[list_idx++]);
  cam_id_stream >> bounding_box.camera_id;
}

void readBoundingBoxWithTimestampLine(
    const std::vector<std::string> &entries_in_file_line,
    BoundingBoxWithTimestamp &bounding_box) {
  std::string substr;

  size_t list_idx = 0;

  bounding_box.min_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.min_pixel_y = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_y = std::stod(entries_in_file_line[list_idx++]);

  bounding_box.semantic_class = entries_in_file_line[list_idx++];

  std::istringstream seconds_stream(entries_in_file_line[list_idx++]);
  seconds_stream >> bounding_box.seconds;

  std::istringstream nano_seconds_stream(entries_in_file_line[list_idx++]);
  nano_seconds_stream >> bounding_box.nano_seconds;

  bounding_box.camera_id = kDefaultCameraId;
  if (list_idx < entries_in_file_line.size()) {
    std::istringstream cam_id_stream(entries_in_file_line[list_idx++]);
    cam_id_stream >> bounding_box.camera_id;
  }
}

void readBoundingBoxWithTimestampAndIdsFromFile(
    const std::string &file_name,
    std::vector<BoundingBoxWithTimestampAndId> &bounding_boxes) {
  std::function<void(const std::vector<std::string> &,
                     BoundingBoxWithTimestampAndId &)>
      object_from_line_reader = readBoundingBoxWithTimestampAndIdLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, bounding_boxes);
}

void readBoundingBoxWithTimestampsFromFile(
    const std::string &file_name,
    std::vector<BoundingBoxWithTimestamp> &bounding_boxes) {
  std::function<void(const std::vector<std::string> &,
                     BoundingBoxWithTimestamp &)>
      object_from_line_reader = readBoundingBoxWithTimestampLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, bounding_boxes);
}

}  // namespace file_io

#endif  // UT_VSLAM_BOUNDING_BOX_BY_TIMESTAMP_IO_H
