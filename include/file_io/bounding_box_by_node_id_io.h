//
// Created by amanda on 4/18/22.
//

#ifndef UT_VSLAM_BOUNDING_BOX_BY_NODE_ID_IO_H
#define UT_VSLAM_BOUNDING_BOX_BY_NODE_ID_IO_H

#include <file_io/file_io_utils.h>
#include <boost/algorithm/string.hpp>

#include <cstdint>
#include <string>

namespace file_io {

struct BoundingBoxWithNodeId {
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

  uint64_t node_id;

  /**
   * Id of the camera that captured this boundign box.
   */
  uint64_t camera_id;
};

std::vector<std::string> convertBoundingBoxWithNodeIdToStringList(
    const BoundingBoxWithNodeId &bounding_box) {
  std::string sem_class = bounding_box.semantic_class;
  boost::algorithm::trim(sem_class);
  return {std::to_string(bounding_box.min_pixel_x),
          std::to_string(bounding_box.min_pixel_y),
          std::to_string(bounding_box.max_pixel_x),
          std::to_string(bounding_box.max_pixel_y),
          sem_class,
          std::to_string(bounding_box.node_id),
          std::to_string(bounding_box.camera_id)};
}

void writeBoundingBoxesWithNodeIdToFile(
    const std::string &file_name,
    const std::vector<BoundingBoxWithNodeId> &bounding_boxes) {
  std::function<std::vector<std::string>(const BoundingBoxWithNodeId &)>
      object_to_str_list_converter = convertBoundingBoxWithNodeIdToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"min_pixel_x",
                               "min_pixel_y",
                               "max_pixel_x",
                               "max_pixel_y",
                               "semantic_class",
                               "node_id",
                               "camera_id"},
                              bounding_boxes,
                              object_to_str_list_converter);
}

void readBoundingBoxWithNodeIdLine(
    const std::vector<std::string> &entries_in_file_line,
    BoundingBoxWithNodeId &bounding_box) {
  size_t list_idx = 0;

  bounding_box.min_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.min_pixel_y = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_y = std::stod(entries_in_file_line[list_idx++]);

  bounding_box.semantic_class = entries_in_file_line[list_idx++];
  boost::algorithm::trim(bounding_box.semantic_class);

  std::istringstream node_id_stream(entries_in_file_line[list_idx++]);
  node_id_stream >> bounding_box.node_id;

  std::istringstream cam_id_stream(entries_in_file_line[list_idx++]);
  cam_id_stream >> bounding_box.camera_id;
}

void readBoundingBoxesWithNodeIdFromFile(
    const std::string &file_name,
    std::vector<BoundingBoxWithNodeId> &bounding_boxes) {
  std::function<void(const std::vector<std::string> &, BoundingBoxWithNodeId &)>
      object_from_line_reader = readBoundingBoxWithNodeIdLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, bounding_boxes);
}

struct BoundingBoxWithNodeIdAndId {
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

  uint64_t node_id;

  /**
   * Id of the camera that captured this boundign box.
   */
  uint64_t camera_id;
};

std::vector<std::string> convertBoundingBoxWithNodeIdAndIdToStringList(
    const BoundingBoxWithNodeIdAndId &bounding_box) {
  std::string sem_class = bounding_box.semantic_class;
  boost::algorithm::trim(sem_class);
  return {std::to_string(bounding_box.ellipsoid_idx),
          std::to_string(bounding_box.min_pixel_x),
          std::to_string(bounding_box.min_pixel_y),
          std::to_string(bounding_box.max_pixel_x),
          std::to_string(bounding_box.max_pixel_y),
          sem_class,
          std::to_string(bounding_box.node_id),
          std::to_string(bounding_box.camera_id)};
}

void writeBoundingBoxWithNodeIdAndIdsToFile(
    const std::string &file_name,
    const std::vector<BoundingBoxWithNodeIdAndId> &bounding_boxes) {
  std::function<std::vector<std::string>(const BoundingBoxWithNodeIdAndId &)>
      object_to_str_list_converter =
          convertBoundingBoxWithNodeIdAndIdToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"ellipsoid_idx",
                               "min_pixel_x",
                               "min_pixel_y",
                               "max_pixel_x",
                               "max_pixel_y",
                               "semantic_class",
                               "node_id",
                               "camera_id"},
                              bounding_boxes,
                              object_to_str_list_converter);
}

void readBoundingBoxWithNodeIdAndIdLine(
    const std::vector<std::string> &entries_in_file_line,
    BoundingBoxWithNodeIdAndId &bounding_box) {
  size_t list_idx = 0;
  std::istringstream ellipsoid_idx_stream(entries_in_file_line[list_idx++]);
  ellipsoid_idx_stream >> bounding_box.ellipsoid_idx;

  bounding_box.min_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.min_pixel_y = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_x = std::stod(entries_in_file_line[list_idx++]);
  bounding_box.max_pixel_y = std::stod(entries_in_file_line[list_idx++]);

  bounding_box.semantic_class = entries_in_file_line[list_idx++];
  boost::algorithm::trim(bounding_box.semantic_class);

  std::istringstream node_id_stream(entries_in_file_line[list_idx++]);
  node_id_stream >> bounding_box.node_id;

  std::istringstream cam_id_stream(entries_in_file_line[list_idx++]);
  cam_id_stream >> bounding_box.camera_id;
}

void readBoundingBoxWithNodeIdAndIdsFromFile(
    const std::string &file_name,
    std::vector<BoundingBoxWithNodeIdAndId> &bounding_boxes) {
  std::function<void(const std::vector<std::string> &,
                     BoundingBoxWithNodeIdAndId &)>
      object_from_line_reader = readBoundingBoxWithNodeIdAndIdLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, bounding_boxes);
}

}  // namespace file_io

#endif  // UT_VSLAM_BOUNDING_BOX_BY_NODE_ID_IO_H
