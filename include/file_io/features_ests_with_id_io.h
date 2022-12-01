//
// Created by amanda on 2/5/22.
//

#ifndef UT_VSLAM_FEATURE_EST_WITH_IO_H
#define UT_VSLAM_FEATURE_EST_WITH_IO_H

#include <file_io/file_io_utils.h>

#include <fstream>

namespace file_io {

struct FeatureEstWithId {
  uint64_t feature_id;
  double x;
  double y;
  double z;
};

void readFeatureEstWithIdLine(
    const std::vector<std::string> &entries_in_file_line,
    FeatureEstWithId &feature_est_with_id) {
  size_t list_idx = 0;
  std::istringstream stream_feature_id(entries_in_file_line[list_idx++]);
  stream_feature_id >> feature_est_with_id.feature_id;

  std::vector<double> data;
  for (size_t i = 0; i < 3; i++) {
    data.emplace_back(std::stod(entries_in_file_line[list_idx++]));
  }
  feature_est_with_id.x = data[0];
  feature_est_with_id.y = data[1];
  feature_est_with_id.z = data[2];
}

void readFeatureEstsWithIdFromFile(
    const std::string &file_name,
    std::vector<FeatureEstWithId> &feature_ests_with_node_id) {
  std::function<void(const std::vector<std::string> &, FeatureEstWithId &)>
      feature_est_from_line_reader = readFeatureEstWithIdLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, feature_est_from_line_reader, feature_ests_with_node_id);
}

std::vector<std::string> convertFeatureEstWithIdToStringList(
    const FeatureEstWithId &feature_est_with_id) {
  return {std::to_string(feature_est_with_id.feature_id),
          std::to_string(feature_est_with_id.x),
          std::to_string(feature_est_with_id.y),
          std::to_string(feature_est_with_id.z)};
}

void writeFeatureEstsWithIdToFile(
    const std::string &file_name,
    const std::vector<FeatureEstWithId> &feature_ests_with_ids) {
  std::function<std::vector<std::string>(const FeatureEstWithId &)>
      object_to_str_list_converter = convertFeatureEstWithIdToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"feat_id", "x", "y", "z"},
                              feature_ests_with_ids,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_FEATURE_EST_WITH_IO_H
