//
// Created by amanda on 2/19/23.
//

#ifndef UT_VSLAM_SEQUENCE_UTILS_H
#define UT_VSLAM_SEQUENCE_UTILS_H
#include <string>
#include <vector>
#include <optional>

namespace vslam_types_refactor {

typedef std::optional<std::string> OptionalWaypointFileName;

struct BagBaseNameAndWaypointFile {
  std::string bag_base_name_;
  OptionalWaypointFileName optional_waypoint_file_base_name_;

  BagBaseNameAndWaypointFile() = default;
  BagBaseNameAndWaypointFile(
      const std::string &bag_base_name,
      const std::optional<std::string> &optional_waypoint_file_base_name)
      : bag_base_name_(bag_base_name),
        optional_waypoint_file_base_name_(optional_waypoint_file_base_name) {}
};

struct SequenceInfo {
  std::string sequence_id_;
  std::vector<BagBaseNameAndWaypointFile> bag_base_names_and_waypoint_files;
};
}  // namespace vslam_types_refactor
#endif  // UT_VSLAM_SEQUENCE_UTILS_H
