//
// Created by amanda on 6/1/21.
//

#ifndef UT_VSLAM_TIMESTAMP_AND_WAYPOINT_IO_H
#define UT_VSLAM_TIMESTAMP_AND_WAYPOINT_IO_H

#include <file_io/file_io_utils.h>
#include <types/timestamped_data_to_frames_utils.h>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace file_io {
struct TimestampAndWaypointInfo {
  uint32_t seconds_;
  uint32_t nano_seconds_;
  vslam_types_refactor::WaypointId waypoint_id_;
  bool reversed_;
};

void readTimestampAndWaypointInfoLine(
    const std::vector<std::string> &entries_in_file_line,
    TimestampAndWaypointInfo &timestamp_and_waypoint_info) {
  size_t list_idx = 0;

  std::istringstream stream_seconds(entries_in_file_line[list_idx++]);
  stream_seconds >> timestamp_and_waypoint_info.seconds_;

  std::istringstream stream_nanoseconds(entries_in_file_line[list_idx++]);
  stream_nanoseconds >> timestamp_and_waypoint_info.nano_seconds_;

  timestamp_and_waypoint_info.waypoint_id_ =
      std::stoi(entries_in_file_line[list_idx++]);
  int reversed_int = std::stoi(entries_in_file_line[list_idx++]);
  timestamp_and_waypoint_info.reversed_ = reversed_int != 0;
}

void readTimestampAndWaypointInfosFromFile(
    const std::string &file_name,
    std::vector<TimestampAndWaypointInfo> &nodes_and_timestamps) {
  std::function<void(const std::vector<std::string> &,
                     TimestampAndWaypointInfo &)>
      object_from_line_reader = readTimestampAndWaypointInfoLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, nodes_and_timestamps);
}

std::vector<std::string> convertTimestampAndWaypointInfoToStringList(
    const TimestampAndWaypointInfo &timestamp_and_waypoint_info) {
  int reversed_int = timestamp_and_waypoint_info.reversed_ ? 1 : 0;
  return {std::to_string(timestamp_and_waypoint_info.seconds_),
          std::to_string(timestamp_and_waypoint_info.nano_seconds_),
          std::to_string(timestamp_and_waypoint_info.waypoint_id_),
          std::to_string(reversed_int)};
}

void writeTimestampAndWaypointInfosToFile(
    const std::string &file_name,
    const std::vector<TimestampAndWaypointInfo>
        &timestamps_and_waypoint_infos) {
  std::function<std::vector<std::string>(const TimestampAndWaypointInfo &)>
      object_to_str_list_converter =
          convertTimestampAndWaypointInfoToStringList;
  file_io::writeObjectsToFile(
      file_name,
      {"seconds", "nanoseconds", "waypoint_id", "reversed"},
      timestamps_and_waypoint_infos,
      object_to_str_list_converter);
}

void readWaypointInfosFromFile(
    const std::string &waypoint_file_name,
    std::vector<vslam_types_refactor::WaypointInfo> &waypoint_infos) {
  std::vector<TimestampAndWaypointInfo> raw_infos;
  readTimestampAndWaypointInfosFromFile(waypoint_file_name, raw_infos);

  for (const TimestampAndWaypointInfo &raw_info : raw_infos) {
    vslam_types_refactor::WaypointInfo waypoint_info;
    waypoint_info.waypoint_timestamp_ =
        std::make_pair(raw_info.seconds_, raw_info.nano_seconds_);
    waypoint_info.reversed_ = raw_info.reversed_;
    waypoint_info.waypoint_id_ = raw_info.waypoint_id_;
    waypoint_infos.emplace_back(waypoint_info);
  }
}

}  // namespace file_io

#endif  // UT_VSLAM_TIMESTAMP_AND_WAYPOINT_IO_H
