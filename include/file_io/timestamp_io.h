//
// Created by amanda on 6/1/21.
//

#ifndef UT_VSLAM_TIMESTAMP_IO_H
#define UT_VSLAM_TIMESTAMP_IO_H

#include <base_lib/pose_utils.h>
#include <file_io/file_io_utils.h>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace file_io {

void readTimestampLine(const std::vector<std::string> &entries_in_file_line,
                       pose::Timestamp &timestamp) {
  size_t list_idx = 0;

  uint32_t seconds;
  std::istringstream stream_seconds(entries_in_file_line[list_idx++]);
  stream_seconds >> seconds;

  uint32_t nano_seconds;
  std::istringstream stream_nanoseconds(entries_in_file_line[list_idx++]);
  stream_nanoseconds >> nano_seconds;

  timestamp = std::make_pair(seconds, nano_seconds);
}

void readTimestampsFromFile(const std::string &file_name,
                            std::vector<pose::Timestamp> &timestamps) {
  std::function<void(const std::vector<std::string> &, pose::Timestamp &)>
      object_from_line_reader = readTimestampLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, timestamps);
}

std::vector<std::string> convertTimestampToStringList(
    const pose::Timestamp &timestamp) {
  return {std::to_string(timestamp.first), std::to_string(timestamp.second)};
}

void writeTimestampsToFile(const std::string &file_name,
                           const std::vector<pose::Timestamp> &timestamps) {
  std::function<std::vector<std::string>(const pose::Timestamp &)>
      object_to_str_list_converter = convertTimestampToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"seconds", "nanoseconds"},
                              timestamps,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_TIMESTAMP_IO_H
