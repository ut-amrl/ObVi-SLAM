//
// Created by amanda on 6/1/21.
//

#ifndef UT_VSLAM_NODE_ID_AND_TIMESTAMP_IO_H
#define UT_VSLAM_NODE_ID_AND_TIMESTAMP_IO_H

#include <file_io/file_io_utils.h>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace file_io {
struct NodeIdAndTimestamp {
  uint64_t node_id_;
  uint32_t seconds_;
  uint32_t nano_seconds_;
};

void readNodeIdAndTimestampLine(
    const std::vector<std::string> &entries_in_file_line,
    NodeIdAndTimestamp &node_and_timestamp) {
  size_t list_idx = 0;
  std::istringstream stream_identifier(entries_in_file_line[list_idx++]);
  stream_identifier >> node_and_timestamp.node_id_;

  std::istringstream stream_seconds(entries_in_file_line[list_idx++]);
  stream_seconds >> node_and_timestamp.seconds_;

  std::istringstream stream_nanoseconds(entries_in_file_line[list_idx++]);
  stream_nanoseconds >> node_and_timestamp.nano_seconds_;
}

void readNodeIdsAndTimestampsFromFile(
    const std::string &file_name,
    std::vector<NodeIdAndTimestamp> &nodes_and_timestamps) {
  std::function<void(const std::vector<std::string> &, NodeIdAndTimestamp &)>
      object_from_line_reader = readNodeIdAndTimestampLine;
  file_io::readObjectListFromFileWithHeader(
      file_name, object_from_line_reader, nodes_and_timestamps);
}

std::vector<std::string> convertNodeIdWithTimestampToStringList(
    const NodeIdAndTimestamp &node_id_and_timestamp) {
  return {std::to_string(node_id_and_timestamp.node_id_),
          std::to_string(node_id_and_timestamp.seconds_),
          std::to_string(node_id_and_timestamp.nano_seconds_)};
}

void writeNodeIdsAndTimestampsToFile(
    const std::string &file_name,
    const std::vector<NodeIdAndTimestamp> &nodes_with_timestamps) {
  std::function<std::vector<std::string>(const NodeIdAndTimestamp &)>
      object_to_str_list_converter = convertNodeIdWithTimestampToStringList;
  file_io::writeObjectsToFile(file_name,
                              {"node_id", "seconds", "nanoseconds"},
                              nodes_with_timestamps,
                              object_to_str_list_converter);
}

}  // namespace file_io

#endif  // UT_VSLAM_NODE_ID_AND_TIMESTAMP_IO_H
