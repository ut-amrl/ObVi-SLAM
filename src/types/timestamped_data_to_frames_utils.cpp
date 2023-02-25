//
// Created by amanda on 2/17/23.
//

#include <types/timestamped_data_to_frames_utils.h>

namespace vslam_types_refactor {

util::BoostHashMap<pose::Timestamp, FrameId> getFramesForRequiredStamps(
    const std::vector<pose::Timestamp> &required_stamps,
    const util::BoostHashMap<pose::Timestamp, FrameId> &poses_by_timestamp) {
  std::vector<std::pair<pose::Timestamp, FrameId>> sorted_nodes_with_stamps;
  for (const auto &pose_and_timestamp : poses_by_timestamp) {
    sorted_nodes_with_stamps.emplace_back(pose_and_timestamp);
  }

  util::BoostHashMap<pose::Timestamp, FrameId> required_frames_by_stamp;
  std::sort(sorted_nodes_with_stamps.begin(),
            sorted_nodes_with_stamps.end(),
            sort_timestamp_and_assoc_data_by_stamp<FrameId>());

  size_t index_next_required_timestamp = 0;
  size_t next_node_and_timestamp = 0;

  bool all_stamps_found = false;
  while ((next_node_and_timestamp < sorted_nodes_with_stamps.size()) &&
         (!all_stamps_found)) {
    pose::Timestamp stamp_for_node =
        sorted_nodes_with_stamps[next_node_and_timestamp].first;
    FrameId node = sorted_nodes_with_stamps[next_node_and_timestamp].second;

    while (pose::timestamp_sort()(
        required_stamps[index_next_required_timestamp], stamp_for_node)) {
      required_frames_by_stamp[required_stamps[index_next_required_timestamp]] =
          node;
      index_next_required_timestamp++;
      if (index_next_required_timestamp >= required_stamps.size()) {
        all_stamps_found = true;
        break;
      }
    }
    next_node_and_timestamp++;
  }

  while (index_next_required_timestamp < required_stamps.size()) {
    required_frames_by_stamp[required_stamps[index_next_required_timestamp]] =
        sorted_nodes_with_stamps.back().second;
    index_next_required_timestamp++;
  }

  return required_frames_by_stamp;
}

}  // namespace vslam_types_refactor