//
// Created by amanda on 2/17/23.
//

#ifndef UT_VSLAM_TIMESTAMPED_DATA_TO_FRAMES_UTILS_H
#define UT_VSLAM_TIMESTAMPED_DATA_TO_FRAMES_UTILS_H

#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

namespace vslam_types_refactor {
typedef int WaypointId;

struct sort_timestamp_and_node_by_stamp {
  inline bool operator()(
      const std::pair<pose::Timestamp, FrameId> &timestamp_and_node_id1,
      const std::pair<pose::Timestamp, FrameId> &timestamp_and_node_id2) {
    return pose::timestamp_sort()(timestamp_and_node_id1.first,
                                  timestamp_and_node_id2.first);
  }
};

struct WaypointInfo {
  WaypointId waypoint_id_;
  pose::Timestamp waypoint_timestamp_;
};

struct AssociatedWaypointInfo {
  WaypointId waypoint_id_;
  FrameId associated_frame_id_;
};

util::BoostHashMap<pose::Timestamp, FrameId> getFramesForRequiredStamps(
    const std::vector<pose::Timestamp> &required_stamps,
    const util::BoostHashMap<pose::Timestamp, FrameId> &poses_by_timestamp);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_TIMESTAMPED_DATA_TO_FRAMES_UTILS_H
