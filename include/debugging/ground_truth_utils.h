//
// Created by amanda on 7/12/23.
//

#ifndef UT_VSLAM_GROUND_TRUTH_UTILS_H
#define UT_VSLAM_GROUND_TRUTH_UTILS_H
#include <base_lib/basic_utils.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_io.h>
#include <file_io/pose_3d_with_timestamp_io.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {
std::optional<std::vector<Pose3D<double>>> getGtTrajectory(
    const FrameId &max_frame_id,
    const std::string &opt_ground_truth_trajectory_file,
    const std::string &opt_ground_truth_extrinsics_file,
    const std::string &opt_nodes_by_timestamp_file) {
  std::optional<std::vector<Pose3D<double>>> gt_trajectory;
  if ((!opt_ground_truth_trajectory_file.empty()) &&
      (!opt_ground_truth_extrinsics_file.empty())) {
    std::vector<std::pair<pose::Timestamp, Pose3D<double>>>
        raw_pose_3ds_with_timestamp;
    file_io::readPose3dsWithTimestampFromFile(opt_ground_truth_trajectory_file,
                                              raw_pose_3ds_with_timestamp);

    Pose3D<double> gt_traj_frame_rel_base_link;
    std::vector<Pose3D<double>> gt_traj_extrinsics_contents;
    file_io::readPose3dsFromFile(opt_ground_truth_extrinsics_file,
                                 gt_traj_extrinsics_contents);
    if (gt_traj_extrinsics_contents.empty()) {
      LOG(ERROR) << "GT trajectory extrinsics missing";
      exit(1);
    }
    if (gt_traj_extrinsics_contents.size() > 1) {
      LOG(WARNING) << "Coarse trajectory extrinsics file contained more than "
                      "one pose. Taking the first";
    }

    gt_traj_frame_rel_base_link = gt_traj_extrinsics_contents.front();

    std::vector<file_io::NodeIdAndTimestamp> nodes_by_timestamps_vec;
    util::BoostHashMap<pose::Timestamp, vslam_types_refactor::FrameId>
        nodes_for_timestamps_map;
    file_io::readNodeIdsAndTimestampsFromFile(opt_nodes_by_timestamp_file,
                                              nodes_by_timestamps_vec);

    for (const file_io::NodeIdAndTimestamp &raw_node_id_and_timestamp :
         nodes_by_timestamps_vec) {
      nodes_for_timestamps_map[std::make_pair(
          raw_node_id_and_timestamp.seconds_,
          raw_node_id_and_timestamp.nano_seconds_)] =
          raw_node_id_and_timestamp.node_id_;
    }

    std::unordered_map<FrameId, Pose3D<double>> gt_traj_map;

    for (const std::pair<pose::Timestamp, Pose3D<double>> &gt_pose_entry :
         raw_pose_3ds_with_timestamp) {
      if (nodes_for_timestamps_map.find(gt_pose_entry.first) !=
          nodes_for_timestamps_map.end()) {
        gt_traj_map[nodes_for_timestamps_map.at(gt_pose_entry.first)] =
            combinePoses(gt_pose_entry.second,
                         poseInverse(gt_traj_frame_rel_base_link));
      }
    }

    bool valid_gt_traj = true;
    std::vector<Pose3D<double>> gt_traj_under_construction;
    for (FrameId frame_num = 0; frame_num <= max_frame_id; frame_num++) {
      if (gt_traj_map.find(frame_num) == gt_traj_map.end()) {
        LOG(WARNING) << "Could not find GT pose corresponding to frame num "
                     << frame_num << "; not using GT trajectory";
        valid_gt_traj = false;
        break;
      }
      gt_traj_under_construction.emplace_back(gt_traj_map.at(frame_num));
    }
    if (valid_gt_traj) {
      gt_trajectory =
          adjustTrajectoryToStartAtOrigin(gt_traj_under_construction);
    }
  }
  return gt_trajectory;
}
}  // namespace vslam_types_refactor
#endif  // UT_VSLAM_GROUND_TRUTH_UTILS_H
