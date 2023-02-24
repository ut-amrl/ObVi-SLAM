//
// Created by amanda on 2/19/23.
//

#include <base_lib/pose_utils.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_timestamp_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/output_problem_data.h>

DEFINE_string(poses_by_frame_file,
              "",
              "File name containing the poses stored with their frame id.");
DEFINE_string(
    frames_for_timestamps_file,
    "",
    "File name containing the frame id for each timestamp of interest");
DEFINE_string(poses_by_timestamp_out_file,
              "",
              "File name to which to output the poses stored by the timestamp");

using namespace vslam_types_refactor;

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  if (FLAGS_poses_by_frame_file.empty()) {
    LOG(ERROR) << "Poses by frame file required";
    exit(1);
  }
  if (FLAGS_frames_for_timestamps_file.empty()) {
    LOG(ERROR) << "frames_for_timestamps_file required";
    exit(1);
  }
  if (FLAGS_poses_by_timestamp_out_file.empty()) {
    LOG(ERROR) << "poses_by_timestamp_out_file required";
    exit(1);
  }

  // Read in the files
  std::vector<file_io::NodeIdAndTimestamp> node_ids_with_timestamps;
  file_io::readNodeIdsAndTimestampsFromFile(FLAGS_frames_for_timestamps_file,
                                            node_ids_with_timestamps);

  RobotPoseResults robot_pose_results;
  readRobotPoseResults(FLAGS_poses_by_frame_file, robot_pose_results);

  std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>
      poses_for_required_timestamps;

  for (const file_io::NodeIdAndTimestamp &node_id_and_timestamp :
       node_ids_with_timestamps) {
    if (robot_pose_results.robot_poses_.find(node_id_and_timestamp.node_id_) ==
        robot_pose_results.robot_poses_.end()) {
      LOG(ERROR) << "Could not find pose for node "
                 << node_id_and_timestamp.node_id_
                 << " corresponding to timestamp "
                 << node_id_and_timestamp.seconds_ << ", "
                 << node_id_and_timestamp.nano_seconds_;
      // Not just adding into list with lost because we don't have the notion of
      // lost so this would represent a bug somewhere.
      continue;
    }
    poses_for_required_timestamps.emplace_back(std::make_pair(
        std::make_pair(node_id_and_timestamp.seconds_,
                       node_id_and_timestamp.nano_seconds_),
        robot_pose_results.robot_poses_.at(node_id_and_timestamp.node_id_)));
  }

  file_io::writeOptionalPose3dsWithTimestampToFile(
      FLAGS_poses_by_timestamp_out_file, poses_for_required_timestamps);
}