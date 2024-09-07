//
// Created by amanda on 2/19/23.
//

#include <base_lib/pose_utils.h>
#include <evaluation/trajectory_evaluation_utils.h>
#include <evaluation/trajectory_interpolation_utils.h>
#include <file_io/cv_file_storage/full_sequence_metrics_file_storage_io.h>
#include <file_io/cv_file_storage/sequence_file_storage_io.h>
#include <file_io/file_access_utils.h>
#include <file_io/pose_3d_io.h>
#include <file_io/pose_3d_with_double_timestamp_io.h>
#include <file_io/pose_3d_with_timestamp_and_waypoint_annotation_io.h>
#include <file_io/pose_3d_with_timestamp_io.h>
#include <file_io/timestamp_and_waypoint_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/types/vslam_types_math_util.h>
#include <refactoring/visualization/ros_visualization.h>
using namespace vslam_types_refactor;
using namespace pose;

DEFINE_string(lego_loam_root_dir, "", "Directory with lego-loam results");
DEFINE_string(
    sequence_file,
    "",
    "File containing the sequence of bags to evaluate the output for. In the "
    "comparison_alg_traj_est_dir, there should be a directory for each bag in "
    "the sequence, prefixed with its 0-indexed number in the sequence. If this "
    "is left blank, a single directory corresponding to one trajectory must be "
    "provided.");
DEFINE_string(param_prefix, "", "param_prefix");

const std::string kOrigTrajPathWithinDir = "/poses/lego_loam_poses.csv";

struct sort_pose3d_timestamp_pair {
  inline bool operator()(const std::pair<Timestamp, Pose3D<double>> &pose_1,
                         const std::pair<Timestamp, Pose3D<double>> &pose_2) {
    return timestamp_sort()(pose_1.first, pose_2.first);
  }
};

void convertToStampAndPose3D(
    const std::vector<file_io::Pose3DWithDoubleTimestamp> &coarse_fixed_poses,
    std::vector<std::pair<Timestamp, Pose3D<double>>>
        &output_poses_with_timestamps) {
  for (const file_io::Pose3DWithDoubleTimestamp &input_pose :
       coarse_fixed_poses) {
    ros::Time node_time(input_pose.timestamp_);
    output_poses_with_timestamps.emplace_back(std::make_pair(
        std::make_pair(node_time.sec, node_time.nsec),
        Pose3D<double>(
            Position3d<double>(input_pose.transl_x_,
                               input_pose.transl_y_,
                               input_pose.transl_z_),
            Orientation3D<double>(Eigen::Quaterniond(input_pose.quat_w_,
                                                     input_pose.quat_x_,
                                                     input_pose.quat_y_,
                                                     input_pose.quat_z_)))));
  }
  std::sort(output_poses_with_timestamps.begin(),
            output_poses_with_timestamps.end(),
            sort_pose3d_timestamp_pair());
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  ros::init(argc, argv, "a_" + node_prefix + "lego_loam_overlay_generator");
  ros::NodeHandle node_handle;
  std::shared_ptr<RosVisualization> vis_manager =
      std::make_shared<RosVisualization>(
          node_handle, param_prefix, node_prefix);
  ros::Duration(2).sleep();

  if (FLAGS_sequence_file.empty()) {
    LOG(ERROR) << "Sequence file must be specified.";
    exit(1);
  }

  if (FLAGS_lego_loam_root_dir.empty()) {
    LOG(ERROR) << "LeGO-LOAM root dir must be specified";
    exit(1);
  }

  SequenceInfo sequence_info;
  readSequenceInfo(FLAGS_sequence_file, sequence_info);
  if (sequence_info.bag_base_names_and_waypoint_files.empty()) {
    LOG(ERROR) << "No bag names in sequence";
    exit(1);
  }

  std::vector<std::string> indiv_traj_dir_base_names;
  std::vector<std::string> orig_lego_loam_traj_names;

  LOG(INFO) << "Getting info from sequence file";
  for (size_t bag_idx = 0;
       bag_idx < sequence_info.bag_base_names_and_waypoint_files.size();
       bag_idx++) {
    indiv_traj_dir_base_names.emplace_back(
        (std::to_string(bag_idx) + "_" +
         sequence_info.bag_base_names_and_waypoint_files[bag_idx]
             .bag_base_name_));
    std::string rosbag_base_name =
        sequence_info.bag_base_names_and_waypoint_files[bag_idx].bag_base_name_;

    std::string orig_traj_name =
        file_io::ensureDirectoryPathEndsWithSlash(FLAGS_lego_loam_root_dir) +
        rosbag_base_name + kOrigTrajPathWithinDir;
    orig_lego_loam_traj_names.emplace_back(orig_traj_name);
  }

  LOG(INFO) << "Reading poses";
  std::vector<std::vector<std::pair<Timestamp, Pose3D<double>>>>
      orig_poses_orig_frame;
  for (const std::string &orig_traj_name : orig_lego_loam_traj_names) {
    std::vector<file_io::Pose3DWithDoubleTimestamp> coarse_fixed_poses_raw;
    readPose3DWithDoubleTimestampFromFile(orig_traj_name,
                                          coarse_fixed_poses_raw);

    std::vector<std::pair<Timestamp, Pose3D<double>>> coarse_fixed_poses;
    convertToStampAndPose3D(coarse_fixed_poses_raw, coarse_fixed_poses);
    std::vector<Pose3D<double>> coarse_fixed_poses_no_stamp;
    for (const std::pair<Timestamp, Pose3D<double>> &fixed_pose :
         coarse_fixed_poses) {
      coarse_fixed_poses_no_stamp.emplace_back(fixed_pose.second);
    }
    orig_poses_orig_frame.emplace_back(coarse_fixed_poses);
  }

  LOG(INFO) << "Computing total translation for each bag";

  double all_bags_translation = 0;
  for (size_t bag_idx = 0; bag_idx < orig_lego_loam_traj_names.size();
       bag_idx++) {
    std::string traj_name = indiv_traj_dir_base_names.at(bag_idx);

    double aggregate_translation = 0;

    std::vector<std::pair<Timestamp, Pose3D<double>>> traj =
        orig_poses_orig_frame.at(bag_idx);
    for (size_t pose_idx = 1; pose_idx < traj.size(); pose_idx++) {
      aggregate_translation +=
          getPose2RelativeToPose1(traj.at(pose_idx - 1).second,
                                  traj.at(pose_idx).second)
              .transl_.norm();
    }

    LOG(INFO) << "Aggregate translation for bag " + traj_name + ": " << aggregate_translation;
    all_bags_translation += aggregate_translation;
  }
  LOG(INFO) << "Cumulative translation " << all_bags_translation;

  return 0;
}