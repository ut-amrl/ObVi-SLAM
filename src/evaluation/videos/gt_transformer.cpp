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

DEFINE_string(interpolated_traj, "", "");
DEFINE_string(
    lego_loam_frame_to_bl_extrinsics,
    "",
    "File containing the extrinsic calibration of the frame "
    "that the (pseudo) ground truth outputs are with respect to "
    "relative to the baselink. If the frame for ground truth is the "
    "ouster's frame, for example, this provides the pose of the ouster"
    " relative to the base_link");
DEFINE_string(interp_out_file, "", "");

FullSequenceMetrics computeMetrics(
    const std::vector<
        std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>>
        &comparison_trajectories_rel_baselink,
    const std::vector<std::vector<std::pair<pose::Timestamp, Pose3D<double>>>>
        &interp_gt_trajectories_rel_baselink,
    const std::vector<std::string> &ros_bag_names,
    const std::string &odom_topic,
    const std::vector<std::vector<WaypointInfo>> &waypoint_info_by_trajectory,
    const std::vector<std::string> &traj_with_waypoints_files,
    const std::shared_ptr<vslam_types_refactor::RosVisualization> &vis_manager =
        nullptr) {
  FullSequenceMetrics full_metrics;
  std::vector<ATEResults> single_traj_ate_results;

  std::vector<
      util::BoostHashMap<pose::Timestamp, std::optional<Pose3D<double>>>>
      poses_by_timestamp_by_trajectory;
  for (size_t traj_num = 0;
       traj_num < comparison_trajectories_rel_baselink.size();
       traj_num++) {
    util::BoostHashMap<pose::Timestamp, std::optional<Pose3D<double>>>
        poses_by_timestamp;
    for (const std::pair<pose::Timestamp, std::optional<Pose3D<double>>>
             &timestamp_and_pose :
         comparison_trajectories_rel_baselink.at(traj_num)) {
      poses_by_timestamp[timestamp_and_pose.first] = timestamp_and_pose.second;
    }
    poses_by_timestamp_by_trajectory.emplace_back(poses_by_timestamp);
  }

  // Assumes odom is for base_link
  std::vector<std::vector<std::pair<Timestamp, Pose2d>>>
      odom_poses_by_trajectory;
  for (const std::string &rosbag_name : ros_bag_names) {
    std::vector<std::pair<Timestamp, Pose2d>> odom_poses_for_bag;
    getOdomPoseEsts(rosbag_name, odom_topic, odom_poses_for_bag);
    odom_poses_by_trajectory.emplace_back(odom_poses_for_bag);
  }

  RawWaypointConsistencyResults raw_consistency_results =
      computeWaypointConsistencyResults(waypoint_info_by_trajectory,
                                        comparison_trajectories_rel_baselink,
                                        poses_by_timestamp_by_trajectory,
                                        odom_poses_by_trajectory,
                                        vis_manager);

  CHECK(raw_consistency_results.pose_and_waypoint_info_for_nodes_per_trajectory_
            .size() == traj_with_waypoints_files.size())
      << "Full trajectory with waypoint annotations structure must match "
         "number of files to output such information to.";
  for (size_t traj_num = 0; traj_num < traj_with_waypoints_files.size();
       traj_num++) {
    std::string traj_with_wps_name = traj_with_waypoints_files.at(traj_num);
    if (traj_with_wps_name.empty()) {
      continue;
    }
    std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>
        pose_and_waypoint_info_for_nodes_for_trajectory =
            raw_consistency_results
                .pose_and_waypoint_info_for_nodes_per_trajectory_.at(traj_num);

    file_io::writePose3dWsithWaypointInfoToFile(
        traj_with_wps_name, pose_and_waypoint_info_for_nodes_for_trajectory);
  }

  for (size_t traj_num = 0;
       traj_num < comparison_trajectories_rel_baselink.size();
       traj_num++) {
    std::vector<std::pair<pose::Timestamp, Pose3D<double>>> gt_rel_bl_traj =
        interp_gt_trajectories_rel_baselink[traj_num];
    std::vector<Pose3D<double>> gt_pose_only;
    for (const std::pair<pose::Timestamp, Pose3D<double>> &gt_entry :
         gt_rel_bl_traj) {
      gt_pose_only.emplace_back(gt_entry.second);
    }

    std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>
        comparison_traj_rel_bl = comparison_trajectories_rel_baselink[traj_num];
    std::vector<std::optional<Pose3D<double>>> comparison_pose_only;
    for (const std::pair<pose::Timestamp, std::optional<Pose3D<double>>>
             &comparison_entry : comparison_traj_rel_bl) {
      comparison_pose_only.emplace_back(comparison_entry.second);
    }

    std::vector<std::optional<Pose3D<double>>> aligned_comparison_pose;
    alignWithGroundTruth(
        gt_pose_only, comparison_pose_only, aligned_comparison_pose);

    ATEResults traj_ate_results =
        generateATEforRotAndTranslForSyncedAlignedTrajectories(
            aligned_comparison_pose, gt_pose_only);
    TrajectoryMetrics single_traj_metrics;
    single_traj_metrics.ate_results_ = traj_ate_results;
    single_traj_ate_results.emplace_back(traj_ate_results);

    for (const auto &waypoint_with_centroid_dev_by_trajectory :
         raw_consistency_results
             .centroid_deviations_by_waypoint_by_trajectory_) {
      WaypointId waypoint = waypoint_with_centroid_dev_by_trajectory.first;
      std::vector<double> centroid_devs_for_waypoint_for_trajectory =
          waypoint_with_centroid_dev_by_trajectory.second.at(traj_num);
      std::vector<double> orientation_devs_for_trajectory =
          raw_consistency_results
              .orientation_deviations_by_waypoint_by_trajectory_.at(waypoint)
              .at(traj_num);
      single_traj_metrics.waypoint_deviations_[waypoint] =
          std::make_pair(centroid_devs_for_waypoint_for_trajectory,
                         orientation_devs_for_trajectory);
      single_traj_metrics.all_rotation_deviations_.insert(
          single_traj_metrics.all_rotation_deviations_.end(),
          orientation_devs_for_trajectory.begin(),
          orientation_devs_for_trajectory.end());
      single_traj_metrics.all_translation_deviations_.insert(
          single_traj_metrics.all_translation_deviations_.end(),
          centroid_devs_for_waypoint_for_trajectory.begin(),
          centroid_devs_for_waypoint_for_trajectory.end());
    }

    full_metrics.indiv_trajectory_metrics_.emplace_back(single_traj_metrics);
  }

  TrajectoryMetrics sequence_results;
  sequence_results.ate_results_ =
      combineSingleTrajectoryResults(single_traj_ate_results);
  for (const TrajectoryMetrics &single_traj_metrics :
       full_metrics.indiv_trajectory_metrics_) {
    sequence_results.all_translation_deviations_.insert(
        sequence_results.all_translation_deviations_.end(),
        single_traj_metrics.all_translation_deviations_.begin(),
        single_traj_metrics.all_translation_deviations_.end());
    sequence_results.all_rotation_deviations_.insert(
        sequence_results.all_rotation_deviations_.end(),
        single_traj_metrics.all_rotation_deviations_.begin(),
        single_traj_metrics.all_rotation_deviations_.end());

    for (const auto &waypoint_and_devs :
         single_traj_metrics.waypoint_deviations_) {
      std::pair<std::vector<double>, std::vector<double>> all_waypoint_devs;
      if (sequence_results.waypoint_deviations_.find(waypoint_and_devs.first) !=
          sequence_results.waypoint_deviations_.end()) {
        all_waypoint_devs =
            sequence_results.waypoint_deviations_.at(waypoint_and_devs.first);
      } else {
        all_waypoint_devs =
            std::make_pair((std::vector<double>){}, (std::vector<double>){});
      }
      std::vector<double> centroid_devs_for_waypoint = all_waypoint_devs.first;
      std::vector<double> orientation_devs_for_waypoint =
          all_waypoint_devs.second;
      centroid_devs_for_waypoint.insert(centroid_devs_for_waypoint.end(),
                                        waypoint_and_devs.second.first.begin(),
                                        waypoint_and_devs.second.first.end());
      orientation_devs_for_waypoint.insert(
          orientation_devs_for_waypoint.end(),
          waypoint_and_devs.second.second.begin(),
          waypoint_and_devs.second.second.end());
      sequence_results.waypoint_deviations_[waypoint_and_devs.first] =
          std::make_pair(centroid_devs_for_waypoint,
                         orientation_devs_for_waypoint);
    }
  }

  full_metrics.sequence_metrics_ = sequence_results;
  return full_metrics;
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;



  ros::init(argc, argv, "gt_transformer");
  ros::NodeHandle node_handle;


  std::vector<std::pair<pose::Timestamp, Pose3D<double>>>
      raw_pose_3ds_with_timestamp;
  file_io::readPose3dsWithTimestampFromFile(FLAGS_interpolated_traj,
                                            raw_pose_3ds_with_timestamp);

  Pose3D<double> gt_traj_frame_rel_base_link;
  std::vector<Pose3D<double>> gt_traj_extrinsics_contents;
  file_io::readPose3dsFromFile(FLAGS_lego_loam_frame_to_bl_extrinsics,
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

  //    std::unordered_map<FrameId, Pose3D<double>> gt_traj_map;
  std::vector<Pose3D<double>> gt_traj_calib;

  for (const std::pair<pose::Timestamp, Pose3D<double>> &gt_pose_entry :
       raw_pose_3ds_with_timestamp) {
    gt_traj_calib.emplace_back(combinePoses(
        gt_pose_entry.second, poseInverse(gt_traj_frame_rel_base_link)));
  }
  std::vector<Pose3D<double>> gt_trajectory;
  LOG(INFO) << "Calibrated but not adjusted ";
  LOG(INFO) << gt_traj_calib.front().transl_;

  gt_trajectory = adjustTrajectoryToStartAtOrigin(gt_traj_calib);


  LOG(INFO) << "Calibrated but not adjusted wihtout time ";
  LOG(INFO) << gt_trajectory.front().transl_;

  std::vector<std::pair<pose::Timestamp, Pose3D<double>>>
      adjusted_raw_pose_3ds_with_timestamp;
  for (FrameId idx = 0; idx < gt_trajectory.size(); idx++) {
    pose::Timestamp time = raw_pose_3ds_with_timestamp.at(idx).first;
    Pose3D<double> pose = gt_trajectory.at(idx);

    adjusted_raw_pose_3ds_with_timestamp.emplace_back(std::make_pair(time, pose));
  }


  LOG(INFO) << "Calibrated and not adjusted ";
  LOG(INFO) << adjusted_raw_pose_3ds_with_timestamp.front().second.transl_;
  file_io::writePose3dsWithTimestampToFile(FLAGS_interp_out_file, adjusted_raw_pose_3ds_with_timestamp);
}