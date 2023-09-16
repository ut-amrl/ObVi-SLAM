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

DEFINE_string(
    lego_loam_frame_to_bl_extrinsics,
    "",
    "File containing the extrinsic calibration of the frame "
    "that the (pseudo) ground truth outputs are with respect to "
    "relative to the baselink. If the frame for ground truth is the "
    "ouster's frame, for example, this provides the pose of the ouster"
    " relative to the base_link");
DEFINE_string(lego_loam_root_dir, "", "Directory with lego-loam results");
DEFINE_string(lego_loam_out_dir, "", "Directory to write overlayed results to");
DEFINE_string(
    sequence_file,
    "",
    "File containing the sequence of bags to evaluate the output for. In the "
    "comparison_alg_traj_est_dir, there should be a directory for each bag in "
    "the sequence, prefixed with its 0-indexed number in the sequence. If this "
    "is left blank, a single directory corresponding to one trajectory must be "
    "provided.");
DEFINE_string(waypoints_files_directory,
              "",
              "Directory where the waypoints files are stored");
DEFINE_string(rosbag_files_directory,
              "",
              "Directory where the rosbags are stored");
DEFINE_string(odometry_topic, "", "Topic on which odometry is published");
DEFINE_string(param_prefix, "", "Prefix for published topics");

const std::string kWaypointAlignedTrajFileName = "traj_with_waypoints.csv";
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

std::vector<std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>>
annotateWithWaypoints(
    const std::vector<
        std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>>
        &comparison_trajectories_rel_baselink,
    const std::vector<std::string> &ros_bag_names,
    const std::string &odom_topic,
    const std::vector<std::vector<WaypointInfo>> &waypoint_info_by_trajectory,
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

  return raw_consistency_results
      .pose_and_waypoint_info_for_nodes_per_trajectory_;
}

std::vector<std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>>
alignGt(const std::vector<
        std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>>
            &gt_rel_bl) {
  std::vector<
      std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>>
      aligned_trajectories;

  std::vector<std::unordered_map<WaypointId, std::vector<Position3d<double>>>>
      orig_positions_for_wps;

  std::unordered_map<WaypointId, std::vector<Position3d<double>>>
      aligned_waypoint_positions;

  // Collect the waypoints for each bag and store the positions
  for (size_t bag_idx = 0; bag_idx < gt_rel_bl.size(); bag_idx++) {
    std::unordered_map<WaypointId, std::vector<Position3d<double>>>
        waypoints_for_traj;
    for (const std::pair<pose::Timestamp, PoseAndWaypointInfoForNode> &pose :
         gt_rel_bl.at(bag_idx)) {
      if (pose.second.waypoint_id_and_reversal_.has_value()) {
        WaypointId wp_id = pose.second.waypoint_id_and_reversal_.value().first;
        if (waypoints_for_traj.find(wp_id) == waypoints_for_traj.end()) {
          waypoints_for_traj[wp_id] = {};
        }
        waypoints_for_traj[wp_id].emplace_back(pose.second.pose_->transl_);
      }
    }
    orig_positions_for_wps.emplace_back(waypoints_for_traj);
  }

  // Initialize the aligned waypoint positions and aligned trajectories with the
  // first trajectory since we're keeping that one fixed
  aligned_waypoint_positions = orig_positions_for_wps.front();
  aligned_trajectories.emplace_back(gt_rel_bl.front());

  // Align the remaining bags one at a time
  for (size_t bag_idx = 1; bag_idx < gt_rel_bl.size(); bag_idx++) {
    std::unordered_map<WaypointId, std::vector<Position3d<double>>>
        wps_for_traj = orig_positions_for_wps.at(bag_idx);

    // We're aligning the trajectories by trying to find the best match between
    // the centroids of the waypoints between the trajectory to adjust and all
    // waypoint positions from the adjusted trajectories

    // Get the centroids of the waypoints for the new trajectory
    std::unordered_map<WaypointId, Position3d<double>> wp_centroids_for_traj;
    for (const auto &wp_for_traj : wps_for_traj) {
      if (!wp_for_traj.second.empty()) {
        Position3d<double> med_pos(0, 0, 0);
        for (const Position3d<double> &wp_pos : wp_for_traj.second) {
          med_pos += wp_pos;
        }
        med_pos /= wp_for_traj.second.size();
        wp_centroids_for_traj[wp_for_traj.first] = med_pos;
      }
    }

    // Get the centroids of the waypoints for the set of aligned trajectories
    std::unordered_map<WaypointId, Position3d<double>> aligned_centroids;
    for (const auto &aligned_wp_entry : aligned_waypoint_positions) {
      if (!aligned_wp_entry.second.empty()) {
        Position3d<double> med_pos(0, 0, 0);
        for (const Position3d<double> &wp_pos : aligned_wp_entry.second) {
          med_pos += wp_pos;
        }
        med_pos /= aligned_wp_entry.second.size();
        aligned_centroids[aligned_wp_entry.first] = med_pos;
      }
    }

    // Find the transform that aligns them (first need to construct two vectors
    // for the respective clouds with the index providing the association
    std::vector<Position3d<double>> est_assoc_points;
    std::vector<Position3d<double>> fixed_assoc_points;
    for (const auto &wp_centroid : wp_centroids_for_traj) {
      if (aligned_centroids.find(wp_centroid.first) !=
          aligned_centroids.end()) {
        est_assoc_points.emplace_back(wp_centroid.second);
        fixed_assoc_points.emplace_back(
            aligned_centroids.at(wp_centroid.first));
      }
    }

    Pose3D<double> aligning_transform =
        getAligningTransform<double>(fixed_assoc_points, est_assoc_points);

    // Adjust the trajectory using the aligning transform
    std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>
        aligned_traj;
    for (const std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>
             &unaligned_entry : gt_rel_bl.at(bag_idx)) {
      PoseAndWaypointInfoForNode new_info;
      new_info.waypoint_id_and_reversal_ =
          unaligned_entry.second.waypoint_id_and_reversal_;
      new_info.pose_ = combinePoses(aligning_transform,
                                    unaligned_entry.second.pose_.value());
      aligned_traj.emplace_back(
          std::make_pair(unaligned_entry.first, new_info));
    }
    aligned_trajectories.emplace_back(aligned_traj);

    // Adjust the waypoints using the aligning transform
    for (const auto &wp_for_traj : wps_for_traj) {
      if (wp_for_traj.second.empty()) {
        continue;
      }
      if (aligned_waypoint_positions.find(wp_for_traj.first) ==
          aligned_waypoint_positions.end()) {
        aligned_waypoint_positions[wp_for_traj.first] = {};
      }

      for (const Position3d<double> &wp_pos : wp_for_traj.second) {
        aligned_waypoint_positions[wp_for_traj.first].emplace_back(
            combinePoseAndPosition(aligning_transform, wp_pos));
      }
    }
  }
  return aligned_trajectories;
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
  if (FLAGS_lego_loam_frame_to_bl_extrinsics.empty()) {
    LOG(ERROR) << "LeGO-LOAM to baselink extrinsics file must be specified";
    exit(1);
  }
  if (FLAGS_rosbag_files_directory.empty()) {
    LOG(ERROR) << "Rosbags file directory required.";
    exit(1);
  }
  if (FLAGS_odometry_topic.empty()) {
    LOG(ERROR) << "Odometry topic must be specified";
    exit(1);
  }

  if (FLAGS_lego_loam_root_dir.empty()) {
    LOG(ERROR) << "LeGO-LOAM root dir must be specified";
    exit(1);
  }

  if (FLAGS_lego_loam_out_dir.empty()) {
    LOG(ERROR) << "LeGO-LOAM out dir";
    exit(1);
  }
  if (FLAGS_waypoints_files_directory.empty()) {
    LOG(ERROR) << "Waypoints files directory must be specified";
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
  std::vector<std::optional<std::string>> waypoint_file_base_names;
  std::vector<std::string> output_dirs;
  std::vector<std::string> rosbag_names;
  for (size_t bag_idx = 0;
       bag_idx < sequence_info.bag_base_names_and_waypoint_files.size();
       bag_idx++) {
    indiv_traj_dir_base_names.emplace_back(
        (std::to_string(bag_idx) + "_" +
         sequence_info.bag_base_names_and_waypoint_files[bag_idx]
             .bag_base_name_));
    std::string rosbag_base_name =
        sequence_info.bag_base_names_and_waypoint_files[bag_idx].bag_base_name_;
    rosbag_names.emplace_back(file_io::ensureDirectoryPathEndsWithSlash(
                                  FLAGS_rosbag_files_directory) +
                              rosbag_base_name + file_io::kBagExtension);
    waypoint_file_base_names.emplace_back(
        sequence_info.bag_base_names_and_waypoint_files[bag_idx]
            .optional_waypoint_file_base_name_);
    std::string out_dir_name =
        file_io::ensureDirectoryPathEndsWithSlash(FLAGS_lego_loam_out_dir) +
        std::to_string(bag_idx) + "_" + rosbag_base_name;

    output_dirs.emplace_back(out_dir_name);

    std::string orig_traj_name =
        file_io::ensureDirectoryPathEndsWithSlash(FLAGS_lego_loam_root_dir) +
        rosbag_base_name + kOrigTrajPathWithinDir;
    orig_lego_loam_traj_names.emplace_back(orig_traj_name);
  }

  for (const std::string &out_dir_name : output_dirs) {
    file_io::makeDirectoryIfDoesNotExist(out_dir_name);
  }

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

  // Read in all LeGO-LOAM trajectories
  // Read in waypoints
  // Apply extrinsics
  // Display

  // TODO maybe?
  // Align by waypoint estimates -- hold first traj constant

  std::vector<std::string> full_paths_for_wp_annotated_trajectories;
  for (const std::string &indiv_traj_dir_base_name :
       indiv_traj_dir_base_names) {
    std::string wp_annotated_out_file_full_path =
        file_io::ensureDirectoryPathEndsWithSlash(
            file_io::ensureDirectoryPathEndsWithSlash(FLAGS_lego_loam_out_dir) +
            indiv_traj_dir_base_name) +
        kWaypointAlignedTrajFileName;
    full_paths_for_wp_annotated_trajectories.emplace_back(
        wp_annotated_out_file_full_path);
  }

  Pose3D<double> lego_loam_frame_rel_base_link;
  std::vector<Pose3D<double>> lego_loam_extrinsics_contents;
  file_io::readPose3dsFromFile(FLAGS_lego_loam_frame_to_bl_extrinsics,
                               lego_loam_extrinsics_contents);
  if (lego_loam_extrinsics_contents.empty()) {
    LOG(ERROR) << "LeGO-LOAM extrinsics missing";
    exit(1);
  }
  if (lego_loam_extrinsics_contents.size() > 1) {
    LOG(WARNING) << "Lego loam extrinsics file contained more than one pose. "
                    "Taking the first";
  }
  lego_loam_frame_rel_base_link = lego_loam_extrinsics_contents.front();

  // Read in LeGO-LOAM trajectories
  // For each, transform using extrinsics
  // For each, shift first pose to origin and transform rest of trajectory
  // accordingly
  std::vector<
      std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>>
      gt_rel_bl;

  for (const std::vector<std::pair<pose::Timestamp, Pose3D<double>>> &gt_traj :
       orig_poses_orig_frame) {
    std::vector<std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>
        interp_gt_traj_rel_bl;
    std::vector<Pose3D<double>> poses;

    for (const std::pair<pose::Timestamp, Pose3D<double>> &orig_gt_pos :
         gt_traj) {
      poses.emplace_back(orig_gt_pos.second);
    }
    std::vector<Pose3D<double>> frame_adjusted =
        adjustTrajectoryToStartAtOriginWithExtrinsics<double>(
            poses, poses.front(), lego_loam_frame_rel_base_link);

    for (size_t pose_num = 0; pose_num < poses.size(); pose_num++) {
      interp_gt_traj_rel_bl.emplace_back(std::make_pair(
          gt_traj.at(pose_num).first, frame_adjusted.at(pose_num)));
    }

    gt_rel_bl.emplace_back(interp_gt_traj_rel_bl);
  }

  if (vis_manager != nullptr) {
    std::vector<std::vector<Pose3D<double>>> gt_init;

    for (size_t traj_num = 0; traj_num < gt_rel_bl.size(); traj_num++) {
      const std::vector<
          std::pair<pose::Timestamp, std::optional<Pose3D<double>>>>
          traj_est = gt_rel_bl.at(traj_num);
      std::vector<Pose3D<double>> traj_valid;

      for (size_t pose_num = 0; pose_num < traj_est.size(); pose_num++) {
        const std::pair<pose::Timestamp, std::optional<Pose3D<double>>>
            old_entry = traj_est.at(pose_num);
        traj_valid.emplace_back(old_entry.second.value());
      }

      gt_init.emplace_back(traj_valid);
    }
    vis_manager->visualizeTrajectories(gt_init, INITIAL);
  }

  std::vector<std::vector<WaypointInfo>> waypoint_info_by_trajectory;

  for (const std::optional<std::string> &waypoint_file_base_name :
       waypoint_file_base_names) {
    if (waypoint_file_base_name.has_value()) {
      std::string full_waypoint_file_name =
          file_io::ensureDirectoryPathEndsWithSlash(
              FLAGS_waypoints_files_directory) +
          waypoint_file_base_name.value() + file_io::kCsvExtension;
      std::vector<WaypointInfo> waypoint_info_for_traj;
      file_io::readWaypointInfosFromFile(full_waypoint_file_name,
                                         waypoint_info_for_traj);
      waypoint_info_by_trajectory.emplace_back(waypoint_info_for_traj);
    } else {
      waypoint_info_by_trajectory.emplace_back((std::vector<WaypointInfo>){});
    }
  }

  std::vector<
      std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>>
      unaligned_annotated = annotateWithWaypoints(gt_rel_bl,
                                                  rosbag_names,
                                                  FLAGS_odometry_topic,
                                                  waypoint_info_by_trajectory,
                                                  vis_manager);

  std::vector<
      std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>>
      aligned_annotated = alignGt(unaligned_annotated);

  if (vis_manager != nullptr) {
    std::vector<std::vector<Pose3D<double>>> est_traj;
    std::unordered_map<WaypointId, std::vector<std::optional<Pose3D<double>>>>
        waypoints;

    for (size_t traj_num = 0; traj_num < aligned_annotated.size(); traj_num++) {
      const std::vector<std::pair<pose::Timestamp, PoseAndWaypointInfoForNode>>
          traj_est = aligned_annotated.at(traj_num);
      std::vector<Pose3D<double>> traj_valid;

      for (size_t pose_num = 0; pose_num < traj_est.size(); pose_num++) {
        const std::pair<pose::Timestamp, PoseAndWaypointInfoForNode> old_entry =
            traj_est.at(pose_num);
        traj_valid.emplace_back(old_entry.second.pose_.value());
        if (old_entry.second.waypoint_id_and_reversal_.has_value()) {
          WaypointId wp_id =
              old_entry.second.waypoint_id_and_reversal_.value().first;
          if (waypoints.find(wp_id) == waypoints.end()) {
            waypoints[wp_id] = {};
          }
          waypoints[wp_id].emplace_back(old_entry.second.pose_.value());
        }
      }

      est_traj.emplace_back(traj_valid);
    }
    //    vis_manager->visualizeWaypoints(waypoints);
    vis_manager->visualizeTrajectories(est_traj, ESTIMATED);
  }

  //  for (size_t bag_idx = 0; bag_idx < aligned_annotated.size(); bag_idx++) {
  //    file_io::writePose3dWsithWaypointInfoToFile(
  //        full_paths_for_wp_annotated_trajectories.at(bag_idx),
  //        aligned_annotated.at(bag_idx));
  //  }
  return 0;
}