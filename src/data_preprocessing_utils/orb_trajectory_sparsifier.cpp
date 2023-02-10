//
// Created by amanda on 12/9/22.
//

#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <file_io/features_ests_with_id_io.h>
#include <file_io/file_access_utils.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/configuration/full_ov_slam_config.h>
#include <refactoring/types/vslam_types_math_util.h>
#include <ros/ros.h>

#include <experimental/filesystem>

namespace vtr = vslam_types_refactor;
namespace fs = std::experimental::filesystem;

DEFINE_string(param_prefix, "", "Rosnode parameter/node prefix");

DEFINE_string(
    input_processed_data_path,
    "",
    "Directory that contains trajectory information to sparsify. Should"
    " have the contents that will be there after running unproject_main");

DEFINE_string(
    output_processed_data_path,
    "",
    "Directory that contains trajectory information to sparsify. Should"
    " have the contents that will be there after running unproject_main");
DEFINE_string(params_config_file, "", "config file containing tunable params");

namespace {
const std::string kFeaturesFolderPath = "features/";
const std::string kFeaturesFile = "features.txt";
// const std::string kDepthsFolder = "depths/";
// const std::string kVelocitiesFolder = "velocities/";
const std::string kPosesFolder = "poses/";
const std::string kRobotPosesFile = "initial_robot_poses_by_node.txt";
const std::string kNodesWithTimestampsFolder = "timestamps/";
const std::string kNodesWithTimestampsFile = "node_ids_and_timestamps.txt";
}  // namespace

void copyAndUpdateFeatureObsFileWithNewFrameNum(
    const std::string &old_feature_obs_file_name,
    const std::string &new_feature_obs_dir,
    const vtr::FrameId &new_frame_id) {
  std::string output_file_name =
      new_feature_obs_dir + std::to_string(new_frame_id) + ".txt";
  std::ifstream old_file_obj(old_feature_obs_file_name);
  std::ofstream output_file(output_file_name, std::ios::trunc);
  bool first_line = true;
  std::string line;
  while (std::getline(old_file_obj, line)) {
    if (first_line) {
      first_line = false;
      output_file << std::to_string(new_frame_id) << "\n";
      continue;
    }
    output_file << line << "\n";
  }
  if (first_line) {
    LOG(ERROR)
        << "The file was completely empty (and likely doesn't exist). File "
        << old_feature_obs_file_name;
    exit(1);
  }
}

void updateFeatureObsWithNewFrameId(
    const std::string &input_feature_obs_dir,
    const std::string &output_feature_obs_dir,
    const std::unordered_map<vtr::FrameId, vtr::FrameId>
        &old_frame_to_new_frame_mapping) {
  for (const auto &entry :
       fs::directory_iterator(fs::path(input_feature_obs_dir))) {
    const auto file_extension = entry.path().extension().string();
    if (!is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::string full_file_name = entry.path().string();
    std::string base_file_name =
        full_file_name.substr(full_file_name.find_last_of("/\\") + 1);
    std::string::size_type const lastPeriod(base_file_name.find_last_of('.'));
    base_file_name = base_file_name.substr(0, lastPeriod);

    vtr::FrameId file_frame_id;
    std::istringstream filename_str_stream(base_file_name);
    filename_str_stream >> file_frame_id;

    if (old_frame_to_new_frame_mapping.find(file_frame_id) !=
        old_frame_to_new_frame_mapping.end()) {
      copyAndUpdateFeatureObsFileWithNewFrameNum(
          full_file_name,
          output_feature_obs_dir,
          old_frame_to_new_frame_mapping.at(file_frame_id));
    }
  }
}

std::unordered_map<vtr::FrameId, vtr::FrameId> getSparsifiedFrames(
    const std::vector<std::pair<vtr::FrameId, pose::Pose3d>> &poses,
    const double &max_pose_inc_threshold_transl,
    const double &max_pose_inc_threshold_rot) {
  std::unordered_map<vtr::FrameId, vtr::FrameId> old_frame_to_new_frame_mapping;
  old_frame_to_new_frame_mapping[poses.front().first] = 0;
  vtr::FrameId next_new_frame_id = 1;
  pose::Pose3d last_added_pose = poses.front().second;

  for (size_t poseIdx = 1; poseIdx < poses.size(); poseIdx++) {
    std::pair<vtr::FrameId, pose::Pose3d> next_candidate_pose = poses[poseIdx];
    pose::Pose3d relative_since_last_pose = pose::getPoseOfObj1RelToObj2(
        next_candidate_pose.second, last_added_pose);

    if ((relative_since_last_pose.first.norm() >
         max_pose_inc_threshold_transl) ||
        (Eigen::AngleAxisd(relative_since_last_pose.second).angle() >
         max_pose_inc_threshold_rot)) {
      old_frame_to_new_frame_mapping[next_candidate_pose.first] =
          next_new_frame_id;
      next_new_frame_id++;
      last_added_pose = next_candidate_pose.second;
    }
  }
  if (old_frame_to_new_frame_mapping.find(poses.back().first) ==
      old_frame_to_new_frame_mapping.end()) {
    old_frame_to_new_frame_mapping[poses.back().first] = next_new_frame_id;
  }
  return old_frame_to_new_frame_mapping;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }
  if (FLAGS_params_config_file.empty()) {
    LOG(ERROR) << "Configuration file required ";
    exit(1);
  }

  vtr::FullOVSLAMConfig config;
  vtr::readConfiguration(FLAGS_params_config_file, config);

  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, node_prefix + "orb_trajectory_sparsifier");
  ros::NodeHandle n;

  std::string input_data_path = file_io::ensureDirectoryPathEndsWithSlash(
      FLAGS_input_processed_data_path);
  std::string output_data_path = file_io::ensureDirectoryPathEndsWithSlash(
      FLAGS_output_processed_data_path);

  std::string old_poses_file = input_data_path + kPosesFolder + kRobotPosesFile;
  std::string new_poses_file =
      output_data_path + kPosesFolder + kRobotPosesFile;

  std::string old_timestamps_file =
      input_data_path + kNodesWithTimestampsFolder + kNodesWithTimestampsFile;
  std::string new_timestamps_file =
      output_data_path + kNodesWithTimestampsFolder + kNodesWithTimestampsFile;

  std::string old_feature_ests_file =
      input_data_path + kFeaturesFolderPath + kFeaturesFile;
  std::string new_feature_ests_file =
      output_data_path + kFeaturesFolderPath + kFeaturesFile;

  file_io::makeDirectoryIfDoesNotExist(output_data_path + kFeaturesFolderPath);
  file_io::makeDirectoryIfDoesNotExist(output_data_path +
                                       kNodesWithTimestampsFolder);
  file_io::makeDirectoryIfDoesNotExist(output_data_path + kPosesFolder);

  std::vector<std::pair<vtr::FrameId, pose::Pose3d>> input_poses_vector;
  file_io::readPose3dsAndNodeIdFromFile(old_poses_file, input_poses_vector);

  std::unordered_map<vtr::FrameId, vtr::FrameId>
      old_frame_to_new_frame_mapping = getSparsifiedFrames(
          input_poses_vector,
          config.sparsifier_params_.max_pose_inc_threshold_transl_,
          config.sparsifier_params_.max_pose_inc_threshold_rot_);

  std::vector<std::pair<uint64_t, pose::Pose3d>> new_poses_vector;
  for (const std::pair<vtr::FrameId, pose::Pose3d> &old_frame_id_pose_pair :
       input_poses_vector) {
    if (old_frame_to_new_frame_mapping.find(old_frame_id_pose_pair.first) !=
        old_frame_to_new_frame_mapping.end()) {
      new_poses_vector.emplace_back(std::make_pair(
          old_frame_to_new_frame_mapping.at(old_frame_id_pose_pair.first),
          old_frame_id_pose_pair.second));
    }
  }
  file_io::writePose3dsWithNodeIdToFile(new_poses_file, new_poses_vector);

  std::vector<file_io::FeatureEstWithId> input_feature_ests;
  file_io::readFeatureEstsWithIdFromFile(old_feature_ests_file,
                                         input_feature_ests);
  file_io::writeFeatureEstsWithIdToFile(new_feature_ests_file,
                                        input_feature_ests);

  std::vector<file_io::NodeIdAndTimestamp> old_node_ids_with_timestamps;
  file_io::readNodeIdsAndTimestampsFromFile(old_timestamps_file,
                                            old_node_ids_with_timestamps);
  std::vector<file_io::NodeIdAndTimestamp> new_node_ids_with_timestamps;
  for (const file_io::NodeIdAndTimestamp &old_node_id_and_timestamp :
       old_node_ids_with_timestamps) {
    if (old_frame_to_new_frame_mapping.find(
            old_node_id_and_timestamp.node_id_) !=
        old_frame_to_new_frame_mapping.end()) {
      file_io::NodeIdAndTimestamp new_node_id_and_timestamp =
          old_node_id_and_timestamp;
      new_node_id_and_timestamp.node_id_ =
          old_frame_to_new_frame_mapping.at(old_node_id_and_timestamp.node_id_);
      new_node_ids_with_timestamps.emplace_back(new_node_id_and_timestamp);
    }
  }
  file_io::writeNodeIdsAndTimestampsToFile(new_timestamps_file,
                                           new_node_ids_with_timestamps);

  updateFeatureObsWithNewFrameId(
      input_data_path, output_data_path, old_frame_to_new_frame_mapping);

  // TODO maybe later:
  // Generate new depths file? (not strictly necessary)
  // Generate new velocities file (again, not strictly necessary)
}