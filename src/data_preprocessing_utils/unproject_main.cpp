#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/features_ests_with_id_io.h>
#include <file_io/file_access_utils.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/types/vslam_types_math_util.h>

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// using namespace std;
// using namespace std::experimental::filesystem;
// using namespace Eigen;
// using namespace vslam_types;
// using namespace vslam_unproject;
// using namespace vslam_io;
// using namespace google;

namespace vtr = vslam_types_refactor;
namespace fs = std::experimental::filesystem;

DEFINE_string(
    raw_data_path,
    "",
    "\nPath to folder containing velocities and timestamps (folders with each "
    "of these names should be present in this path). The velocities folder "
    "contains a file for each frame except the first with the initial estimate "
    "of the frame relative to the previous frame. Files are named "
    "<frame_num>.txt and the first line contains the frame number and the "
    "second line is the pose of the primary camera relative to the primary "
    "camera's pose in the previous frame. The timestamps folder should contain "
    "a file named node_ids_and_timestamps.txt. The first line has the column "
    "headers and remaining lines have 3 comma-separated values with frame_id, "
    "seconds, nanoseconds. All data in this path contains non-zero-adjusted "
    "frame numbers, so this script adjusts all of them so that the minimum "
    "frame number is 0.");

DEFINE_string(
    calibration_path,
    "",
    "Path to folder containing the calibration data. Should contain a file "
    "named camera_matrix.txt (each line has the format <camera_id fx fy cx "
    "cy>) and another file named extrinsics.txt (each line has format "
    "<camera_id tx ty tz qx qy qz qw>, where the pose values are the camera's "
    "position with respect to the frame to estimate, i.e. baselink). The "
    "primary camera id (what the velocities in the raw path are relative to) "
    "should occur first in both files.");

DEFINE_string(
    processed_data_path,
    "",
    "Path to folder containing feature detections and depth data for each "
    "frame. Frame number adjusted timestamps and initial estimates for robot "
    "poses and feature points are written to this directory as well. Feature "
    "detection files are in the root of this directory and are named <frame "
    "id>.txt. Each feature detection file starts with the frame number in the "
    "first line, a throwaway line (orb output pose), and the remaining lines "
    "have the format <feature num camera_num px_for_cam py_for_cam ...(all but "
    "feature num repeated for all observing cameras). The depths folder has a "
    "file per frame (<framenum>.txt) and each file has the first line with the "
    "frame id, second line should be ignored, and remaining lines have feature "
    "id follwed by depth (distance from left camera). All data in this folder "
    "has been adjusted so that the first frame has id 0.");

namespace {
const std::string kIntrinsicCalibrationPath = "camera_matrix.txt";
const std::string kExtrinsicCalibrationPath = "extrinsics.txt";
const std::string kFeaturesFolderPath = "features/";
const std::string kFeaturesFile = "features.txt";
const std::string kDepthsFolder = "depths/";
const std::string kVelocitiesFolder = "velocities/";
const std::string kPosesFolder = "poses/";
const std::string kRobotPosesFile = "initial_robot_poses_by_node.txt";
const std::string kNodesWithTimestampsFolder = "timestamps/";
const std::string kNodesWithTimestampsFile = "node_ids_and_timestamps.txt";
}  // namespace

struct FeatureProjector {
  vtr::FrameId frame_id_;
  vtr::FeatureId feature_id_;
  vtr::PixelCoord<double> measurement_;
  vtr::Pose3D<double> robot_pose_;
  double depth_;
  vtr::CameraId cam_id_;

  FeatureProjector() {}
  FeatureProjector(const vtr::FrameId& frame_id,
                   const vtr::FeatureId& feature_id,
                   const vtr::PixelCoord<double>& measurement,
                   const vtr::CameraId& camera_id)
      : frame_id_(frame_id),
        feature_id_(feature_id),
        measurement_(measurement),
        cam_id_(camera_id) {}
};

void LoadCameraIntrinsics(
    const std::string& intrinsics_path,
    std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>&
        intrinsics) {
  std::vector<file_io::CameraIntrinsicsWithId> camera_intrinsics_with_ids;
  file_io::readCameraIntrinsicsWithIdsFromFile(intrinsics_path,
                                               camera_intrinsics_with_ids);

  for (const file_io::CameraIntrinsicsWithId& cam_intrinsics :
       camera_intrinsics_with_ids) {
    vtr::CameraIntrinsicsMat<double> intrinsics_mat;
    intrinsics_mat << cam_intrinsics.mat_00, cam_intrinsics.mat_01,
        cam_intrinsics.mat_02, cam_intrinsics.mat_10, cam_intrinsics.mat_11,
        cam_intrinsics.mat_12, cam_intrinsics.mat_20, cam_intrinsics.mat_21,
        cam_intrinsics.mat_22;
    intrinsics[cam_intrinsics.camera_id] = intrinsics_mat;
  }
}

void LoadCameraExtrinsics(
    const std::string& extrinsics_path,
    std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>&
        extrinsics,
    vtr::CameraId& primary_camera_id) {
  std::vector<file_io::CameraExtrinsicsWithId> camera_extrinsics_with_ids;
  file_io::readCameraExtrinsicsWithIdsFromFile(extrinsics_path,
                                               camera_extrinsics_with_ids);

  bool first = true;
  for (const file_io::CameraExtrinsicsWithId& extrinsics_for_cam :
       camera_extrinsics_with_ids) {
    vtr::Position3d<double> extrinsics_pos(extrinsics_for_cam.transl_x,
                                           extrinsics_for_cam.transl_y,
                                           extrinsics_for_cam.transl_z);

    vtr::Orientation3D<double> extrinsics_orient(
        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
                           extrinsics_for_cam.quat_x,
                           extrinsics_for_cam.quat_y,
                           extrinsics_for_cam.quat_z));
    vtr::CameraExtrinsics<double> extrinsics_obj(extrinsics_pos,
                                                 extrinsics_orient);

    extrinsics[extrinsics_for_cam.camera_id] = extrinsics_obj;
    if (first) {
      primary_camera_id = extrinsics_for_cam.camera_id;
      first = false;
    }
  }
}

void loadTrajectoryFromVelocities(
    const std::string& dataset_path,
    const vtr::CameraId& primary_cam_id,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>&
        extrinsics_map,
    std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>& trajectory,
    vtr::FrameId& min_orig_frame_id) {
  vtr::CameraExtrinsics<double>
      primary_cam_extrinsics;  // TODO  need to verify that this is camera pose
                               // relative to baselink (and not vice versa)
  if (extrinsics_map.find(primary_cam_id) == extrinsics_map.end()) {
    LOG(ERROR) << "Could not find extrinsics for the primary camera. Exiting";
    exit(1);
  }
  primary_cam_extrinsics = extrinsics_map.at(primary_cam_id);
  vtr::Pose3D<double> extrinsics_inv = vtr::poseInverse(primary_cam_extrinsics);

  // load all relative poses (velocity)
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> velocities_by_frame_id;
  min_orig_frame_id = std::numeric_limits<vtr::FrameId>::max();
  for (const auto& entry :
       fs::directory_iterator(fs::path(dataset_path + kVelocitiesFolder))) {
    const auto file_extension = entry.path().extension().string();
    if (!is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }
    std::string line;
    getline(data_file_stream, line);
    vtr::FrameId frame_id;
    std::stringstream ss_frame_id(line);
    ss_frame_id >> frame_id;
    getline(data_file_stream, line);
    std::stringstream ss_robot_pose(line);
    float x, y, z, qx, qy, qz, qw;
    ss_robot_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
    vtr::Position3d<double> translation(x, y, z);
    vtr::Orientation3D<double> rotation_a(Eigen::Quaterniond(qw, qx, qy, qz));

    // We want to get the poses of the robot frame rather than the camera, so we
    // need to adjust the relative poses to refer to the robot frame
    vtr::Pose3D<double> cam_velocity(translation, rotation_a);
    vtr::Pose3D<double> base_link_velocity =
        vtr::combinePoses(primary_cam_extrinsics,
                          vtr::combinePoses(cam_velocity, extrinsics_inv));
    velocities_by_frame_id[frame_id] = base_link_velocity;

    // Frame id - 1 because velocity is presumed to be relative to the prior
    // frame (therefore implying that frame_id -1 exists)
    min_orig_frame_id = std::min(min_orig_frame_id, frame_id - 1);
  }
  // convert all relative poses to absolute ones
  size_t nframes = velocities_by_frame_id.size();

  trajectory[0] = vtr::Pose3D<double>(
      vtr::Position3d<double>(),
      vtr::Orientation3D<double>(0, Eigen::Vector3d::UnitZ()));
  // TODO Is this correctly handling non-zero initial frame output by ORB-SLAM
  for (size_t curr_frame_id = 1; curr_frame_id < nframes; ++curr_frame_id) {
    size_t prev_frame_id = curr_frame_id - 1;
    trajectory[curr_frame_id] = vtr::combinePoses(
        trajectory[prev_frame_id],
        velocities_by_frame_id[curr_frame_id + min_orig_frame_id]);
  }
}

void LoadDepths(
    const std::string& dataset_path,
    std::unordered_map<vtr::FeatureId, FeatureProjector>& features) {
  for (const auto& entry :
       fs::directory_iterator(fs::path(dataset_path + kDepthsFolder))) {
    const auto file_extension = entry.path().extension().string();
    if (!is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }
    std::string line;
    getline(data_file_stream, line);
    vtr::FrameId frame_id;
    std::stringstream ss_frame_id(line);
    ss_frame_id >> frame_id;
    getline(data_file_stream, line);  // skip the second line

    vtr::FeatureId feature_id;
    float depth;
    while (getline(data_file_stream, line)) {
      std::stringstream ss_depth(line);
      ss_depth >> feature_id >> depth;
      if (features[feature_id].frame_id_ == frame_id) {
        features[feature_id].depth_ = depth;
      }
    }
  }
}

void LoadFeaturesWithRelPoses(
    const std::string& dataset_path,
    std::unordered_map<vtr::FeatureId, FeatureProjector>& features,
    std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>& trajectory,
    vtr::FeatureId& min_feature_id,
    vtr::FeatureId& max_feature_id) {
  for (const auto& entry : fs::directory_iterator(fs::path(dataset_path))) {
    const auto file_extension = entry.path().extension().string();
    if (!is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }
    // TODO need to verify that frame ids here have already been converted to 0
    // indexing
    std::string line;
    getline(data_file_stream, line);
    vtr::FrameId frame_id;
    std::stringstream ss_frame_id(line);
    ss_frame_id >> frame_id;
    getline(data_file_stream, line);  // skip the absolute pose line

    vtr::FeatureId feature_id;
    vtr::CameraId camera1_id, camera2_id;
    float measurement_x1, measurement_y1, measurement_x2, measurement_y2;
    while (getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      ss_feature >> feature_id >> camera1_id >> measurement_x1 >>
          measurement_y1 >> camera2_id >> measurement_x2 >> measurement_y2;
      if (feature_id > max_feature_id) {
        max_feature_id = feature_id;
      }
      if (feature_id < min_feature_id) {
        min_feature_id = feature_id;
      }
      features[feature_id] = FeatureProjector(
          frame_id,
          feature_id,
          vtr::PixelCoord<double>(measurement_x1, measurement_y1),
          camera1_id);
    }
  }

  // iterate through all depths associated with the left camera measuremnts
  LoadDepths(dataset_path, features);

  // Update the robot poses in features
  for (auto& feature : features) {
    size_t frame_id = feature.second.frame_id_;
    feature.second.robot_pose_ = trajectory[frame_id];
  }
}

std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> EstimatePoints3D(
    const std::unordered_map<vtr::FeatureId, FeatureProjector>& features,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>&
        intrinsics_map,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>&
        extrinsics_map) {
  std::unordered_map<vtr::FeatureId,
                     std::pair<vtr::FrameId, vtr::Position3d<double>>>
      points;
  for (const auto& feature : features) {
    if (points.find(feature.first) != points.end()) {
      if (points[feature.first].first < feature.second.frame_id_) {
        continue;
      }
    }
    vtr::CameraId cam_id = feature.second.cam_id_;
    if (intrinsics_map.find(cam_id) == intrinsics_map.end()) {
      LOG(WARNING) << "No intrinsics found for cam id " << cam_id
                   << " associated with feature " << feature.first
                   << " at frame " << feature.second.frame_id_
                   << ". Skipping feature";
      continue;
    }
    if (extrinsics_map.find(cam_id) == extrinsics_map.end()) {
      LOG(WARNING) << "No intrinsics found for cam id " << cam_id
                   << " associated with feature " << feature.first
                   << " at frame " << feature.second.frame_id_
                   << ". Skipping feature";
      continue;
    }
    const vtr::CameraIntrinsicsMat<double> intrinsics =
        intrinsics_map.at(cam_id);
    const vtr::CameraExtrinsics<double> extrinsics = extrinsics_map.at(cam_id);
    vtr::Position3d<double> point =
        getWorldFramePos(feature.second.measurement_,
                         intrinsics,
                         extrinsics,
                         feature.second.robot_pose_,
                         feature.second.depth_);
    points[feature.first] = std::make_pair(feature.second.frame_id_, point);
    //    if (!dumpToFile) {  // debug
    //                        //      std::cout << feature.second << std::endl;
    //      std::cout << "point: " << point.transpose() << std::endl;
    //      std::cout << std::endl;
    //    }
  }
  std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> feature_ests;
  for (const auto& point : points) {
    feature_ests[point.first] = point.second.second;
  }
  return feature_ests;
}

void writePointsToFile(
    const std::string& output_file,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& points,
    const vtr::FeatureId& min_feature_id,
    const vtr::FeatureId& max_feature_id) {
  std::ofstream fp;
  //  fp.open(dataset_path + kFeaturesPath, std::fstream::trunc);
  fp.open(output_file, std::fstream::trunc);
  if (!fp.is_open()) {
    LOG(FATAL) << "[dataset_path + kFeaturesPath] Fail to load: "
               << output_file;
    //               << dataset_path + kFeaturesPath;
    return;
  }
  std::vector<file_io::FeatureEstWithId> feature_ests;
  for (vtr::FeatureId feat_id = min_feature_id; feat_id <= max_feature_id;
       feat_id++) {
    if (points.find(feat_id) == points.end()) {
      continue;
    }
    vtr::Position3d<double> point = points.at(feat_id);

    file_io::FeatureEstWithId feat;
    feat.feature_id = feat_id;
    feat.x = point.x();
    feat.y = point.y();
    feat.z = point.z();

    feature_ests.emplace_back(feat);
  }

  file_io::writeFeatureEstsWithIdToFile(output_file, feature_ests);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  CHECK(!FLAGS_calibration_path.empty()) << "No calibration path specified.";
  CHECK(!FLAGS_raw_data_path.empty()) << "No raw data path specified.";
  CHECK(!FLAGS_processed_data_path.empty())
      << "No processed data path specified.";

  std::string calibration_path =
      file_io::ensureDirectoryPathEndsWithSlash(FLAGS_calibration_path);
  std::string raw_data_path =
      file_io::ensureDirectoryPathEndsWithSlash(FLAGS_raw_data_path);
  std::string processed_data_path =
      file_io::ensureDirectoryPathEndsWithSlash(FLAGS_processed_data_path);
  LOG(INFO) << "Calibration path: " << calibration_path;
  LOG(INFO) << "Raw data path: " << raw_data_path;
  LOG(INFO) << "Processed data path: " << processed_data_path;

  // extrinsics: transform from the robot frame to the camera frame
  // TODO: double check this transformation (it takes an inverse in
  // structured_main)
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      extrinsics_map;
  vtr::CameraId primary_cam_id;
  LOG(INFO) << "Loading extrinsics from path"
            << (calibration_path + kExtrinsicCalibrationPath);
  LoadCameraExtrinsics(calibration_path + kExtrinsicCalibrationPath,
                       extrinsics_map,
                       primary_cam_id);

  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> trajectory;
  vtr::FrameId min_orig_frame_id;
  LOG(INFO) << "Loading trajectory from velocities";
  loadTrajectoryFromVelocities(raw_data_path,
                               primary_cam_id,
                               extrinsics_map,
                               trajectory,
                               min_orig_frame_id);

  std::unordered_map<vtr::FeatureId, FeatureProjector> features_map;
  vtr::FeatureId min_feature_id = std::numeric_limits<vtr::FeatureId>::max();
  vtr::FeatureId max_feature_id = std::numeric_limits<vtr::FeatureId>::min();
  LoadFeaturesWithRelPoses(processed_data_path,
                           features_map,
                           trajectory,
                           min_feature_id,
                           max_feature_id);
  // LoadFeaturesWithAbsPosesByFrameId(FLAGS_dataset_path, 500, features_map);
  // intrinsics: project 3D point from camera's baselink frame to 2D measurement
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      intrinsics_map;
  LOG(INFO) << "Loading intrinsics from path"
            << (calibration_path + kIntrinsicCalibrationPath);
  LoadCameraIntrinsics(calibration_path + kIntrinsicCalibrationPath,
                       intrinsics_map);

  // debug
  std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> feature_ests =
      EstimatePoints3D(features_map, intrinsics_map, extrinsics_map);

  std::string output_features_dir = processed_data_path + kFeaturesFolderPath;
  file_io::makeDirectoryIfDoesNotExist(output_features_dir);
  writePointsToFile(output_features_dir + kFeaturesFile,
                    feature_ests,
                    min_feature_id,
                    max_feature_id);
  std::string robot_poses_dir = processed_data_path + kPosesFolder;
  file_io::makeDirectoryIfDoesNotExist(robot_poses_dir);

  std::vector<std::pair<uint64_t, pose::Pose3d>> poses_vector;

  vtr::FrameId max_frame_id = std::numeric_limits<vtr::FrameId>::min();
  for (const auto& pose_map_entry : trajectory) {
    max_frame_id = std::max(max_frame_id, pose_map_entry.first);
  }

  for (vtr::FrameId frame_idx = 0; frame_idx <= max_frame_id; frame_idx++) {
    if (trajectory.find(frame_idx) == trajectory.end()) {
      LOG(ERROR)
          << "No trajectory node found for frame " << frame_idx
          << " (after switching to 0 index). Initial pose generation failed";
      exit(1);
    }
    vtr::Pose3D<double> traj_pose = trajectory.at(frame_idx);
    pose::Pose3d file_io_pose = std::make_pair(
        traj_pose.transl_, Eigen::Quaterniond(traj_pose.orientation_));
    poses_vector.emplace_back(std::make_pair(frame_idx, file_io_pose));
  }

  file_io::writePose3dsWithNodeIdToFile(robot_poses_dir + kRobotPosesFile,
                                        poses_vector);

  std::string input_timestamps_path =
      raw_data_path + kNodesWithTimestampsFolder + kNodesWithTimestampsFile;
  std::string output_timestamps_dir =
      processed_data_path + kNodesWithTimestampsFolder;
  std::vector<file_io::NodeIdAndTimestamp> original_timestamps;
  file_io::readNodeIdsAndTimestampsFromFile(input_timestamps_path,
                                            original_timestamps);

  std::vector<file_io::NodeIdAndTimestamp> adjusted_timestamps;
  for (const file_io::NodeIdAndTimestamp& node_id_and_timestamp :
       original_timestamps) {
    file_io::NodeIdAndTimestamp adjusted = node_id_and_timestamp;
    adjusted.node_id_ = adjusted.node_id_ - min_orig_frame_id;
    adjusted_timestamps.emplace_back(adjusted);
  }

  file_io::makeDirectoryIfDoesNotExist(output_timestamps_dir);
  file_io::writeNodeIdsAndTimestampsToFile(
      output_timestamps_dir + kNodesWithTimestampsFile, adjusted_timestamps);
}