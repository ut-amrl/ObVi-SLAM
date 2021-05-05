#include "vslam_io.h"

#include <glog/logging.h>

#include <algorithm>
#include <experimental/filesystem>  //c++ 17 file iteration
#include <fstream>
#include <map>
#include <random>

namespace {
const std::string calibration_path = "calibration/camera_matrix.txt";
const std::string features_path = "features/features.txt";
}  // namespace

using namespace vslam_types;

namespace vslam_io {

namespace fs = std::experimental::filesystem;

void LoadStructurelessUTSLAMProblem(
    const std::string& dataset_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob,
    Eigen::Matrix3f& camera_mat) {
  std::default_random_engine generator;  // TODO remove
  std::normal_distribution<double> distribution(0.0, 0.0);

  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames
  std::unordered_map<uint64_t, RobotPose> poses_by_id;
  for (const auto& entry : fs::directory_iterator(fs::path(dataset_path))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the dataset_path directory
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }

    // If we haven't skipped it lets load it into the UTSLAMProblem
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }

    std::string line;

    // Read frame ID from 1st line
    std::getline(data_file_stream, line);
    std::stringstream ss_id(line);
    uint64_t frame_id;
    ss_id >> frame_id;

    // Use 0 as the first index
    // TODO We should maybe just fix our dataset so it is zero indexed instead
    frame_id--;

    // Read frame/robot pose from 2nd line
    std::getline(data_file_stream, line);
    std::stringstream ss_pose(line);
    float x, y, z, qx, qy, qz, qw;
    ss_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Vector3f loc(x + distribution(generator),
                        y + distribution(generator),
                        z + distribution(generator));
    Eigen::Quaternionf angle_q(qw, qx, qy, qz);
    angle_q.normalize();
    Eigen::AngleAxisf angle(angle_q);
    RobotPose pose(frame_id, loc, angle);

    poses_by_id[frame_id] = pose;

    // Read pixels and IDS from all other lines
    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      uint64_t feature_id;
      float x, y;
      ss_feature >> feature_id >> x >> y;
      VisionFeature feature(feature_id, frame_id, Eigen::Vector2f(x, y));
      prob.tracks[feature_id].track.push_back(feature);
      prob.tracks[feature_id].feature_idx =
          feature_id;  // TODO dont reset this every time

      // TODO should the feature ID just be the ID in the map and not a part of
      // the feature track/
    }
  }

  // Sort all feature tracks so frame_idxs are in ascending order
  for (auto& ft : prob.tracks) {
    std::sort(ft.second.track.begin(), ft.second.track.end());
  }

  for (uint64_t frame_num = 0; frame_num < poses_by_id.size(); frame_num++) {
    if (poses_by_id.find(frame_num) == poses_by_id.end()) {
      LOG(ERROR) << "No pose found for frame num (after subtracting 1) "
                 << frame_num;
      return;
    }
    prob.robot_poses.emplace_back(poses_by_id[frame_num]);
  }

  // Load camera calibration matrix
  vslam_io::LoadCameraCalibration(
      dataset_path + "calibration/camera_matrix.txt", camera_mat);

  return;
}

void LoadStructurelessUTSLAMProblemMicrosoft(
    const std::string& dataset_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob,
    Eigen::Matrix3f& camera_mat) {
  std::unordered_map<uint64_t, RobotPose> poses_by_id;
  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames to load features
  for (const auto& entry :
       fs::directory_iterator(fs::path(dataset_path + "pics/"))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the dataset_path directory
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    // If we haven't skipped it lets load it into the UTSLAMProblem
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }
    // Get frame ID from file name
    std::istringstream iss(entry.path().filename().stem());
    uint64_t frame_id;
    iss >> frame_id;

    // Read pixels and IDS from all other lines
    std::string line;
    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      uint64_t feature_id;
      float x, y;
      ss_feature >> feature_id >> x >> y;
      VisionFeature feature(feature_id, frame_id, Eigen::Vector2f(x, y));
      prob.tracks[feature_id].track.push_back(feature);
      prob.tracks[feature_id].feature_idx =
          feature_id;  // TODO dont reset this every time?
    }
  }

  // Sort all feature tracks so frame_idxs are in ascending order
  for (auto& ft : prob.tracks) {
    std::sort(ft.second.track.begin(), ft.second.track.end());
  }

  // Load ALL poses
  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames to load poses
  for (const auto& entry :
       fs::directory_iterator(fs::path(dataset_path + "poses/"))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the dataset_path directory
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    // If we haven't skipped it lets load it into the UTSLAMProblem
    std::ifstream pose_file_stream(entry.path());
    if (pose_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }
    // Get frame ID from file name
    std::istringstream iss(entry.path().filename().stem());
    uint64_t frame_id;
    iss >> frame_id;

    // Read SE(3 matrix from each file)
    std::string line;
    std::getline(pose_file_stream, line);  // First line
    std::stringstream ss_pose(line);
    float r00, r01, r02, t0;
    ss_pose >> r00 >> r01 >> r02 >> t0;
    std::getline(pose_file_stream, line);  // Second line
    ss_pose = std::stringstream(line);
    float r10, r11, r12, t1;
    ss_pose >> r10 >> r11 >> r12 >> t1;
    std::getline(pose_file_stream, line);  // Third line
    ss_pose = std::stringstream(line);
    float r20, r21, r22, t2;
    ss_pose >> r20 >> r21 >> r22 >> t2;

    Eigen::Vector3f loc(t0, t1, t2);
    Eigen::Matrix3f R;
    R << r00, r01, r02, r10, r11, r12, r20, r21, r22;

    Eigen::AngleAxisf angle(R);
    RobotPose pose(frame_id, loc, angle);

    poses_by_id[frame_id] = pose;
  }

  for (uint64_t frame_num = 0; frame_num < poses_by_id.size(); frame_num++) {
    if (poses_by_id.find(frame_num) == poses_by_id.end()) {
      LOG(ERROR) << "No pose found for frame num (after subtracting 1) "
                 << frame_num;
      return;
    }
    prob.robot_poses.emplace_back(poses_by_id[frame_num]);
  }

  // TODO make not manual
  float fx = 585;
  float fy = 585;
  float cx = 320;
  float cy = 240;
  camera_mat.setIdentity();
  camera_mat(0, 0) = fx;
  camera_mat(1, 1) = fy;
  camera_mat(0, 2) = cx;
  camera_mat(1, 2) = cy;

  return;
}  // namespace vslam_io

void LoadStructuredUTSLAMProblem(
    const std::string& dataset_path,
    vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack>& prob,
    Eigen::Matrix3f& camera_mat) {
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.0);

  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames
  std::unordered_map<uint64_t, RobotPose> poses_by_id;
  for (const auto& entry : fs::directory_iterator(fs::path(dataset_path))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the dataset_path directory
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }

    // If we haven't skipped it lets load it into the UTSLAMProblem
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }

    std::string line;

    // Read frame ID from 1st line
    std::getline(data_file_stream, line);
    std::stringstream ss_id(line);
    uint64_t frame_id;
    ss_id >> frame_id;

    // Use 0 as the first index
    // TODO We should maybe just fix our dataset so it is zero indexed instead
    frame_id--;

    // Read frame/robot pose from 2nd line
    std::getline(data_file_stream, line);
    std::stringstream ss_pose(line);
    float x, y, z, qx, qy, qz, qw;
    ss_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Vector3f loc(x + distribution(generator),
                        y + distribution(generator),
                        z + distribution(generator));
    Eigen::Quaternionf angle_q(qw, qx, qy, qz);
    angle_q.normalize();
    Eigen::AngleAxisf angle(angle_q);
    RobotPose pose(frame_id, loc, angle);

    poses_by_id[frame_id] = pose;

    // Read pixels and IDS from all other lines
    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      uint64_t feature_id;
      float x, y;
      ss_feature >> feature_id >> x >> y;
      VisionFeature feature(feature_id, frame_id, Eigen::Vector2f(x, y));
      prob.tracks[feature_id].feature_track.track.push_back(feature);
      prob.tracks[feature_id].feature_track.feature_idx =
          feature_id;  // TODO dont reset this every time

      // TODO should the feature ID just be the ID in the map and not a part
      // of the feature track/
    }
  }

  // Sort all feature tracks so frame_idxs are in ascending order
  for (auto& ft : prob.tracks) {
    std::sort(ft.second.feature_track.track.begin(),
              ft.second.feature_track.track.end());
  }

  for (uint64_t frame_num = 0; frame_num < poses_by_id.size(); frame_num++) {
    if (poses_by_id.find(frame_num) == poses_by_id.end()) {
      LOG(ERROR) << "No pose found for frame num (after subtracting 1) "
                 << frame_num;
      return;
    }
    prob.robot_poses.emplace_back(poses_by_id[frame_num]);
  }

  // Load features
  std::ifstream feature_file_stream;
  feature_file_stream.open(dataset_path + features_path);
  if (feature_file_stream.fail()) {
    LOG(FATAL) << " Failed to open 3D feature file.";
    return;
  }

  // Read in IDs and features from all lines
  std::string line;
  while (std::getline(feature_file_stream, line)) {
    std::stringstream ss_feature(line);
    int ID;
    double x, y, z;
    ss_feature >> ID >> x >> y >> z;

    // See if we have the ID in the feature track map and populate its 3D
    // point if we do
    std::unordered_map<uint64_t, vslam_types::StructuredVisionFeatureTrack>::
        const_iterator it = prob.tracks.find(ID);
    if (it != prob.tracks.end()) {
      prob.tracks[ID].point = Eigen::Vector3d(x, y, z);
    }
  }

  // Load camera calibration matrix
  vslam_io::LoadCameraCalibration(dataset_path + calibration_path, camera_mat);

  return;
}

void LoadCameraCalibration(const std::string& calibration_path,
                           Eigen::Matrix3f& camera_mat) {
  std::ifstream calibration_file_stream;
  calibration_file_stream.open(calibration_path);
  if (calibration_file_stream.fail()) {
    LOG(FATAL) << "LoadCameraCalibration() failed to load: " << calibration_path
               << " are you sure this a valid path to the calibrationfile? ";
    return;
  }

  std::string line;
  std::getline(calibration_file_stream, line);
  std::stringstream ss_calib(line);
  float fx, fy, cx, cy;
  ss_calib >> fx >> fy >> cx >> cy;
  camera_mat.setIdentity();
  camera_mat(0, 0) = fx;
  camera_mat(1, 1) = fy;
  camera_mat(0, 2) = cx;
  camera_mat(1, 2) = cy;
  return;
}

}  // namespace vslam_io