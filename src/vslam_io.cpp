#include "vslam_io.h"

#include <glog/logging.h>

#include <algorithm>
#include <experimental/filesystem>  //c++ 17 file iteration
#include <fstream>
#include <map>
#include <random>

namespace {
const std::string kIntrinsicsPath = "calibration/camera_matrix.txt";
const std::string kExtrinsicsPath = "calibration/extrinsics.txt";
const std::string kFeaturesPath = "features/features.txt";
const std::string kTxtExtension = ".txt";
const vslam_types::CameraId kDefaultCameraId = 1;
}  // namespace

using namespace vslam_types;
using namespace std;

namespace vslam_io {

namespace fs = std::experimental::filesystem;

void LoadStructurelessUTSLAMProblem(
    const std::string& dataset_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob) {
  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames
  std::unordered_map<uint64_t, RobotPose> poses_by_id;

  std::function<VisionFeatureTrack*(const uint64_t&)> feature_track_retriever =
      [&](const uint64_t& feature_id) { return &(prob.tracks[feature_id]); };

  if (!ReadAllFeatureFiles(
          dataset_path, feature_track_retriever, poses_by_id)) {
    return;
  }

  // Sort all feature tracks so frame_idxs are in ascending order
  for (auto& ft : prob.tracks) {
    std::sort(ft.second.track.begin(), ft.second.track.end());
  }

  SetRobotPosesInSlamProblem(poses_by_id, prob);

  // Load camera calibration data
  vslam_io::LoadCameraCalibrationData(dataset_path,
                                      prob.camera_instrinsics_by_camera,
                                      prob.camera_extrinsics_by_camera);
}

void LoadStructurelessUTSLAMProblemMicrosoft(
    const std::string& dataset_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob) {
  std::unordered_map<uint64_t, RobotPose> poses_by_id;

  std::function<VisionFeatureTrack*(const uint64_t&)> feature_track_retriever =
      [&](const uint64_t& feature_id) { return &(prob.tracks[feature_id]); };

  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames to load features
  for (const auto& entry :
       fs::directory_iterator(fs::path(dataset_path + "pics/"))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the dataset_path directory
    if (!fs::is_regular_file(entry) || file_extension != kTxtExtension) {
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
    ReadFeaturesFromFile(data_file_stream, frame_id, feature_track_retriever);
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
    if (!fs::is_regular_file(entry) || file_extension != kTxtExtension) {
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

  SetRobotPosesInSlamProblem(poses_by_id, prob);

  vslam_io::LoadCameraIntrinsicsAndExtrinsicsMicrosoftDataset(
      prob.camera_instrinsics_by_camera, prob.camera_extrinsics_by_camera);
}

void LoadStructurelessUTSLAMProblemTartan(
    const std::string& dataset_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob) {
  std::unordered_map<uint64_t, RobotPose> poses_by_id;

  std::function<VisionFeatureTrack*(const uint64_t&)> feature_track_retriever =
      [&](const uint64_t& feature_id) { return &(prob.tracks[feature_id]); };

  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames to load features
  for (const auto& entry :
       fs::directory_iterator(fs::path(dataset_path + "matches_left/"))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the dataset_path directory
    if (!fs::is_regular_file(entry) || file_extension != kTxtExtension) {
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
    ReadFeaturesFromFile(data_file_stream, frame_id, feature_track_retriever);
  }

  // Sort all feature tracks so frame_idxs are in ascending order
  for (auto& ft : prob.tracks) {
    std::sort(ft.second.track.begin(), ft.second.track.end());
  }

  // Load ALL poses
  std::ifstream pose_file_stream;
  pose_file_stream.open(dataset_path + "pose_left.txt");
  if (pose_file_stream.fail()) {
    LOG(FATAL) << " Failed to open Tartan pose file.";
    return;
  }

  // Read in poses from all lines

  std::string line;
  uint64_t frame_id = 0;
  while (std::getline(pose_file_stream, line)) {
    std::stringstream ss_pose(line);
    float x, y, z, qx, qy, qz, qw;
    ss_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Vector3f loc(x, y, z);
    Eigen::Quaternionf angle_q(qw, qx, qy, qz);
    angle_q.normalize();
    Eigen::AngleAxisf angle(angle_q);
    RobotPose pose(frame_id, loc, angle);

    poses_by_id[frame_id] = pose;

    frame_id++;
  }

  SetRobotPosesInSlamProblem(poses_by_id, prob);

  vslam_io::LoadCameraIntrinsicsAndExtrinsicsTartanDataset(
      prob.camera_instrinsics_by_camera, prob.camera_extrinsics_by_camera);
}

void LoadCameraIntrinsicsAndExtrinsicsMicrosoftDataset(
    std::unordered_map<vslam_types::CameraId, vslam_types::CameraIntrinsics>&
        camera_intrinsics_by_camera_id,
    std::unordered_map<vslam_types::CameraId, vslam_types::CameraExtrinsics>&
        camera_extrinsics_by_camera_id) {
  Eigen::Matrix3f camera_mat;
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

  vslam_types::CameraIntrinsics intrinsics{camera_mat};
  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  vslam_types::CameraExtrinsics extrinsics{
      Eigen::Vector3f(0, 0, 0),
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse()};  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  camera_intrinsics_by_camera_id = {{kDefaultCameraId, intrinsics}};
  camera_extrinsics_by_camera_id = {{kDefaultCameraId, extrinsics}};
}

void LoadCameraIntrinsicsAndExtrinsicsTartanDataset(
    std::unordered_map<vslam_types::CameraId, vslam_types::CameraIntrinsics>&
        camera_intrinsics_by_camera_id,
    std::unordered_map<vslam_types::CameraId, vslam_types::CameraExtrinsics>&
        camera_extrinsics_by_camera_id) {
  Eigen::Matrix3f camera_mat;
  // TODO make not manual
  float fx = 320;
  float fy = 320;
  float cx = 320;
  float cy = 240;
  camera_mat.setIdentity();
  camera_mat(0, 0) = fx;
  camera_mat(1, 1) = fy;
  camera_mat(0, 2) = cx;
  camera_mat(1, 2) = cy;

  vslam_types::CameraIntrinsics intrinsics{camera_mat};
  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  vslam_types::CameraExtrinsics extrinsics{
      Eigen::Vector3f(0, 0, 0),
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse()};  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  camera_intrinsics_by_camera_id = {{kDefaultCameraId, intrinsics}};
  camera_extrinsics_by_camera_id = {{kDefaultCameraId, extrinsics}};
}

void LoadStructuredUTSLAMProblem(
    const std::string& dataset_path,
    vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack>&
        prob) {
  cout << "dataset_path: " << dataset_path << endl;
  
  // Iterate over all files/folders in the dataset_path directory - i.e. over
  // all frames
  std::unordered_map<uint64_t, RobotPose> poses_by_id;

  std::function<VisionFeatureTrack*(const uint64_t&)> feature_track_retriever =
      [&](const uint64_t& feature_id) {
        return &(prob.tracks[feature_id].feature_track);
      };
  if (!ReadAllFeatureFiles(
          dataset_path, feature_track_retriever, poses_by_id)) {
    return;
  }

  // Sort all feature tracks so frame_idxs are in ascending order
  for (auto& ft : prob.tracks) {
    std::sort(ft.second.feature_track.track.begin(),
              ft.second.feature_track.track.end());
  }

  SetRobotPosesInSlamProblem(poses_by_id, prob);
  std::unordered_map<uint64_t, Eigen::Vector3d> feature_estimates_by_id;
  if (!LoadInitialFeatureEstimates(dataset_path + kFeaturesPath,
                                   feature_estimates_by_id)) {
    return;
  }

  for (const auto& feature_id_and_est : feature_estimates_by_id) {
    if (prob.tracks.find(feature_id_and_est.first) != prob.tracks.end()) {
      prob.tracks[feature_id_and_est.first].point = feature_id_and_est.second;
    }
  }

  // Load camera calibration data
  vslam_io::LoadCameraCalibrationData(dataset_path,
                                      prob.camera_instrinsics_by_camera,
                                      prob.camera_extrinsics_by_camera);
}

void LoadCameraCalibrationData(
    const std::string& dataset_path,
    std::unordered_map<vslam_types::CameraId, vslam_types::CameraIntrinsics>&
        camera_intrinsics_by_camera_id,
    std::unordered_map<vslam_types::CameraId, vslam_types::CameraExtrinsics>&
        camera_extrinsics_by_camera_id) {

  /*
  Eigen::Matrix3f camera_mat;

  // Load camera calibration matrix
  vslam_io::LoadCameraCalibrationMatrix(
      calibration_directory_path + kIntrinsicsPath, camera_mat);

  vslam_types::CameraIntrinsics intrinsics{camera_mat};
  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  vslam_types::CameraExtrinsics extrinsics{
      Eigen::Vector3f(0, 0, 0),
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse()};  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  camera_intrinsics_by_camera_id = {{kDefaultCameraId, intrinsics}};
  camera_extrinsics_by_camera_id = {{kDefaultCameraId, extrinsics}};
  */

  LoadCameraIntrinsics(dataset_path + kIntrinsicsPath, camera_intrinsics_by_camera_id);
  LoadCameraExtrinsics(dataset_path + kExtrinsicsPath, camera_extrinsics_by_camera_id);
}

void LoadCameraCalibrationMatrix(const std::string& calibration_path,
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

void ReadFeaturesFromFile(std::ifstream& data_file_stream,
                          const uint64_t& frame_id,
                          std::function<VisionFeatureTrack*(const uint64_t&)>
                              feature_track_retriever) {
  // TODO Taijing -- fix this to read in pixel estimates and their camera ids

  std::string line;
  // Read pixels and IDS from all other lines
  while (std::getline(data_file_stream, line)) {
    std::stringstream ss_feature(line);
    std::vector<CameraId> camera_ids;
    int camera_id;
    std::vector<float> xs, ys;
    uint64_t feature_id;
    float x, y;
    ss_feature >> feature_id;
    ss_feature >> camera_id;
    while ( !ss_feature.eof() ) {
      ss_feature >> x >> y;
      camera_ids.emplace_back(camera_id);
      xs.emplace_back(x);
      ys.emplace_back(y);
      ss_feature >> camera_id;
    }
    CameraId primary_camera_id = kDefaultCameraId; // FIXME
    VisionFeatureTrack* feature_track = feature_track_retriever(feature_id);
    x = xs[0];
    y = ys[0];
    VisionFeature feature(feature_id,
                          frame_id,
                          {{camera_id, Eigen::Vector2f(x, y)}},
                          primary_camera_id); 
    feature_track->track.push_back(feature);
    feature_track->feature_idx = feature_id; 
    /*
    for (size_t i = 0; i < camera_ids.size(); ++i) {
      camera_id = camera_ids[i];
      x = xs[i];
      y = ys[i];
      VisionFeature feature(feature_id,
                            frame_id,
                            {{camera_id, Eigen::Vector2f(x, y)}},
                            primary_camera_id); 
      feature_track->track.push_back(feature);
      feature_track->feature_idx = feature_id;  // TODO dont reset this every time
    }*/
    // TODO should the feature ID just be the ID in the map and not a part of
    // the feature track/
  }
}

void ReadRobotPoseAndFeaturesFromFile(
    std::ifstream& data_file_stream,
    std::unordered_map<uint64_t, RobotPose>& poses_by_id,
    std::function<vslam_types::VisionFeatureTrack*(const uint64_t&)>
        feature_track_retriever) {
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
  Eigen::Vector3f loc(x, y, z);
  Eigen::Quaternionf angle_q(qw, qx, qy, qz);
  angle_q.normalize();
  Eigen::AngleAxisf angle(angle_q);
  RobotPose pose(frame_id, loc, angle);

  poses_by_id[frame_id] = pose;

  // Read pixels and IDS from all other
  ReadFeaturesFromFile(data_file_stream, frame_id, feature_track_retriever);
}

bool ReadAllFeatureFiles(
    const std::string& dataset_path,
    std::function<vslam_types::VisionFeatureTrack*(const uint64_t&)>
        feature_track_retriever,
    std::unordered_map<uint64_t, vslam_types::RobotPose>& poses_by_id) {
  cout << "dataset_path: " << dataset_path << endl;
  for (const auto& entry : fs::directory_iterator(fs::path(dataset_path))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the dataset_path directory
    if (!fs::is_regular_file(entry) || file_extension != kTxtExtension) {
      continue;
    }

    // If we haven't skipped it lets load it into the UTSLAMProblem
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return false;
    }

    ReadRobotPoseAndFeaturesFromFile(
        data_file_stream, poses_by_id, feature_track_retriever);
  }
  return true;
}

bool LoadInitialFeatureEstimates(
    const std::string& features_file,
    std::unordered_map<uint64_t, Eigen::Vector3d>& feature_estimates_by_id) {
  // Load features
  std::ifstream feature_file_stream;
  feature_file_stream.open(features_file);
  if (feature_file_stream.fail()) {
    LOG(FATAL) << " Failed to open 3D feature file.";
    return false;
  }


  // Read in IDs and features from all lines
  std::string line;
  while (std::getline(feature_file_stream, line)) {
    std::stringstream ss_feature(line);
    int ID;
    double x, y, z;
    ss_feature >> ID >> x >> y >> z;
    feature_estimates_by_id[ID] = Eigen::Vector3d(x, y, z);
  }

  return true;
}

void LoadCameraIntrinsics(const std::string& intrinsics_path,
                          std::unordered_map<vslam_types::CameraId, vslam_types::CameraIntrinsics>& intrinsics) {
  
  std::ifstream intrinsics_file_stream;
  intrinsics_file_stream.open(intrinsics_path);
  if (intrinsics_file_stream.fail()) {
    LOG(FATAL) << "LoadCameraIntrinsics() failed to load: " << intrinsics_path
               << " are you sure this a valid path to the intrinsics file? ";
    return;
  }
  std::string line;
  CameraId camera_id;
  float fx, fy, cx,cy;
  while ( std::getline(intrinsics_file_stream, line) ) {
    std::stringstream ss_intrinsics(line);
    ss_intrinsics >> camera_id >> fx >> fy >> cx >> cy;
    intrinsics[camera_id].camera_mat.setIdentity();
    intrinsics[camera_id].camera_mat(0, 0) = fx;
    intrinsics[camera_id].camera_mat(1, 1) = fy;
    intrinsics[camera_id].camera_mat(0, 2) = cx;
    intrinsics[camera_id].camera_mat(1, 2) = cy;
  }
  return;
}

void LoadCameraExtrinsics(const std::string& extrinsics_path,
                          vslam_types::CameraExtrinsics& extrinsics) {
  // TODO Taijing fill in :)
  std::ifstream extrinsics_file_stream;
  extrinsics_file_stream.open(extrinsics_path);
  if (extrinsics_file_stream.fail()) {
    LOG(FATAL) << "LoadCameraExtrinsics() failed to load: " << extrinsics_path
               << " are you sure this a valid path to the extrinsics file? ";
    return;
  }
  // TODO check extrinsics file format; 
  // only read the first line for now; may need to change to read in multiple lines
  // multi  cam: <cameraId> <x y z qx qy qz qw>
  float camera_id, x, y, z, qx, qy, qz, qw;
  std::string line;
  std::getline(extrinsics_file_stream, line);
  std::stringstream ss_extrinsics(line);
  ss_extrinsics >> camera_id >> x >> y >> z >> qx >> qy >> qw;
  extrinsics.translation = Eigen::Vector3f(x, y, z);
  extrinsics.rotation    = Eigen::Quaternionf(qw, qx, qy, qz);
  return;
}

void LoadCameraExtrinsics(const std::string& extrinsics_path,
                          std::unordered_map<vslam_types::CameraId, vslam_types::CameraExtrinsics>& extrinsics) {
   // TODO Taijing fill in :)
  std::ifstream extrinsics_file_stream;
  extrinsics_file_stream.open(extrinsics_path);
  if (extrinsics_file_stream.fail()) {
    LOG(FATAL) << "LoadCameraExtrinsics() failed to load: " << extrinsics_path
               << " are you sure this a valid path to the extrinsics file? ";
    return;
  }
  std::string line;
  float camera_id, x, y, z, qx, qy,qz, qw;
  while ( std::getline(extrinsics_file_stream, line) ) {
    std::stringstream ss_extrinsics(line);
    ss_extrinsics >> camera_id >> x >> y >> z >> qx >> qy >> qw;
    extrinsics[camera_id].translation = Eigen::Vector3f(x, y, z);
    extrinsics[camera_id].rotation    = Eigen::Quaternionf(qw, qx, qy, qz);
  }
  return;
}

}  // namespace vslam_io