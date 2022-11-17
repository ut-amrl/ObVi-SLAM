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

DEFINE_string(dataset_path,
              "",
              "\nPath to folder containing the dataset. Structured as - \n"
              "vslam_setX/\n\tcalibration/camera_matrix.txt\n\tfeatures/"
              "features.txt\n\t0000x.txt\n\n");

namespace {
const std::string kIntrinsicCalibrationPath = "calibration/camera_matrix.txt";
const std::string kExtrinsicCalibrationPath = "calibration/extrinsics.txt";
const std::string kFeaturesPath = "features/features.txt";
const std::string kDepthsFolder = "depths/";
const std::string kVelocitiesFolder = "velocities/";
const std::string kPosesFolder = "poses/";
const std::string kRobotPosesPath =
    kPosesFolder + "initial_robot_poses_by_node.txt";
}  // namespace

struct FeatureProjector {
  vtr::FrameId frame_id_;
  vtr::FeatureId feature_id_;
  vtr::PixelCoord<double> measurement_;
  vtr::Pose3D<double> robot_pose_;
  double depth_;

  FeatureProjector() {}
  FeatureProjector(const vtr::FrameId& frame_id,
                   const vtr::FeatureId& feature_id,
                   const vtr::PixelCoord<double>& measurement)
      : frame_id_(frame_id),
        feature_id_(feature_id),
        measurement_(measurement) {}
};

void LoadCameraIntrinsics(
    const std::string& intrinsics_path,
    std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>&
        intrinsics) {
  std::ifstream intrinsics_file_stream;
  intrinsics_file_stream.open(intrinsics_path);
  if (intrinsics_file_stream.fail()) {
    LOG(FATAL) << "LoadCameraIntrinsics() failed to load: " << intrinsics_path
               << " are you sure this a valid path to the intrinsics file? ";
    return;
  }
  std::string line;
  vtr::CameraId camera_id;
  float fx, fy, cx, cy;
  while (std::getline(intrinsics_file_stream, line)) {
    std::stringstream ss_intrinsics(line);
    ss_intrinsics >> camera_id >> fx >> fy >> cx >> cy;
    intrinsics[camera_id].setIdentity();
    intrinsics[camera_id](0, 0) = fx;
    intrinsics[camera_id](1, 1) = fy;
    intrinsics[camera_id](0, 2) = cx;
    intrinsics[camera_id](1, 2) = cy;
  }
}

void LoadCameraExtrinsics(
    const std::string& extrinsics_path,
    std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>&
        extrinsics) {
  std::ifstream extrinsics_file_stream;
  extrinsics_file_stream.open(extrinsics_path);
  if (extrinsics_file_stream.fail()) {
    LOG(FATAL) << "LoadCameraExtrinsics() failed to load: " << extrinsics_path
               << " are you sure this a valid path to the extrinsics file? ";
    return;
  }
  std::string line;
  float camera_id, x, y, z, qx, qy, qz, qw;
  while (std::getline(extrinsics_file_stream, line)) {
    std::stringstream ss_extrinsics(line);
    ss_extrinsics >> camera_id >> x >> y >> z >> qx >> qy >> qz >> qw; // TODO added the qz (wasn't populated before); is this right?
    extrinsics[camera_id].transl_ = vtr::Position3d<double>(x, y, z);
    extrinsics[camera_id].orientation_ =
        vtr::Orientation3D<double>(Eigen::Quaterniond(qw, qx, qy, qz));
  }
}

void loadTrajectoryFromVelocities(
    const std::string& dataset_path,
    std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>& trajectory,
    vtr::FrameId& min_orig_frame_id) {
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
    vtr::Pose3D<double> velocity(translation, rotation_a);
    velocities_by_frame_id[frame_id] = velocity;
    min_orig_frame_id = std::min(min_orig_frame_id, frame_id);
  }
  // convert all relative poses to absolute ones
  size_t nframes = velocities_by_frame_id.size();
  //  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> poses_by_frame_id;
  // NOTE hardcode first pose
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
    std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>& trajectory) {
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
      features[feature_id] = FeatureProjector(
          frame_id,
          feature_id,
          vtr::PixelCoord<double>(measurement_x1, measurement_y1));
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
    const vtr::CameraIntrinsicsMat<double>& intrinsics,
    const vtr::CameraExtrinsics<double>& extrinsics) {
  std::unordered_map<vtr::FeatureId,
                     std::pair<vtr::FrameId, vtr::Position3d<double>>>
      points;
  for (const auto& feature : features) {
    if (points.find(feature.first) != points.end()) {
      if (points[feature.first].first < feature.second.frame_id_) {
        continue;
      }
    }
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
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& points) {
  std::ofstream fp;
  //  fp.open(dataset_path + kFeaturesPath, std::fstream::trunc);
  fp.open(output_file, std::fstream::trunc);
  if (!fp.is_open()) {
    LOG(FATAL) << "[dataset_path + kFeaturesPath] Fail to load: "
               << output_file;
    //               << dataset_path + kFeaturesPath;
    return;
  }
  for (const auto& point : points) {
    fp << point.first << " " << point.second.x() << " " << point.second.y()
       << " " << point.second.z() << std::endl;
  }
  fp.close();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> trajectory;
  vtr::FrameId min_orig_frame_id;
  loadTrajectoryFromVelocities(
      FLAGS_dataset_path, trajectory, min_orig_frame_id);

  std::unordered_map<vtr::FeatureId, FeatureProjector> features_map;
  LoadFeaturesWithRelPoses(FLAGS_dataset_path, features_map, trajectory);
  // LoadFeaturesWithAbsPosesByFrameId(FLAGS_dataset_path, 500, features_map);
  // intrinsics: project 3D point from camera's baselink frame to 2D measurement
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      intrinsics_map;
  LoadCameraIntrinsics(FLAGS_dataset_path + kIntrinsicCalibrationPath,
                       intrinsics_map);
  // extrinsics: transform from the robot frame to the camera frame
  // TODO: double check this transformation (it takes an inverse in
  // structured_main)
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      extrinsics_map;
  LoadCameraExtrinsics(FLAGS_dataset_path + kExtrinsicCalibrationPath,
                       extrinsics_map);
  // debug
  std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> feature_ests =
      EstimatePoints3D(features_map, intrinsics_map[1], extrinsics_map[1]);

  writePointsToFile(FLAGS_dataset_path + kFeaturesPath, feature_ests);

  if (kRobotPosesPath.empty()) {
    LOG(ERROR) << "No robot poses file provided";
    exit(1);
  }

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
    pose::Pose3d file_io_pose = std::make_pair(traj_pose.transl_, Eigen::Quaterniond(traj_pose.orientation_));
    poses_vector.emplace_back(std::make_pair(frame_idx, file_io_pose));
  }

  file_io::writePose3dsWithNodeIdToFile(kRobotPosesPath, poses_vector);
}