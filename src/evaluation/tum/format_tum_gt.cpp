#include <analysis/cumulative_timer_constants.h>
#include <analysis/cumulative_timer_factory.h>
#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <debugging/ground_truth_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/bounding_box_by_timestamp_io.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_info_io_utils.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <file_io/cv_file_storage/long_term_object_map_file_storage_io.h>
#include <file_io/cv_file_storage/object_and_reprojection_feature_pose_graph_file_storage_io.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_timestamp_io.h>
#include <file_io/pose_io_utils.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/bounding_box_retriever.h>
#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/configuration/full_ov_slam_config.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/long_term_map/long_term_map_factor_creator.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/visual_feature_frontend/visual_feature_front_end.h>
#include <refactoring/visual_feature_processing/orb_output_low_level_feature_reader.h>
#include <refactoring/visualization/ros_visualization.h>
#include <refactoring/visualization/save_to_file_visualizer.h>
#include <ros/ros.h>
#include <run_optimization_utils/optimization_runner.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

DEFINE_string(gt_filepath_in,  "", "input groundtruth filepath (in TUM format)");
DEFINE_string(gt_filepath_out, "", "output groundtruth filepath (in ObVi-SLAM format)");

void LoadPoses(const std::string &filepath, std::map<double, vtr::Pose3D<double>> &poses) {
  std::ifstream infile;
  infile.open(filepath, std::ios::in);
  if (!infile.is_open()) {
    LOG(FATAL) << "Failed to open file " << filepath;
  }
  std::string line;
  double timestamp, tx, ty, tz, qx, qy, qz, qw;
  while (std::getline(infile, line)) {
    if (line.at(0) == '#') { continue; }
    std::stringstream ss(line);
    ss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    poses[timestamp] = 
        vtr::Pose3D<double>(Eigen::Vector3d(tx, ty, tz), 
            vtr::Orientation3D<double>(Eigen::Quaterniond(qw, qx, qy, qz)));
  }
  infile.close();
}

void SavePoses(const std::string &filepath, const std::map<double, vtr::Pose3D<double>> &timestamped_poses) {
  std::ofstream ofile;
  ofile.open(filepath, std::ios::out);
  if (!ofile.is_open()) {
    LOG(FATAL) << "Failed to open file " << filepath;
  }
  const char kDelim = ' ';
  for (const auto &stamped_pose : timestamped_poses) {
    Eigen::Quaterniond quat(stamped_pose.second.orientation_);
    ofile << std::setprecision(20) << stamped_pose.first << kDelim
          << stamped_pose.second.transl_.x() << kDelim
          << stamped_pose.second.transl_.y() << kDelim
          << stamped_pose.second.transl_.z() << kDelim
          << quat.w() << kDelim
          << quat.x() << kDelim
          << quat.y() << kDelim
          << quat.z() << std::endl;
  }
  ofile.close();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  if (FLAGS_gt_filepath_in.empty()) {
    LOG(FATAL) << "gt_filepath_in cannot be empty!";
  }

  if (FLAGS_gt_filepath_out.empty()) {
    LOG(FATAL) << "gt_filepath_out cannot be empty!";
  }

  std::map<double, vtr::Pose3D<double>> timestamped_poses;  
  LoadPoses(FLAGS_gt_filepath_in, timestamped_poses);
  LOG(INFO) << "timestamped_poses size: " << timestamped_poses.size();

  std::vector<vtr::Pose3D<double>> poses;
  for (const auto &timestamped_pose : timestamped_poses) {
    poses.emplace_back(timestamped_pose.second);
  }
  poses = adjustTrajectoryToStartAtOrigin(poses);
  if (poses.size() != timestamped_poses.size()) {
    LOG(FATAL) << "size mismatches: " << poses.size() << " vs. " 
        << timestamped_poses.size();
  }
  size_t poseIdx = 0;
  for (auto &timestamped_pose : timestamped_poses) {
    timestamped_pose.second = poses.at(poseIdx);
    ++poseIdx;
  }
  LOG(INFO) << "timestamped_poses size: " << timestamped_poses.size();
  SavePoses(FLAGS_gt_filepath_out, timestamped_poses);

  return 0;
}