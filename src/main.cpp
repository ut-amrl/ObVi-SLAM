#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>

#include "vslam_io.h"
#include "vslam_types.h"

DEFINE_string(
    data_path,
    "",
    "Path to folder containing the files with frames, poses, and labeled "
    "keypoints for each image");

DEFINE_string(calibration_path,
              "",
              "Path to the file containing the calibration (K) for the camera");

using std::cout;
using std::endl;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // Make empty unstructured slam problem
  vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> prob;
  // Load unstructured slam problem
  vslam_io::LoadStructurelessUTSLAMProblem(FLAGS_data_path, prob);
  // Make empty camera calibration matrix
  Eigen::Matrix3f K;
  // Load camera calibration matrix
  vslam_io::LoadCameraCalibration(FLAGS_calibration_path, K);

  // Print poses to terminal for display
  for (const auto& pose : prob.robot_poses) {
    cout << pose << endl;
  }

  // Print feature tracks to terminal for display
  for (const auto& ft : prob.tracks) {
    for (const auto& feature : ft.second.track) {
      cout << feature << endl;
    }
  }

  // Print camera intrinsics to terminal for display
  cout << K << endl;

  return 0;
}