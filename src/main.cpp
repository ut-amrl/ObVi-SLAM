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

using std::cout;
using std::endl;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // Make empty unstructured slam problem
  vslam_types::UTSLAMProblem prob;
  // Load unstructured slam problem
  vslam_io::LoadUTSLAMProblem(FLAGS_data_path, prob);

  // Print poses to terminal for display
  for (const auto& pose : prob.robot_poses) {
    cout << pose << endl;
  }

  // Print feature tracks to terminal for display
  cout << prob.tracks.size() << endl;
  for (const auto& ft : prob.tracks) {
    for (const auto& feature : ft.second.track) {
      cout << feature << endl;
    }
  }
  return 0;
}