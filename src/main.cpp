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
  vslam_types::UTSLAMProblem<int> prob;
  // Load unstructured slam problem
  vslam_io::LoadUTSLAMProblem(FLAGS_data_path, &prob);

  // Print poses to terminal for display
  for (const auto& pose : prob.robot_poses) {
    cout << pose << endl;
  }

  // Print feature tracks to terminal for display
  for (const auto& track : prob.track_database.feature_tracks) {
    for (const auto& feature : track.getTrack()) {
      cout << feature << endl;
    }
  }

  return 0;
}