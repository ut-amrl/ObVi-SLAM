#include "vslam_io.h"

#include <glog/logging.h>

#include <algorithm>
#include <experimental/filesystem>  //c++ 17 file iteration
#include <fstream>

namespace vslam_io {
using namespace vslam_types;

namespace fs = std::experimental::filesystem;

void LoadStructurelessUTSLAMProblem(
    std::string const& data_path,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>& prob) {
  // Iterate over all files/folders in the data_path directory - i.e. over all
  // frames
  for (const auto& entry : fs::directory_iterator(fs::path(data_path))) {
    const auto file_extension = entry.path().extension().string();

    // If it isn't a data file, skip it - we identify data files as
    // "regular files" with a .txt extension in the data_path directory
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }

    // If we haven't skipped it lets load it into the UTSLAMProblem
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "LoadUTSLAMProblem() failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return;
    }

    std::string line;

    // Read frame ID from 1st line
    std::getline(data_file_stream, line);
    std::stringstream ss_id(line);
    uint64_t frame_id;
    ss_id >> frame_id;

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
    prob.robot_poses.push_back(pose);

    // Read features from all other lines
    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      uint64_t feature_id;
      float x, y;
      ss_feature >> feature_id >> x >> y;
      VisionFeature feature(feature_id, frame_id, Eigen::Vector2f(x, y));
      prob.tracks[feature_id].track.push_back(feature);
    }
  }

  // Sort all feature tracks so frame_idxs are in ascending order
  for (auto& ft : prob.tracks) {
    std::sort(ft.second.track.begin(), ft.second.track.end());
  }

  return;
}

}  // namespace vslam_io