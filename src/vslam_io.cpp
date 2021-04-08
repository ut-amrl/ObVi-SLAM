#include "vslam_io.h"

#include <glog/logging.h>

#include <filesystem>  //c++ 17 file iteration
#include <fstream>

namespace vslam_io {

using namespace vslam_types;

namespace fs = std::filesystem;

void LoadUTSLAMProblem(const std::string data_path,
                       vslam_types::UTSLAMProblem<int>* const prob_ptr) {
  if (!prob_ptr) {
    LOG(FATAL) << "LoadUTSLAMProblem() passed a bad pointer -_-";
    return;  // Exit early
  }
  vslam_types::UTSLAMProblem<int>& prob = *prob_ptr;

  // Iterate over all files/folders in the data_path directory - i.e. over all
  // frames
  for (const auto& entry : fs::directory_iterator(fs::path(data_path))) {
    const auto file_extension = entry.path().extension().string();
    // If it isn't a data file file skip it - we identify data files as "regular
    // files" with a .txt extension in the dat_path directory
    if (!entry.is_regular_file() || file_extension != ".txt") {
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
    // Read frame/robotpose from 2nd line
    std::getline(data_file_stream, line);
    std::stringstream ss_pose(line);
    float x, y, z, qx, qy, qz, qw;
    ss_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Vector3f loc(x, y, z);
    Eigen::Quaternionf angle_q(qw, qx, qy, qz);
    angle_q.normalize();
    Eigen::AngleAxisf angle(angle_q);
    RobotPose pose(frame_id, loc, angle);
    // Read features from all other lines
    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      uint64_t feature_id;
      float x, y;
      ss_feature >> feature_id >> x >> y;
      VisionFeature<int> feature(
          feature_id, frame_id, 1111, Eigen::Vector2f(x, y));
      std::cout << feature << std::endl;
    }
  }
}
}  // namespace vslam_io