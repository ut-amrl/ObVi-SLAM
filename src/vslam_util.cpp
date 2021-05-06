#include <vslam_util.h>

#include <fstream>

namespace vslam_util {

void SaveKITTIPoses(const std::string& filename,
                    const std::vector<vslam_types::RobotPose>& poses) {
  // Open output file
  std::ofstream f;
  f.open(filename);
  for (const auto& pose : poses) {
    Eigen::Matrix3f R = pose.angle.toRotationMatrix();
    f << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << pose.loc(0)
      << " " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
      << pose.loc(1) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2)
      << " " << pose.loc(2) << std::endl;
  }
  f.close();
}

}  // namespace vslam_util