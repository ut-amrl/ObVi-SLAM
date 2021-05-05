#include <vslam_util.h>

#include <fstream>

namespace vslam_util {

template <typename T>
void CalcEssentialMatrix(const Eigen::Transform<T, 3, Eigen::Affine>& rel_tf,
                         const Eigen::Matrix<T, 3, 3>& essential_mat) {
  // Extract Tx and R rel_tf (rel_tf aka "cam 1 to cam 2 tf")
  Eigen::Matrix<T, 3, 1> t_vec = rel_tf.translation();
  Eigen::Matrix<T, 3, 3> t_cross;  // skew symmetric
  t_cross << T(0.0), -t_vec(2), t_vec(1), t_vec(2), T(0.0), -t_vec(0),
      -t_vec(1), t_vec(0), T(0.0);
  Eigen::Matrix<T, 3, 3> rotation = rel_tf.linear();

  *essential_mat = t_cross * rotation;

  return;
}

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