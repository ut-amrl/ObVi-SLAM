#include <vslam_noise_util.h>

namespace vslam_util {

Eigen::Vector3d CorruptFeature(const Eigen::Vector3d &sigma_linear,
                               Eigen::Vector3d &feature_init_pose) {
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  // Corrupt linear dimension
  Eigen::Vector3d dl(sigma_linear.x() * distribution(generator),
                     sigma_linear.y() * distribution(generator),
                     sigma_linear.z() * distribution(generator));

  return feature_init_pose + dl;
}
}  // namespace vslam_util