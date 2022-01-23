//
// Created by amanda on 1/23/22.
//

#include <eigen3/Eigen/Dense>

namespace synthetic_problem {
Eigen::AngleAxisf createAngleAxisFromYaw(const float &yaw) {
  return Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
}
}  // namespace synthetic_problem