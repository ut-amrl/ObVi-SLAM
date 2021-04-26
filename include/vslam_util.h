#ifndef UT_VSLAM_SLAM_UTIL_H
#define UT_VSLAM_SLAM_UTIL_H

#include "eigen3/Eigen/Dense"

namespace vslam_util {

template <typename T>
void CalcEssentialMatrix(Eigen::Transform<T, 3, Eigen::Affine> const& rel_tf,
                         Eigen::Matrix<T, 3, 3>* const essential_mat);

}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_UTIL_H