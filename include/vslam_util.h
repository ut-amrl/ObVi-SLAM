#ifndef UT_VSLAM_SLAM_UTIL_H
#define UT_VSLAM_SLAM_UTIL_H

#include "eigen3/Eigen/Dense"

namespace vslam_util {

/**
 * Calculate the essential matrix given a relative transform
 *
 * @param rel_tf[in]    Relative transfrom between two poses
 *
 * @return  Essential matrix corresponding to the realtive transform
 */
template <typename T>
void CalcEssentialMatrix(const Eigen::Transform<T, 3, Eigen::Affine>& rel_tf,
                         const Eigen::Matrix<T, 3, 3>& essential_mat);

}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_UTIL_H