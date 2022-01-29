//
// Created by amanda on 1/10/22.
//

#ifndef UT_VSLAM_ELLIPSOID_UTILS_H
#define UT_VSLAM_ELLIPSOID_UTILS_H

#include <vslam_util.h>

#include <eigen3/Eigen/Dense>

namespace vslam_util {

/**
 * Constant added to dimension when computing the dual form for an ellipsoid.
 *
 * TODO experiment with this number: current value was chosen arbitrarily.
 */
const float kDimensionRegularizationConstant = 1e-3;

/**
 * Create the matrix representation (dual form) for the ellipsoid.
 *
 * The dual form is given as a 4x4 matrix with the upper 3x3 matrix given by
 * -R D transpose(R) - t transpose(t), where D is a diagonal matrix with the
 * dimension components plus a reguarlization term on the diagonal, the bottom
 * right corner as -1, and the bottom row and right column excluding the last
 * entry given by -transpose(t) and -t, respectively.
 *
 * @tparam T                Type of data representing fields of the ellipsoid
 *                          (needed so that this can be used during
 *                          optimization).
 * @param ellipsoid_data    Array containing ellipsoid data. Must be a 9 entry
 *                          vector with 3 for translation, 3 for angle-axis,
 *                          3 for dimension.
 *
 * @return Matrix representation for the ellipsoid.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> createDualRepresentationForEllipsoid(
    const T *ellipsoid_data) {
  Eigen::Matrix<T, 4, 4> dual_rep;
  dual_rep(3, 3) = T(-1);
  Eigen::Matrix<T, 3, 1> transl(
      ellipsoid_data[0], ellipsoid_data[1], ellipsoid_data[2]);
  Eigen::Matrix<T, 3, 1> neg_transl = T(-1) * transl;
  dual_rep.bottomLeftCorner(1, 3) = neg_transl.transpose();
  dual_rep.topRightCorner(3, 1) = neg_transl;

  Eigen::DiagonalMatrix<T, 3> diag_mat(
      pow(ellipsoid_data[6], 2) + T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[7], 2) + T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[8], 2) + T(kDimensionRegularizationConstant));

  Eigen::Matrix<T, 3, 3> rot_mat = Exp(Eigen::Matrix<T, 3, 1>(
      ellipsoid_data[3], ellipsoid_data[4], ellipsoid_data[5]));

  dual_rep.topLeftCorner(3, 3) = (rot_mat * diag_mat * (rot_mat.transpose())) +
                                 (neg_transl * transl.transpose());

  return dual_rep;
}
}  // namespace vslam_util

#endif  // UT_VSLAM_ELLIPSOID_UTILS_H
