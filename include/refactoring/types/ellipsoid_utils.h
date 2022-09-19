//
// Created by amanda on 1/10/22.
//

#ifndef UT_VSLAM_REFACTORING_ELLIPSOID_UTILS_H
#define UT_VSLAM_REFACTORING_ELLIPSOID_UTILS_H

#include <refactoring/types/vslam_math_util.h>
#include <refactoring/types/vslam_types_math_util.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
using std::cout;
using std::endl;

namespace vslam_types_refactor {

namespace {
/**
 * Constant added to dimension when computing the dual form for an ellipsoid.
 *
 * TODO experiment with this number: current value was chosen arbitrarily.
 */
const float kDimensionRegularizationConstant = 1e-3;
}  // namespace

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
      pow(ellipsoid_data[6] / T(2), 2) + T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[7] / T(2), 2) + T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[8] / T(2), 2) + T(kDimensionRegularizationConstant));

  //  LOG(INFO) << "Ellipsoid dim\n" << ellipsoid_data[6] / T(2) << "\n"
  //            << ellipsoid_data[7] / T(2) << "\n" << ellipsoid_data[8] / T(2);

  Eigen::Matrix<T, 3, 3> rot_mat = Exp(Eigen::Matrix<T, 3, 1>(
      ellipsoid_data[3], ellipsoid_data[4], ellipsoid_data[5]));

  dual_rep.topLeftCorner(3, 3) = (rot_mat * diag_mat * (rot_mat.transpose())) +
                                 (neg_transl * transl.transpose());

  return dual_rep;
}

/**
 * Get the predicted bounding box for an ellipsoid observed by a camera on a
 * robot at the given robot pose.
 *
 * @tparam T                    Datatype for numbers to evaluate.
 * @param ellipsoid[in]         Estimate of the ellipsoid parameters. This is a
 *                              9 entry array with the first 3 entries
 *                              corresponding to the translation, the second 3
 *                              entries containing the axis-angle representation
 *                              (with angle given by the magnitude of the
 *                              vector), and the final 3 entries corresponding
 *                              to the dimensions of the ellipsoid.
 * @param robot_pose[in]        Robot's pose in the world frame corresponding to
 *                              the location of where the feature was imaged.
 *                              This is a 6 entry array with the first 3 entries
 *                              corresponding to the translation and the second
 *                              3 entries containing the axis-angle
 *                              representation (with angle given by the
 *                              magnitude of the vector).
 * @param robot_to_cam_tf[in]   Transform that provides the robot's position in
 *                              the camera frame (inverse of extrinsics, which
 *                              provide the camera's pose in the robot frame).
 * @param intrinsics[in]        Camera intrinsics.
 * @param corner_locations[out] Predicted corner locations based on the current
 *                              ellipsoid and robot pose estimates.
 */
template <typename T>
void getCornerLocationsVector(
    const T *ellipsoid,
    const T *robot_pose,
    const Eigen::Transform<T, 3, Eigen::Affine> &robot_to_cam_tf,
    const Eigen::Matrix<T, 3, 3> &intrinsics,
    Eigen::Matrix<T, 4, 1> &corner_results) {
  Eigen::Transform<T, 3, Eigen::Affine> robot_to_world_current =
      PoseArrayToAffine(&(robot_pose[3]), &(robot_pose[0]));
//  LOG(INFO) << "Robot pose " << robot_to_world_current.matrix();

  // Robot to world defines the robot's pose in the world frame
  // Cam to robot defines the camera pose in the robot's frame
  // We want the world's pose in the camera frame
  Eigen::Transform<T, 3, Eigen::Affine> world_to_camera =
      robot_to_cam_tf * robot_to_world_current.inverse();
  Eigen::Transform<T, 3, Eigen::AffineCompact> world_to_camera_compact =
      world_to_camera;
  Eigen::Matrix<T, 4, 4> ellipsoid_dual_rep =
      createDualRepresentationForEllipsoid(ellipsoid);

//  LOG(INFO) << "Ellipsoid dual rep " << ellipsoid_dual_rep;

  Eigen::Matrix<T, 3, 3> g_mat =
      intrinsics * world_to_camera_compact.matrix() * ellipsoid_dual_rep *
      world_to_camera_compact.matrix().transpose() * intrinsics.transpose();

//  LOG(INFO) << "G mat " << g_mat;
  T g1_1 = g_mat(0, 0);
  T g1_3 = g_mat(0, 2);
  T g2_2 = g_mat(1, 1);
  T g2_3 = g_mat(1, 2);
  T g3_3 = g_mat(2, 2);
  T x_sqrt_component = sqrt(pow(g1_3, 2) - (g1_1 * g3_3));
  T y_sqrt_component = sqrt(pow(g2_3, 2) - (g2_2 * g3_3));

//  LOG(INFO) << "x/y sqrt components " << x_sqrt_component << "; "
//            << y_sqrt_component;

  // TODO Verify once we have real data that these are in the right order?
  //  is min and max actually the min and max?
  //    Eigen::Matrix<T, 4, 1> corner_results(g1_3 - x_sqrt_component,
  //                                          g1_3 + x_sqrt_component,
  //                                          g2_3 - y_sqrt_component,
  //                                          g2_3 + y_sqrt_component);
  corner_results << g1_3 + x_sqrt_component, g1_3 - x_sqrt_component,
      g2_3 + y_sqrt_component, g2_3 - y_sqrt_component;
  corner_results = corner_results / g3_3;
}
template <typename NumType>
BbCornerPair<NumType> getCornerLocationsPair(
    const EllipsoidState<NumType> &ellispoid_est,
    const Pose3D<NumType> &robot_pose,
    const CameraExtrinsics<NumType> &cam_extrinsics,
    const CameraIntrinsicsMat<NumType> &cam_intrinsics) {
  BbCorners<NumType> corner_locations_raw;
  RawEllipsoid<NumType> raw_ellipsoid = convertToRawEllipsoid(ellispoid_est);
  NumType *ellipsoid_pose_ptr = raw_ellipsoid.data();
  RawPose3d<NumType> raw_robot_pose = convertPoseToArray(robot_pose);
  NumType *robot_pose_ptr = raw_robot_pose.data();

  Transform6Dof<NumType> robot_to_cam_tf =
      convertToAffine(cam_extrinsics).inverse();

  getCornerLocationsVector(ellipsoid_pose_ptr,
                           robot_pose_ptr,
                           robot_to_cam_tf,
                           cam_intrinsics,
                           corner_locations_raw);
  return cornerLocationsVectorToPair<NumType>(corner_locations_raw);
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_ELLIPSOID_UTILS_H
