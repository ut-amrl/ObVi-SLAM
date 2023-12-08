//
// Created by amanda on 1/10/22.
//

#ifndef UT_VSLAM_REFACTORING_ELLIPSOID_UTILS_H
#define UT_VSLAM_REFACTORING_ELLIPSOID_UTILS_H

#include <refactoring/types/vslam_math_util.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>
#include <refactoring/types/vslam_types_math_util.h>

#include <eigen3/Eigen/Dense>

namespace vslam_types_refactor {

namespace {
/**
 * Constant added to dimension when computing the dual form for an ellipsoid.
 *
 * TODO experiment with this number: current value was chosen arbitrarily.
 */
const float kDimensionRegularizationConstant = 1e-3;
}  // namespace

template <typename T>
void getEllipsoidDimMatAndTf(
    const T *ellipsoid_data,
    Eigen::Matrix<T, 4, 4> &dim_mat,
    Eigen::Transform<T, 3, Eigen::Affine> &ellipsoid_pose) {
  Eigen::Translation<T, 3> transl(
      ellipsoid_data[0], ellipsoid_data[1], ellipsoid_data[2]);

  Eigen::Matrix<T, 4, 1> diag_mat(
      pow(ellipsoid_data[kEllipsoidPoseParameterizationSize] / T(2), 2) +
          T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[kEllipsoidPoseParameterizationSize + 1] / T(2), 2) +
          T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[kEllipsoidPoseParameterizationSize + 2] / T(2), 2) +
          T(kDimensionRegularizationConstant),
      T(-1));
  dim_mat = diag_mat.asDiagonal();

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  Eigen::AngleAxis<T> axis_angle =
      Eigen::AngleAxis<T>(ellipsoid_data[3], Eigen::Matrix<T, 3, 1>::UnitZ());
#else
  const Eigen::Matrix<T, 3, 1> rotation_axis(
      ellipsoid_data[3], ellipsoid_data[4], ellipsoid_data[5]);
  Eigen::AngleAxis<T> axis_angle = VectorToAxisAngle(rotation_axis);
#endif

  ellipsoid_pose = transl * Eigen::Quaternion<T>(axis_angle);
}

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
      pow(ellipsoid_data[kEllipsoidPoseParameterizationSize] / T(2), 2) +
          T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[kEllipsoidPoseParameterizationSize + 1] / T(2), 2) +
          T(kDimensionRegularizationConstant),
      pow(ellipsoid_data[kEllipsoidPoseParameterizationSize + 2] / T(2), 2) +
          T(kDimensionRegularizationConstant));

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  Eigen::Matrix<T, 3, 3> rot_mat =
      Eigen::Quaternion<T>(Eigen::AngleAxis<T>(ellipsoid_data[3],
                                               Eigen::Matrix<T, 3, 1>::UnitZ()))
          .matrix();
#else
  Eigen::Matrix<T, 3, 3> rot_mat = Exp(Eigen::Matrix<T, 3, 1>(
      ellipsoid_data[3], ellipsoid_data[4], ellipsoid_data[5]));
#endif
  dual_rep.topLeftCorner(3, 3) = (rot_mat * diag_mat * (rot_mat.transpose())) +
                                 (neg_transl * transl.transpose());
  return dual_rep;
}

template <typename T>
Eigen::Matrix<T, 4, 4> createDualRepresentationForFullDOFEllipsoid(
    const FullDOFEllipsoidState<T> &ellipsoid_state) {
  Eigen::DiagonalMatrix<T, 3> diag_mat(
      pow(ellipsoid_state.dimensions_.x() / T(2), 2) +
          T(kDimensionRegularizationConstant),
      pow(ellipsoid_state.dimensions_.y() / T(2), 2) +
          T(kDimensionRegularizationConstant),
      pow(ellipsoid_state.dimensions_.z() / T(2), 2) +
          T(kDimensionRegularizationConstant));

  Eigen::Matrix<T, 3, 3> rot_mat =
      Eigen::Quaternion<T>(ellipsoid_state.pose_.orientation_).matrix();

  Eigen::Matrix<T, 4, 4> dual_rep;
  dual_rep(3, 3) = T(-1);
  Eigen::Matrix<T, 3, 1> transl = ellipsoid_state.pose_.transl_;
  Eigen::Matrix<T, 3, 1> neg_transl = T(-1) * transl;
  dual_rep.bottomLeftCorner(1, 3) = neg_transl.transpose();
  dual_rep.topRightCorner(3, 1) = neg_transl;

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
bool getCornerLocationsVectorRectified(
    const T *ellipsoid,
    const T *robot_pose,
    const Eigen::Transform<T, 3, Eigen::Affine> &robot_to_cam_tf,
    Eigen::Matrix<T, 4, 1> &corner_results) {
  Eigen::Transform<T, 3, Eigen::Affine> robot_to_world_current =
      PoseArrayToAffine(&(robot_pose[3]), &(robot_pose[0]));

  // Robot to world defines the robot's pose in the world frame
  // Cam to robot defines the camera pose in the robot's frame
  // We want the world's pose in the camera frame
  Eigen::Transform<T, 3, Eigen::Affine> world_to_camera =
      robot_to_cam_tf * robot_to_world_current.inverse();

  // Get the ellipsoid's pose in the world frame as a tf
  Eigen::Matrix<T, 4, 4> dim_mat;
  Eigen::Transform<T, 3, Eigen::Affine> ellipsoid_pose;

  getEllipsoidDimMatAndTf(ellipsoid, dim_mat, ellipsoid_pose);

  Eigen::Transform<T, 3, Eigen::AffineCompact> combined_tf_compact =
      world_to_camera * ellipsoid_pose;

  Eigen::Matrix<T, 3, 3> q_mat = combined_tf_compact.matrix() * dim_mat *
                                 combined_tf_compact.matrix().transpose();

//  // Check if behind camera
//  T t_z = combined_tf_compact.translation().z();
//  T q33_adjusted_sqrt = sqrt(q_mat(2, 2) + pow(t_z, 2));

//  T plus_tan_z_plane = t_z + q33_adjusted_sqrt;
//  T min_tan_z_plane = t_z - q33_adjusted_sqrt;
//  if ((plus_tan_z_plane < T(0)) && (min_tan_z_plane < T(0))) {
//    LOG(WARNING)
//        << "Ellipsoid fully behind camera plane. Not physically possible.";
//    // TODO should we sleep or exit here to make this error more evident?
//  }

  T q1_1 = q_mat(0, 0);
  T q1_3 = q_mat(0, 2);
  T q2_2 = q_mat(1, 1);
  T q2_3 = q_mat(1, 2);
  T q3_3 = q_mat(2, 2);

  T x_inner_sqrt = pow(q1_3, 2) - (q1_1 * q3_3);
  T y_inner_sqrt = pow(q2_3, 2) - (q2_2 * q3_3);
  if ((x_inner_sqrt <= T(0)) || (y_inner_sqrt <= T(0))) {
    return false;
  }
  T x_sqrt_component = sqrt(x_inner_sqrt);
  T y_sqrt_component = sqrt(y_inner_sqrt);

  // TODO Verify once we have real data that these are in the right order?
  //  is min and max actually the min and max?
  //    Eigen::Matrix<T, 4, 1> corner_results(q1_3 - x_sqrt_component,
  //                                          q1_3 + x_sqrt_component,
  //                                          q2_3 - y_sqrt_component,
  //                                          q2_3 + y_sqrt_component);
  corner_results << q1_3 + x_sqrt_component, q1_3 - x_sqrt_component,
      q2_3 + y_sqrt_component, q2_3 - y_sqrt_component;
  corner_results = corner_results / q3_3;
  return true;
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
bool getCornerLocationsVector(
    const T *ellipsoid,
    const T *robot_pose,
    const Eigen::Transform<T, 3, Eigen::Affine> &robot_to_cam_tf,
    const Eigen::Matrix<T, 3, 3> &intrinsics,
    Eigen::Matrix<T, 4, 1> &corner_results) {
  Eigen::Matrix<T, 4, 1> rect_corner_results;
  if (!getCornerLocationsVectorRectified(
          ellipsoid, robot_pose, robot_to_cam_tf, rect_corner_results)) {
    return false;
  }

  Eigen::Matrix<T, 4, 1> adjust_center(
      intrinsics(0, 2), intrinsics(0, 2), intrinsics(1, 2), intrinsics(1, 2));
  Eigen::Matrix<T, 4, 1> adjust_scale(
      intrinsics(0, 0), intrinsics(0, 0), intrinsics(1, 1), intrinsics(1, 1));
  corner_results =
      (adjust_scale.asDiagonal() * rect_corner_results) + adjust_center;

  return true;
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

inline bool pixelInBoundingBoxClosedSet(const BbCornerPair<double> &bb,
                                        const PixelCoord<double> &pixel) {
  return ((bb.first.x() <= pixel.x()) && (bb.first.y() <= pixel.y()) &&
          (bb.second.x() >= pixel.x()) && (bb.second.y() >= pixel.y()));
}

template <typename NumType>
BbCornerPair<NumType> inflateBoundingBox(
    const BbCornerPair<NumType> &original_bb, const NumType &inflation_size) {
  return std::make_pair(
      PixelCoord<double>(original_bb.first.x() - inflation_size,
                         original_bb.first.y() - inflation_size),
      PixelCoord<double>(original_bb.second.x() + inflation_size,
                         original_bb.second.y() + inflation_size));
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_ELLIPSOID_UTILS_H
