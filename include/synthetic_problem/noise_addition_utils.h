//
// Created by amanda on 1/22/22.
//

#ifndef UT_VSLAM_NOISE_ADDITION_UTILS_H
#define UT_VSLAM_NOISE_ADDITION_UTILS_H

#include <shared/util/random.h>
#include <vslam_types.h>

namespace synthetic_problem {

/**
 * Add Gaussian noise to the vector assuming a diagonal covariance matrix.
 *
 * @tparam VecDim Dimension of the vector to add noise to.
 * @param orig_vec      Vector to add noise to.
 * @param std_devs      Standard deviations (squared, these would form the
 * diagonal entries of the covariance matrix).
 * @param random_gen    Random number generator.
 *
 * @return Vector with Gaussian noise added to it.
 */
template <int VecDim>
Eigen::Matrix<float, VecDim, 1> addNoiseToVectorDiagonalCov(
    const Eigen::Matrix<float, VecDim, 1> &orig_vec,
    const Eigen::Matrix<float, VecDim, 1> &std_devs,
    util_random::Random &random_gen) {
  Eigen::Matrix<float, VecDim, 1> dim_noise;
  for (int i = 0; i < orig_vec.rows(); i++) {
    dim_noise(i, 0) = random_gen.Gaussian(0, std_devs(i));
  }
  return orig_vec + dim_noise;
}

/**
 * Add noise to the angle-axis represented orientation assuming a diagonal
 * covariance matrix.
 *
 * @param orig_val      Original orientation (expressed as axis-angle).
 * @param std_devs      Standard deviation for the components of the
 * orientation.
 * @param random_gen    Random number generator.
 *
 * @return Orientation with noise added.
 */
Eigen::AngleAxisf addNoiseToAngleAxisDiagonalCov(
    const Eigen::AngleAxisf &orig_val,
    const Eigen::Vector3f &std_devs,
    util_random::Random &random_gen);

/**
 * Add noise to the ellipsoid estimate.
 *
 * @param ellipsoid_est         Ellipsoid estimate to add noise to.
 * @param standard_deviations   Standard deviation for the ellipsoid estimate.
 *                              The first 3 entries provide the translation
 *                              noise, the second 3 provide the orientation
 *                              noise, and the last 3 provide the dimension
 *                              noise.
 * @param random_gen            Random number generator.
 *
 * @return Noisy ellipsoid estimate.
 */
vslam_types::EllipsoidEstimate addNoiseToEllipsoidEstimate(
    const vslam_types::EllipsoidEstimate &ellipsoid_est,
    const Eigen::Matrix<float, 9, 1> &standard_deviations,
    util_random::Random &random_gen);

/**
 * Apply noise to a relative pose, proportional to the magnitude of the pose
 * difference.
 *
 * @param pose_to_corrupt   Pose change (ex. odometry from t-1 to t) to add
 *                          noise to.
 * @param std_devs          Standard deviation for each component of the pose.
 * @param random_gen        Random number generator.
 *
 * @return Relative pose with added noise proportional to the magnitude of the
 * pose change.
 */
vslam_types::RobotPose applyNoiseToRelativePoseWithScaledMagnitude(
    const vslam_types::RobotPose &pose_to_corrupt,
    const Eigen::Matrix<float, 6, 1> &std_devs,
    util_random::Random &random_gen);

/**
 * Generate a single random noisy pose. Magnitude of the pose does not impact
 * the noise applied.
 *
 * @param std_devs      Standard deviation for each component of the pose.
 * @param random_gen    Random number generator.
 *
 * @return Pose with noise added.
 */
vslam_types::RobotPose generateRandomNoisyPose(
    const Eigen::Matrix<float, 6, 1> &std_devs,
    util_random::Random &random_gen);

/**
 * Add noise to an entire trajectory by corrupting the relative poses between
 * subsequent poses and then reconstructing the trajectory from the relative
 * poses.
 *
 * @param orig_trajectory       Trajectory to corrupt.
 * @param odometry_std_devs     Standard deviation to apply to relative poses
 *                              (scaled by the magnitude of the relative pose).
 * @param random_gen            Random number generator.
 *
 * @return Trajectory with added noise.
 */
std::vector<vslam_types::RobotPose> addOdomNoiseToTrajectory(
    const std::vector<vslam_types::RobotPose> &orig_trajectory,
    const Eigen::Matrix<float, 6, 1> &odometry_std_devs,
    util_random::Random &random_gen);

}  // namespace synthetic_problem

#endif  // UT_VSLAM_NOISE_ADDITION_UTILS_H
