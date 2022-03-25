//
// Created by amanda on 1/22/22.
//

#include <glog/logging.h>
#include <synthetic_problem/noise_addition_utils.h>
#include <vslam_math_util.h>
#include <vslam_types_math_util.h>
using namespace vslam_util;

namespace synthetic_problem {

Eigen::AngleAxisf addNoiseToAngleAxisDiagonalCov(
    const Eigen::AngleAxisf &orig_val,
    const Eigen::Vector3f &std_devs,
    util_random::Random &random_gen) {
  Eigen::Vector3f rotation_noise_vec = addNoiseToVectorDiagonalCov(
      Eigen::Vector3f(0, 0, 0), std_devs, random_gen);

  Eigen::AngleAxisf rotation_aa = VectorToAxisAngle(rotation_noise_vec);

  return Eigen::AngleAxisf(orig_val * rotation_aa);
}

vslam_types::EllipsoidEstimate addNoiseToEllipsoidEstimate(
    const vslam_types::EllipsoidEstimate &ellipsoid_est,
    const Eigen::Matrix<float, 9, 1> &standard_deviations,
    util_random::Random &random_gen) {
  Eigen::Vector3f new_loc =
      addNoiseToVectorDiagonalCov(ellipsoid_est.loc,
                                  Eigen::Vector3f(standard_deviations(0, 0),
                                                  standard_deviations(1, 0),
                                                  standard_deviations(2, 0)),
                                  random_gen);

  Eigen::AngleAxisf new_orientation =
      addNoiseToAngleAxisDiagonalCov(ellipsoid_est.orientation,
                                     Eigen::Vector3f(standard_deviations(3, 0),
                                                     standard_deviations(4, 0),
                                                     standard_deviations(5, 0)),
                                     random_gen);

  Eigen::Vector3f new_dim =
      addNoiseToVectorDiagonalCov(ellipsoid_est.ellipsoid_dim,
                                  Eigen::Vector3f(standard_deviations(6, 0),
                                                  standard_deviations(7, 0),
                                                  standard_deviations(8, 0)),
                                  random_gen);

  return vslam_types::EllipsoidEstimate(new_loc,
                                        new_orientation,
                                        new_dim,
                                        ellipsoid_est.semantic_class,
                                        ellipsoid_est.ellipsoid_idx);
}

vslam_types::RobotPose applyNoiseToRelativePoseWithScaledMagnitude(
    const vslam_types::RobotPose &pose_to_corrupt,
    const Eigen::Matrix<float, 6, 1> &std_devs,
    util_random::Random &random_gen) {
  Eigen::Vector3f unscaled_noise(random_gen.Gaussian(0, std_devs(0)),
                                 random_gen.Gaussian(0, std_devs(1)),
                                 random_gen.Gaussian(0, std_devs(2)));
  Eigen::Vector3f scaled_noise =
      (pose_to_corrupt.loc.array() * unscaled_noise.array()).matrix();
  Eigen::Vector3f new_transl = pose_to_corrupt.loc + scaled_noise;

  // TODO is this right?
  Eigen::Vector3f unscaled_orientation_noise_vec = addNoiseToVectorDiagonalCov(
      Eigen::Vector3f(0, 0, 0),
      Eigen::Vector3f(std_devs(3, 0), std_devs(4, 0), std_devs(5, 0)),
      random_gen);
  float angle = pose_to_corrupt.angle.angle();
  Eigen::Vector3f axis = pose_to_corrupt.angle.axis();
  Eigen::Vector3f scaled_orientation_noise_vec =
      (angle * axis.array() * unscaled_orientation_noise_vec.array()).matrix();

  Eigen::AngleAxisf scaled_orientation_noise =
      VectorToAxisAngle(scaled_orientation_noise_vec);

  Eigen::AngleAxisf new_orientation =
      Eigen::AngleAxisf(pose_to_corrupt.angle * scaled_orientation_noise);

  return vslam_types::RobotPose(
      pose_to_corrupt.frame_idx, new_transl, new_orientation);
}

vslam_types::RobotPose generateRandomNoisyPose(
    const Eigen::Matrix<float, 6, 1> &std_devs,
    util_random::Random &random_gen) {
  Eigen::Vector3f new_loc = addNoiseToVectorDiagonalCov(
      Eigen::Vector3f(0, 0, 0),
      Eigen::Vector3f(std_devs(0, 0), std_devs(1, 0), std_devs(2, 0)),
      random_gen);

  Eigen::AngleAxisf new_orientation = addNoiseToAngleAxisDiagonalCov(
      Eigen::AngleAxisf(0, Eigen::Vector3f(0, 0, 0)),
      Eigen::Vector3f(std_devs(3, 0), std_devs(4, 0), std_devs(5, 0)),
      random_gen);

  // TODO need to make sure frame id isn't used -- should probably find
  // representation for 3D pose that doesn't have a frame id
  return vslam_types::RobotPose(0, new_loc, new_orientation);
}

std::vector<vslam_types::RobotPose> addOdomNoiseToTrajectory(
    const std::vector<vslam_types::RobotPose> &orig_trajectory,
    const Eigen::Matrix<float, 6, 1> &odometry_std_devs,
    util_random::Random &random_gen) {
  std::vector<vslam_types::RobotPose> new_traj;
  new_traj.emplace_back(orig_trajectory[0]);
  for (int i = 1; i < orig_trajectory.size(); i++) {
    vslam_types::RobotPose noise_free_relative_pose =
        vslam_util::getPose2RelativeToPose1(orig_trajectory[i - 1],
                                            orig_trajectory[i]);
    vslam_types::RobotPose noisy_relative_pose =
        applyNoiseToRelativePoseWithScaledMagnitude(
            noise_free_relative_pose, odometry_std_devs, random_gen);
    vslam_types::RobotPose new_pose =
        vslam_util::combinePoses(new_traj.back(), noisy_relative_pose);
    new_traj.emplace_back(new_pose);
  }
  return new_traj;
}

}  // namespace synthetic_problem
