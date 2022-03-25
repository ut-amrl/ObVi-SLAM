#ifndef UT_VSLAM_SLAM_NOISE_UTIL_H
#define UT_VSLAM_SLAM_NOISE_UTIL_H

#include <glog/logging.h>
#include <vslam_types.h>

#include <eigen3/Eigen/Dense>
#include <random>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace vslam_util {
/**
 * Perturbs a rotation matrix by the noise dw represented in so(3)
 *
 * @param dw[in]        Perturbation in so(3)
 * @param R[in,out]     Perturbed rotation in SO(3)
 */
template <typename T>
void PerturbRotationMatrix(const Eigen::Matrix<T, 3, 1>& dw,
                           Eigen::Matrix<T, 3, 3>& R) {
  // R_perturb = Exp(Log(R) + dw)
  //           = Exp(w + dw)
  //           = Exp(w)*Exp(J_r(w) * dw)
  //           = R * Exp(J_r(Log(R)) * dw)
  Eigen::Matrix<T, 3, 1> J_dw =
      GetRodriguesJacobian(Log(R)) * dw;  // TODO remove this intermediate step
  R = R * Exp(J_dw);
}

/**
 * Add noise to robot poses - Can be used to corrupt groundtruth poses into
 * imperfect odometry measuremets - simulates odometry!
 *
 * @param odom_alphas[in]   Odom alpha 1 and 3 as described :
 * https://wiki.ros.org/amcl
 * @param poses[out]   Ground truth robot poses to add noise to
 */
template <typename T>
void CorruptRobotPoses(const Eigen::Matrix<T, 2, 1>& odom_alphas,
                       std::vector<vslam_types::RobotPose>& poses) {
  std::default_random_engine generator;
  std::normal_distribution<T> distribution(0.0, 1.0);

  std::vector<vslam_types::RobotPose> gt_poses = poses;
  for (int i = 1; i < gt_poses.size(); ++i) {
    // Frame 1
    Eigen::Transform<float, 3, Eigen::Affine> first_robot_pose_in_world =
        Eigen::Translation<float, 3>(gt_poses[i - 1].loc) *
        gt_poses[i - 1].angle;
    // Frame 2
    Eigen::Transform<float, 3, Eigen::Affine> second_robot_pose_in_world =
        Eigen::Translation<float, 3>(gt_poses[i].loc) * gt_poses[i].angle;
    // Relative transform from frame 2 to 1
    Eigen::Transform<float, 3, Eigen::Affine> rel_tf =
        first_robot_pose_in_world.inverse() * second_robot_pose_in_world;

    // Corrupt linear dimension proportional to linear displacement
    Eigen::Matrix<float, 3, 1> dl(fabs(rel_tf.translation().x()) *
                                      odom_alphas(1) * distribution(generator),
                                  fabs(rel_tf.translation().y()) *
                                      odom_alphas(1) * distribution(generator),
                                  fabs(rel_tf.translation().z()) *
                                      odom_alphas(1) * distribution(generator));
    rel_tf.translation() += dl;
    // Corrupt orientation dimension
    Eigen::AngleAxis<T> rel_rotation(rel_tf.linear().cast<T>());
    Eigen::Matrix<T, 3, 1> dw(
        fabs(rel_rotation.angle() * rel_rotation.axis().x()) * odom_alphas(0) *
            distribution(generator),
        fabs(rel_rotation.angle() * rel_rotation.axis().y()) * odom_alphas(0) *
            distribution(generator),
        fabs(rel_rotation.angle() * rel_rotation.axis().z()) * odom_alphas(0) *
            distribution(generator));
    Eigen::Matrix<T, 3, 3> R = rel_tf.linear().cast<T>();
    PerturbRotationMatrix(dw, R);
    rel_tf.linear() = R.template cast<float>();

    // Out most recent noise pose
    Eigen::Transform<float, 3, Eigen::Affine> corrupted_robot_pose_in_world =
        Eigen::Translation<float, 3>(poses[i - 1].loc) * poses[i - 1].angle;

    // Add noisy relative tf to our most recent noisy pose
    Eigen::Transform<float, 3, Eigen::Affine>
        next_corrupted_robot_pose_in_world =
            corrupted_robot_pose_in_world * rel_tf;

    poses[i].loc = next_corrupted_robot_pose_in_world.translation();
    poses[i].angle = next_corrupted_robot_pose_in_world.linear();
  }
}

/**
 * Corrupt the feature pose by drawing from a gaussian centered around the
 * feature pose.
 *
 * @param sigma_linear          Standard deviations for the gaussian.
 * @param feature_init_pose     Initial feature pose (uncorrupted).
 *
 * @return Corrupted feature pose.
 */
Eigen::Vector3d CorruptFeature(const Eigen::Vector3d& sigma_linear,
                               Eigen::Vector3d& feature_init_pose);

}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_NOISE_UTIL_H