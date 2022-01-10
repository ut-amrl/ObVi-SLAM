#include <glog/logging.h>
#include <vslam_util.h>

#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>

namespace vslam_util {

void SaveKITTIPoses(const std::string &filename,
                    const std::vector<vslam_types::RobotPose> &poses) {
  // Open output file
  std::ofstream f;
  f.open(filename);
  for (const auto &pose : poses) {
    Eigen::Matrix3f R = pose.angle.toRotationMatrix();
    f << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << pose.loc(0)
      << " " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
      << pose.loc(1) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2)
      << " " << pose.loc(2) << std::endl;
  }
  f.close();
}

void AdjustTrajectoryToStartAtZero(
    const std::vector<vslam_types::RobotPose> &original_trajectory,
    std::vector<vslam_types::RobotPose> &adjusted_trajectory) {
  adjusted_trajectory.clear();
  vslam_types::RobotPose new_init_pose(
      original_trajectory[0].frame_idx,
      Eigen::Vector3f(0, 0, 0),
      Eigen::AngleAxisf(Eigen::Quaternionf(1, 0, 0, 0)));
  adjusted_trajectory.emplace_back(new_init_pose);
  for (int i = 1; i < original_trajectory.size(); i++) {
    adjusted_trajectory.emplace_back(getPose2RelativeToPose1(
        original_trajectory[0], original_trajectory[i]));
  }
}

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

vslam_types::RobotPose getPose2RelativeToPose1(
    const vslam_types::RobotPose &pose_1,
    const vslam_types::RobotPose &pose_2) {
  Eigen::Affine3f pose_1_mat = pose_1.RobotToWorldTF();
  Eigen::Affine3f pose_2_mat = pose_2.RobotToWorldTF();

  Eigen::Affine3f pose_2_rel_to_1 = pose_1_mat.inverse() * pose_2_mat;

  vslam_types::RobotPose rel_pose;
  rel_pose.loc = pose_2_rel_to_1.translation();
  rel_pose.angle = Eigen::AngleAxisf(pose_2_rel_to_1.linear());

  return rel_pose;
}

Eigen::Vector3d getPositionRelativeToPose(const vslam_types::RobotPose &pose_1,
                                          const Eigen::Vector3d &position) {
  Eigen::Affine3f pose_1_mat = pose_1.RobotToWorldTF();

  Eigen::Vector3d transformed = pose_1_mat.inverse().cast<double>() * position;

  return transformed;
}

std::vector<vslam_types::OdometryFactor> getOdomFactorsFromInitPosesAndNoise(
    const std::vector<vslam_types::RobotPose> &initial_trajectory,
    const Eigen::Vector2d &odom_alphas) {
  std::vector<vslam_types::OdometryFactor> odom_factors;

  for (uint64_t i = 1; i < initial_trajectory.size(); i++) {
    vslam_types::RobotPose relative_pose = getPose2RelativeToPose1(
        initial_trajectory[i - 1], initial_trajectory[i]);

    Eigen::Matrix<double, 6, 6> odom_cov_mat =
        Eigen::Matrix<double, 6, 6>::Zero();

    odom_cov_mat(0, 0) = pow(relative_pose.loc.x() * odom_alphas(1), 2);
    odom_cov_mat(1, 1) = pow(relative_pose.loc.y() * odom_alphas(1), 2);
    odom_cov_mat(2, 2) = pow(relative_pose.loc.z() * odom_alphas(1), 2);
    odom_cov_mat(3, 3) =
        pow(relative_pose.angle.angle() * relative_pose.angle.axis().x() *
                odom_alphas(0),
            2);
    odom_cov_mat(4, 4) =
        pow(relative_pose.angle.angle() * relative_pose.angle.axis().y() *
                odom_alphas(0),
            2);
    odom_cov_mat(5, 5) =
        pow(relative_pose.angle.angle() * relative_pose.angle.axis().z() *
                odom_alphas(0),
            2);

    Eigen::Matrix<double, 6, 6> sqrt_information =
        odom_cov_mat.inverse().sqrt();

    vslam_types::OdometryFactor factor(i - 1,
                                       i,
                                       relative_pose.loc,
                                       Eigen::Quaternionf(relative_pose.angle),
                                       sqrt_information);
  }
  return odom_factors;
}

}  // namespace vslam_util