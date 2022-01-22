#include <vslam_util.h>

#include <fstream>

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
  rel_pose.frame_idx = pose_2.frame_idx;

  return rel_pose;
}

Eigen::Vector3d getPositionRelativeToPose(const vslam_types::RobotPose &pose_1,
                                          const Eigen::Vector3d &position) {
  Eigen::Affine3f pose_1_mat = pose_1.RobotToWorldTF();

  Eigen::Vector3d transformed = pose_1_mat.inverse().cast<double>() * position;

  return transformed;
}

vslam_types::RobotPose combinePoses(const vslam_types::RobotPose &pose_1,
                                    const vslam_types::RobotPose &pose_2) {
  Eigen::Affine3f pose_1_mat = pose_1.RobotToWorldTF();
  Eigen::Affine3f pose_2_mat = pose_2.RobotToWorldTF();

  Eigen::Affine3f pose_2_rel_to_world_mat = pose_1_mat * pose_2_mat;

  vslam_types::RobotPose pose_2_world;
  pose_2_world.loc = pose_2_rel_to_world_mat.translation();
  pose_2_world.angle = Eigen::AngleAxisf(pose_2_rel_to_world_mat.linear());
  pose_2_world.frame_idx = pose_2.frame_idx;

  return pose_2_world;
}

}  // namespace vslam_util