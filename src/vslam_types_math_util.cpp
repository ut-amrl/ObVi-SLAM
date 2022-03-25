#include <vslam_types_math_util.h>

namespace vslam_util {

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