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
  vslam_types::RobotPose new_init_pose;
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

  return rel_pose;
}

}  // namespace vslam_util