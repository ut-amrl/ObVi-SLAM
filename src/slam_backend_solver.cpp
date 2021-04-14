#include <ceres/ceres.h>
#include <pairwise_2d_feature_cost_functor.h>
#include <slam_backend_solver.h>

namespace vslam_solver {

void RobotPosesToSLAMNodes(
    const std::vector<vslam_types::RobotPose> &robot_poses,
    std::vector<SLAMNode> &nodes) {
  for (const vslam_types::RobotPose &robot_pose : robot_poses) {
    nodes.emplace_back(FromRobotPose(robot_pose));
  }
}

void SLAMNodesToRobotPoses(const std::vector<SLAMNode> &slam_nodes,
                           std::vector<vslam_types::RobotPose> &updated_poses) {
  updated_poses.clear();
  for (const SLAMNode &node : slam_nodes) {
    updated_poses.emplace_back(FromSLAMNode(node));
  }
}

SLAMNode FromRobotPose(const vslam_types::RobotPose &robot_pose) {
  return SLAMNode(robot_pose.frame_idx, robot_pose.loc, robot_pose.angle);
}

vslam_types::RobotPose FromSLAMNode(const SLAMNode &slam_node) {
  Eigen::Vector3f rotation_axis(
      slam_node.pose[3], slam_node.pose[4], slam_node.pose[5]);

  Eigen::AngleAxisf rotation_aa = vslam_types::VectorToAxisAngle(rotation_axis);

  Eigen::Vector3f transl(
      slam_node.pose[0], slam_node.pose[1], slam_node.pose[2]);
  return vslam_types::RobotPose(slam_node.node_idx, transl, rotation_aa);
}

bool SLAMSolver::SolveSLAM(
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::CameraExtrinsics &extrinsics,
    const vslam_types::UTSLAMProblem &slam_problem,
    std::vector<vslam_types::RobotPose> &updated_robot_poses) {
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // TODO configure options

  std::vector<SLAMNode> slam_nodes;
  RobotPosesToSLAMNodes(updated_robot_poses, slam_nodes);

  AddVisionFactors(slam_problem, intrinsics, extrinsics, problem, &slam_nodes);

  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  SLAMNodesToRobotPoses(slam_nodes, updated_robot_poses);

  return (summary.termination_type == ceres::CONVERGENCE ||
          summary.termination_type == ceres::USER_SUCCESS);
}

void SLAMSolver::AddVisionFactors(
    const vslam_types::UTSLAMProblem &slam_problem,
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::CameraExtrinsics &extrinsics,
    ceres::Problem &ceres_problem,
    std::vector<SLAMNode> *updated_solved_nodes) {
  std::vector<SLAMNode> &solution = *updated_solved_nodes;

  for (const auto &feature_track_by_id : slam_problem.tracks) {
    for (int i = 0; i < feature_track_by_id.second.track.size() - 1; i++) {
      vslam_types::VisionFeature f1 = feature_track_by_id.second.track[i];
      for (int j = i + 1; j < feature_track_by_id.second.track.size(); j++) {
        vslam_types::VisionFeature f2 = feature_track_by_id.second.track[j];

        double *initial_pose_block = solution[f1.frame_idx].pose;
        double *curr_pose_block = solution[f2.frame_idx].pose;

        ceres_problem.AddResidualBlock(
            Pairwise2dFeatureCostFunctor::create(
                intrinsics,
                extrinsics,
                f1,
                f2,
                solver_optimization_params_.epipolar_error_std_dev),
            nullptr,
            initial_pose_block,
            curr_pose_block);
      }
    }
  }
}
}  // namespace vslam_solver