#include <bounding_box_factor.h>
#include <ceres/ceres.h>
#include <object_slam_backend_solver.h>
#include <shape_prior_factor.h>
#include <slam_backend_solver.h>

namespace vslam_solver {

vslam_types::EllipsoidEstimateNode FromEllipsoidEstimate(
    const vslam_types::EllipsoidEstimate &ellipsoid_estimate) {
  return vslam_types::EllipsoidEstimateNode(ellipsoid_estimate.loc,
                                            ellipsoid_estimate.orientation,
                                            ellipsoid_estimate.ellipsoid_dim,
                                            ellipsoid_estimate.semantic_class);
}

vslam_types::EllipsoidEstimate FromEllipsoidNode(
    const vslam_types::EllipsoidEstimateNode &ellipsoid_node) {
  Eigen::Vector3f rotation_axis(
      ellipsoid_node.pose[3], ellipsoid_node.pose[4], ellipsoid_node.pose[5]);

  Eigen::AngleAxisf rotation_aa = vslam_types::VectorToAxisAngle(rotation_axis);

  Eigen::Vector3f transl(
      ellipsoid_node.pose[0], ellipsoid_node.pose[1], ellipsoid_node.pose[2]);
  Eigen::Vector3f dim(
      ellipsoid_node.pose[6], ellipsoid_node.pose[7], ellipsoid_node.pose[8]);
  return vslam_types::EllipsoidEstimate(
      transl, rotation_aa, dim, ellipsoid_node.semantic_class);
}

void EllipsoidEstimatesToNodes(
    const std::vector<vslam_types::EllipsoidEstimate> &ellispoid_estimates,
    std::vector<vslam_types::EllipsoidEstimateNode> &nodes) {
  for (const vslam_types::EllipsoidEstimate &estimate : ellispoid_estimates) {
    nodes.emplace_back(FromEllipsoidEstimate(estimate));
  }
}

void EllipsoidNodesToEllipsoidEstimates(
    const std::vector<vslam_types::EllipsoidEstimateNode> &ellipsoid_nodes,
    std::vector<vslam_types::EllipsoidEstimate> &updated_estimates) {
  updated_estimates.clear();
  for (const vslam_types::EllipsoidEstimateNode &node : ellipsoid_nodes) {
    updated_estimates.emplace_back(FromEllipsoidNode(node));
  }
}

template <typename FeatureTrackType>
void AddEllipsoidFactors(
    const ObjectSlamProblemParams &object_slam_params,
    vslam_types::UTObjectSLAMProblem<FeatureTrackType> &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *robot_pose_nodes_ptr,
    std::vector<vslam_types::EllipsoidEstimateNode> *ellipsoid_nodes_ptr) {
  std::vector<vslam_types::EllipsoidEstimateNode> &ellipsoid_nodes =
      *ellipsoid_nodes_ptr;
  std::vector<vslam_types::SLAMNode> &robot_pose_nodes = *robot_pose_nodes_ptr;
  for (vslam_types::EllipsoidEstimateNode &ellipsoid_node : ellipsoid_nodes) {
    double *ellipsoid_params_block = ellipsoid_node.pose;

    ceres_problem.AddResidualBlock(
        ShapePriorFactor::createUsingSemanticClassMapping(
            ellipsoid_node,
            object_slam_params.semantic_shape_prior_params
                .mean_and_cov_by_semantic_class),
        new ceres::HuberLoss(
            object_slam_params.semantic_shape_prior_params.huber_loss_param),
        ellipsoid_params_block);
  }

  for (const vslam_types::ObjectImageBoundingBoxDetection &bounding_box :
       slam_problem.bounding_boxes) {
    vslam_types::EllipsoidEstimateNode observed_ellipsoid =
        ellipsoid_nodes[bounding_box.ellipsoid_idx];
    vslam_types::SLAMNode observed_at_node =
        robot_pose_nodes[bounding_box.frame_idx];

    vslam_types::CameraExtrinsics extrinsics =
        slam_problem.camera_extrinsics_by_camera[bounding_box.camera_id];
    vslam_types::CameraIntrinsics intrinsics =
        slam_problem.camera_instrinsics_by_camera[bounding_box.camera_id];

    ceres_problem.AddResidualBlock(
        BoundingBoxFactor::createBoundingBoxFactor(
            bounding_box,
            intrinsics,
            extrinsics,
            object_slam_params.ellipsoid_bounding_box_constraint_params
                .bounding_box_covariance),
        new ceres::HuberLoss(
            object_slam_params.ellipsoid_bounding_box_constraint_params
                .huber_loss_param),
        observed_ellipsoid.pose,
        observed_at_node.pose);
  }
}

// TODO make all class methods lower case ?
template <typename FeatureTrackType, typename ProblemParams>
bool ObjectSLAMSolver::SolveObjectSLAM(
    const bool &use_pom,
    const bool &hold_robot_poses_constant,
    const std::function<
        void(const ProblemParams &,
             vslam_types::UTObjectSLAMProblem<FeatureTrackType> &,
             ceres::Problem &,
             std::vector<vslam_types::SLAMNode> *)> vision_constraint_adder,
    const std::function<
        void(const ProblemParams &,
             vslam_types::UTObjectSLAMProblem<FeatureTrackType> &,
             ceres::Problem &,
             std::vector<vslam_types::SLAMNode> *,
             std::vector<vslam_types::EllipsoidEstimateNode> *)>
        ellipsoid_constraint_creator,
    const std::function<std::shared_ptr<ceres::IterationCallback>(
        const vslam_types::UTObjectSLAMProblem<FeatureTrackType> &,
        std::vector<vslam_types::SLAMNode> *)> callback_creator,
    const ProblemParams &problem_params,
    vslam_types::UTObjectSLAMProblem<FeatureTrackType> &slam_problem,
    std::vector<vslam_types::RobotPose> &updated_robot_poses,
    std::vector<vslam_types::EllipsoidEstimate> &updated_ellipsoid_estimates) {
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // TODO configure options

  options.max_num_iterations = solver_optimization_params_.max_iterations;
  options.minimizer_type = solver_optimization_params_.minimizer_type;
  options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;

  std::vector<vslam_types::SLAMNode> slam_nodes;
  RobotPosesToSLAMNodes(updated_robot_poses, slam_nodes);

  std::vector<vslam_types::EllipsoidEstimateNode> ellispoid_nodes;
  EllipsoidEstimatesToNodes(updated_ellipsoid_estimates, ellispoid_nodes);

  // TODO add POM constraint adder

  if (!hold_robot_poses_constant) {
    vision_constraint_adder(problem_params, slam_problem, problem, &slam_nodes);
  }

  ellipsoid_constraint_creator(
      problem_params, slam_problem, problem, &slam_nodes, &ellispoid_nodes);

  // TODO these may cause errors if there were no bounding box observations at
  // the poses being set constant Set the first pose constant
  problem.SetParameterBlockConstant(slam_nodes[0].pose);

  if (hold_robot_poses_constant) {
    for (int i = 1; i < slam_nodes.size(); i++) {
      problem.SetParameterBlockConstant(slam_nodes[i].pose);
    }
  }

  // TODO consider making the callback creator return a vector of callbacks
  std::shared_ptr<ceres::IterationCallback> viz_callback =
      callback_creator(slam_problem, &slam_nodes);

  if (viz_callback != nullptr) {
    options.callbacks.emplace_back(viz_callback.get());
    options.update_state_every_iteration = true;
  }

  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  SLAMNodesToRobotPoses(slam_nodes, updated_robot_poses);
  EllipsoidNodesToEllipsoidEstimates(ellispoid_nodes,
                                     updated_ellipsoid_estimates);

  return (summary.termination_type == ceres::CONVERGENCE ||
          summary.termination_type == ceres::USER_SUCCESS);
}

template bool
ObjectSLAMSolver::SolveObjectSLAM<vslam_types::VisionFeatureTrack,
                                  StructurelessObjectSlamProblemParams>(
    const bool &use_pom,
    const bool &hold_robot_poses_constant,
    const std::function<void(
        const StructurelessObjectSlamProblemParams &,
        vslam_types::UTObjectSLAMProblem<vslam_types::VisionFeatureTrack> &,
        ceres::Problem &,
        std::vector<vslam_types::SLAMNode> *)> vision_constraint_adder,
    const std::function<void(
        const StructurelessObjectSlamProblemParams &,
        vslam_types::UTObjectSLAMProblem<vslam_types::VisionFeatureTrack> &,
        ceres::Problem &,
        std::vector<vslam_types::SLAMNode> *,
        std::vector<vslam_types::EllipsoidEstimateNode> *)>
        ellipsoid_constraint_creator,
    const std::function<std::shared_ptr<ceres::IterationCallback>(
        const vslam_types::UTObjectSLAMProblem<vslam_types::VisionFeatureTrack>
            &,
        std::vector<vslam_types::SLAMNode> *)> callback_creator,
    const StructurelessObjectSlamProblemParams &problem_params,
    vslam_types::UTObjectSLAMProblem<vslam_types::VisionFeatureTrack>
        &slam_problem,
    std::vector<vslam_types::RobotPose> &updated_robot_poses,
    std::vector<vslam_types::EllipsoidEstimate> &updated_ellipsoid_estimates);

template bool
ObjectSLAMSolver::SolveObjectSLAM<vslam_types::StructuredVisionFeatureTrack,
                                  StructuredObjectSlamProblemParams>(
    const bool &use_pom,
    const bool &hold_robot_poses_constant,
    const std::function<void(const StructuredObjectSlamProblemParams &,
                             vslam_types::UTObjectSLAMProblem<
                                 vslam_types::StructuredVisionFeatureTrack> &,
                             ceres::Problem &,
                             std::vector<vslam_types::SLAMNode> *)>
        vision_constraint_adder,
    const std::function<void(const StructuredObjectSlamProblemParams &,
                             vslam_types::UTObjectSLAMProblem<
                                 vslam_types::StructuredVisionFeatureTrack> &,
                             ceres::Problem &,
                             std::vector<vslam_types::SLAMNode> *,
                             std::vector<vslam_types::EllipsoidEstimateNode> *)>
        ellipsoid_constraint_creator,
    const std::function<std::shared_ptr<ceres::IterationCallback>(
        const vslam_types::UTObjectSLAMProblem<
            vslam_types::StructuredVisionFeatureTrack> &,
        std::vector<vslam_types::SLAMNode> *)> callback_creator,
    const StructuredObjectSlamProblemParams &problem_params,
    vslam_types::UTObjectSLAMProblem<vslam_types::StructuredVisionFeatureTrack>
        &slam_problem,
    std::vector<vslam_types::RobotPose> &updated_robot_poses,
    std::vector<vslam_types::EllipsoidEstimate> &updated_ellipsoid_estimates);

template void AddEllipsoidFactors(
    const ObjectSlamProblemParams &object_slam_params,
    vslam_types::UTObjectSLAMProblem<vslam_types::StructuredVisionFeatureTrack>
        &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *robot_pose_nodes_ptr,
    std::vector<vslam_types::EllipsoidEstimateNode> *ellipsoid_nodes_ptr);

template void AddEllipsoidFactors(
    const ObjectSlamProblemParams &object_slam_params,
    vslam_types::UTObjectSLAMProblem<vslam_types::VisionFeatureTrack>
        &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *robot_pose_nodes_ptr,
    std::vector<vslam_types::EllipsoidEstimateNode> *ellipsoid_nodes_ptr);
}  // namespace vslam_solver