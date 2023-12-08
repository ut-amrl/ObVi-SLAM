//
// Created by amanda on 1/6/23.
//

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <debugging/optimization_logger.h>
#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/bounding_box_frontend/pending_object_estimator.h>
#include <refactoring/factors/bounding_box_factor.h>
#include <refactoring/factors/shape_prior_factor.h>

namespace vslam_types_refactor {

template <typename ObjectAppearanceInfo,
          typename PendingObjInfo,
          typename VisualFeatureFactorType>
std::unordered_map<ObjectId, EllipsoidState<double>>
refineInitialEstimateForPendingObjects(
    const std::unordered_map<ObjectId, EllipsoidState<double>>
        &rough_initial_estimates,
    const std::unordered_map<
        ObjectId,
        UninitializedEllispoidInfo<ObjectAppearanceInfo, PendingObjInfo>>
        &uninitialized_obj_info,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>> &pose_graph,
    const PendingObjectEstimatorParams &estimator_params,
    const FrameId &frame_id,
    const CameraId &camera_id) {
#ifdef RUN_TIMERS
  CumulativeFunctionTimer::Invocation pending_est_refinement_invoc(
      CumulativeTimerFactory::getInstance()
          .getOrCreateFunctionTimer(
              kTimerNameRefineInitialEstimateForPendingObjects)
          .get());
#endif
  ceres::Problem problem;
  std::unordered_map<ObjectId, EllipsoidEstimateNode> raw_ests;
  std::unordered_map<FrameId, RobotPoseNode> robot_pose_nodes;
  std::unordered_map<FrameId, RawPose3d<double>> robot_pose_estimates;
  std::unordered_set<FrameId> added_poses;
  pose_graph->getRobotPoseEstimates(robot_pose_estimates);
  for (const auto &robot_pose : robot_pose_estimates) {
    RawPose3d<double> pose = robot_pose.second;
    robot_pose_nodes[robot_pose.first] = RobotPoseNode(pose);
  }

  for (const auto &est : rough_initial_estimates) {
    raw_ests[est.first] =
        EllipsoidEstimateNode(convertToRawEllipsoid(est.second));
  }

  std::unordered_map<std::string,
                     std::pair<ObjectDim<double>, Covariance<double, 3>>>
      mean_and_cov_by_semantic_class =
          pose_graph->getMeanAndCovBySemanticClass();
  for (const auto &obj_info : uninitialized_obj_info) {
    double *ellipsoid_ptr = raw_ests.at(obj_info.first).ellipsoid_->data();
    for (const auto &obs : obj_info.second.observation_factors_) {
      CameraId cam_id = obs.camera_id_;
      CameraExtrinsics<double> extrinsics;
      if (!pose_graph->getExtrinsicsForCamera(cam_id, extrinsics)) {
        LOG(ERROR) << "In using factor for pending obj " << obj_info.first
                   << " could not find extrinsics for camera " << cam_id
                   << "; not adding to pose graph";
        continue;
      }

      CameraIntrinsicsMat<double> intrinsics;
      if (!pose_graph->getIntrinsicsForCamera(cam_id, intrinsics)) {
        LOG(ERROR) << "In using factor for pending obj " << obj_info.first
                   << " could not find intrinsics for camera " << cam_id
                   << "; not adding to pose graph";
        continue;
      }

      double *robot_pose_block = robot_pose_nodes[obs.frame_id_].pose_->data();
      added_poses.insert(obs.frame_id_);
      problem.AddResidualBlock(
          BoundingBoxFactor::createBoundingBoxFactor(
              estimator_params.object_residual_params_
                  .invalid_ellipsoid_error_val_,
              obs.bounding_box_corners_,
              intrinsics,
              extrinsics,
              obs.bounding_box_corners_covariance_,
              std::nullopt,
              std::nullopt,
              std::nullopt),
          new ceres::HuberLoss(estimator_params.object_residual_params_
                                   .object_observation_huber_loss_param_),
          ellipsoid_ptr,
          robot_pose_block);
    }
    std::string semantic_class = obj_info.second.semantic_class_;
    problem.AddResidualBlock(
        ShapePriorFactor::createShapeDimPrior(
            mean_and_cov_by_semantic_class.at(semantic_class).first,
            mean_and_cov_by_semantic_class.at(semantic_class).second),
        new ceres::HuberLoss(estimator_params.object_residual_params_
                                 .shape_dim_prior_factor_huber_loss_param_),
        ellipsoid_ptr);
  }
  // Set poses constant
  for (const FrameId &added_pose : added_poses) {
    double *robot_pose_block = robot_pose_nodes[added_pose].pose_->data();
    problem.SetParameterBlockConstant(robot_pose_block);
  }

  ceres::Solver::Options options;
  options.max_num_iterations =
      estimator_params.solver_params_.max_num_iterations_;
  options.use_nonmonotonic_steps =
      estimator_params.solver_params_.allow_non_monotonic_steps_;
  options.function_tolerance =
      estimator_params.solver_params_.function_tolerance_;
  options.gradient_tolerance =
      estimator_params.solver_params_.gradient_tolerance_;
  options.parameter_tolerance =
      estimator_params.solver_params_.parameter_tolerance_;

  options.num_threads = 20;
  options.linear_solver_type = ceres::SPARSE_SCHUR;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport();

  if ((summary.termination_type == ceres::TerminationType::FAILURE) ||
      (summary.termination_type == ceres::TerminationType::USER_FAILURE)) {
    LOG(ERROR) << "Ceres optimization failed for pending  object estimation";
    exit(1);
  }
  std::string opt_identifier =
      std::to_string(frame_id) + "_" + std::to_string(camera_id);
  std::shared_ptr<IterationLogger> pending_obj_iter_loggger =
      IterationLoggerFactory::getInstance()
      .getOrCreateLoggerOfType(
          IterationLoggerFactory::kPendingEstimatorOptimizationType);
  if (pending_obj_iter_loggger != nullptr) {
    pending_obj_iter_loggger->logIterations(opt_identifier, summary);
  }
  LOG(INFO) << "Optimization complete";
  std::unordered_map<ObjectId, EllipsoidState<double>> updated_estimates;
  for (const auto &ellipsoid_node : raw_ests) {
    updated_estimates[ellipsoid_node.first] =
        convertToEllipsoidState(*(ellipsoid_node.second.ellipsoid_));
  }
  return updated_estimates;
}

template std::unordered_map<ObjectId, EllipsoidState<double>>
refineInitialEstimateForPendingObjects(
    const std::unordered_map<ObjectId, EllipsoidState<double>>
        &rough_initial_estimates,
    const std::unordered_map<
        ObjectId,
        UninitializedEllispoidInfo<FeatureBasedFrontEndObjAssociationInfo,
                                   FeatureBasedFrontEndPendingObjInfo>>
        &uninitialized_obj_info,
    const std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        ReprojectionErrorFactor>> &pose_graph,
    const PendingObjectEstimatorParams &estimator_params,
    const FrameId &frame_id,
    const CameraId &camera_id);
}  // namespace vslam_types_refactor