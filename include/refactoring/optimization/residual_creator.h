//
// Created by amanda on 6/19/22.
//

#ifndef UT_VSLAM_RESIDUAL_CREATOR_H
#define UT_VSLAM_RESIDUAL_CREATOR_H

#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <refactoring/factors/bounding_box_factor.h>
#include <refactoring/factors/reprojection_cost_functor.h>
#include <refactoring/factors/shape_prior_factor.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/optimization_solver_params.h>

namespace vslam_types_refactor {

// TODO consider trying to make this generic to both types of obj pose graphs
template <typename CachedInfo>
bool createObjectObservationResidual(
    const vslam_types_refactor::FeatureFactorId &factor_id,
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const std::function<
        bool(const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId> &,
             const std::shared_ptr<
                 vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
             CachedInfo &)> &cached_info_creator,
    ceres::Problem *problem,
    ceres::ResidualBlockId &residual_id,
    CachedInfo &cached_info,
    const bool &debug = false) {
  // Get the factor
  ObjectObservationFactor factor;
  if (!pose_graph->getObjectObservationFactor(factor_id, factor)) {
    LOG(ERROR) << "Could not find object observation factor with id "
               << factor_id << "; not adding to pose graph";
    return false;
  }

  CameraExtrinsics<double> extrinsics;
  if (!pose_graph->getExtrinsicsForCamera(factor.camera_id_, extrinsics)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find extrinsics for camera " << factor.camera_id_
               << "; not adding to pose graph";
    return false;
  }

  CameraIntrinsicsMat<double> intrinsics;
  if (!pose_graph->getIntrinsicsForCamera(factor.camera_id_, intrinsics)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find intrinsics for camera " << factor.camera_id_
               << "; not adding to pose graph";
    return false;
  }

  double *ellipsoid_param_block;
  if (!pose_graph->getObjectParamPointers(factor.object_id_,
                                          &ellipsoid_param_block)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find ellipsoid parameter block for object "
               << factor.object_id_ << "; not adding to pose graph";
    return false;
  }

  double *robot_pose_block;
  if (!pose_graph->getPosePointers(factor.frame_id_, &robot_pose_block)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find robot pose parameter block for frame "
               << factor.frame_id_ << "; not adding to pose graph";
    return false;
  }

  if (!cached_info_creator(
          std::make_pair(kObjectObservationFactorTypeId, factor_id),
          pose_graph,
          cached_info)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could make cached info";
    return false;
  }

  residual_id = problem->AddResidualBlock(
      BoundingBoxFactor::createBoundingBoxFactor(
          residual_params.object_residual_params_.invalid_ellipsoid_error_val_,
          factor.bounding_box_corners_,
          intrinsics,
          extrinsics,
          factor.bounding_box_corners_covariance_,
          factor.object_id_,
          factor.frame_id_,
          factor.camera_id_,
          debug),
      new ceres::HuberLoss(residual_params.object_residual_params_
                               .object_observation_huber_loss_param_),
      ellipsoid_param_block,
      robot_pose_block);
  return true;
}

template <typename CachedInfo>
bool createObjectShapeDimPriorResidual(
    const vslam_types_refactor::FeatureFactorId &factor_id,
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const std::function<
        bool(const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId> &,
             const std::shared_ptr<
                 vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
             CachedInfo &)> &cached_info_creator,
    ceres::Problem *problem,
    ceres::ResidualBlockId &residual_id,
    CachedInfo &cached_info) {
  // Get the factor
  ShapeDimPriorFactor factor;
  if (!pose_graph->getShapeDimPriorFactor(factor_id, factor)) {
    LOG(ERROR) << "Could not find shape dim prior factor with id " << factor_id
               << "; not adding to pose graph";
    return false;
  }

  double *ellipsoid_param_block;
  if (!pose_graph->getObjectParamPointers(factor.object_id_,
                                          &ellipsoid_param_block)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find ellipsoid parameter block for object "
               << factor.object_id_ << "; not adding to pose graph";
    return false;
  }

  if (!cached_info_creator(
          std::make_pair(kShapeDimPriorFactorTypeId, factor_id),
          pose_graph,
          cached_info)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could make cached info";
    return false;
  }

  residual_id = problem->AddResidualBlock(
      ShapePriorFactor::createShapeDimPrior(factor.mean_shape_dim_,
                                            factor.shape_dim_cov_),
      new ceres::HuberLoss(residual_params.object_residual_params_
                               .shape_dim_prior_factor_huber_loss_param_),
      ellipsoid_param_block);
  return true;
}

template <typename CachedInfo>
bool createReprojectionErrorResidual(
    const vslam_types_refactor::FeatureFactorId &factor_id,
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const std::function<
        bool(const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId> &,
             const std::shared_ptr<
                 vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
             CachedInfo &)> &cached_info_creator,
    ceres::Problem *problem,
    ceres::ResidualBlockId &residual_id,
    CachedInfo &cached_info) {
  // Get the factor
  ReprojectionErrorFactor factor;
  if (!pose_graph->getVisualFactor(factor_id, factor)) {
    LOG(ERROR) << "Could not find visual feature factor with id " << factor_id
               << "; not adding to pose graph";
    return false;
  }

  CameraExtrinsics<double> extrinsics;
  if (!pose_graph->getExtrinsicsForCamera(factor.camera_id_, extrinsics)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find extrinsics for camera " << factor.camera_id_
               << "; not adding to pose graph";
    return false;
  }

  CameraIntrinsicsMat<double> intrinsics;
  if (!pose_graph->getIntrinsicsForCamera(factor.camera_id_, intrinsics)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find intrinsics for camera " << factor.camera_id_
               << "; not adding to pose graph";
    return false;
  }

  double *feature_position_block;
  if (!pose_graph->getFeaturePointers(factor.feature_id_,
                                      &feature_position_block)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find visual feature parameter block for feature "
               << factor.feature_id_ << "; not adding to pose graph";
    return false;
  }

  double *robot_pose_block;
  if (!pose_graph->getPosePointers(factor.frame_id_, &robot_pose_block)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find robot pose parameter block for frame "
               << factor.frame_id_ << "; not adding to pose graph";
    return false;
  }

  if (!cached_info_creator(
          std::make_pair(kReprojectionErrorFactorTypeId, factor_id),
          pose_graph,
          cached_info)) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could make cached info";
    return false;
  }

  residual_id = problem->AddResidualBlock(
      ReprojectionCostFunctor::create(intrinsics,
                                      extrinsics,
                                      factor.feature_pos_,
                                      factor.reprojection_error_std_dev_),
      new ceres::HuberLoss(residual_params.visual_residual_params_
                               .reprojection_error_huber_loss_param_),
      robot_pose_block,
      feature_position_block);

  return true;
}

template <typename CachedInfo>
bool createRelPoseResidual(
    const vslam_types_refactor::FeatureFactorId &factor_id,
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const std::function<
        bool(const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId> &,
             const std::shared_ptr<
                 vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
             CachedInfo &)> &cached_info_creator,
    ceres::Problem *problem,
    ceres::ResidualBlockId &residual_id,
    CachedInfo &cached_info) {
  vslam_types_refactor::RelPoseFactor factor;
  if (!pose_graph->getPoseFactor(factor_id, factor)) {
    LOG(ERROR) << "Could not find pose factor with id " << factor_id
               << "; not adding to pose graph";
    return false;
  }

  double *robot_pose1_block;
  double *robot_pose2_block;
  if ((!pose_graph->getPosePointers(factor.frame_id_1_, &robot_pose1_block)) ||
      (!pose_graph->getPosePointers(factor.frame_id_2_, &robot_pose2_block))) {
    LOG(ERROR) << "In using factor with id " << factor_id
               << " could not find robot pose parameter block for frame "
               << factor.frame_id_1_ << " or " << factor.frame_id_2_
               << "; not adding to pose graph";
    return false;
  }

  // std::optional<RawPose3d<double>> raw_robot_pose1, raw_robot_pose2;
  // raw_robot_pose1 = pose_graph->getRobotPose(factor.frame_id_1_);
  // raw_robot_pose2 = pose_graph->getRobotPose(factor.frame_id_2_);
  // if ((!raw_robot_pose1.has_value()) || (!raw_robot_pose2.has_value())) {
  //   LOG(ERROR) << "Could not find robot pose parameter block for frame "
  //              << factor.frame_id_1_ << " or " << factor.frame_id_2_
  //              << "; not adding to pose graph";
  //   return false;
  // }
  // vslam_types_refactor::Pose3D<double> measured_pose_deviation =
  //     vslam_types_refactor::getPose2RelativeToPose1(
  //         vslam_types_refactor::convertToPose3D(raw_robot_pose1.value()),
  //         vslam_types_refactor::convertToPose3D(raw_robot_pose2.value()));

  residual_id = problem->AddResidualBlock(
      RelativePoseFactor::createRelativePoseFactor(
          factor.measured_pose_deviation_, factor.pose_deviation_cov_),
      new ceres::HuberLoss(residual_params.relative_pose_factor_huber_loss_),
      robot_pose1_block,
      robot_pose2_block);

  return true;
}

template <typename CachedInfo>
bool createResidual(
    const std::pair<vslam_types_refactor::FactorType,
                    vslam_types_refactor::FeatureFactorId> &factor_info,
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
        &residual_params,
    const std::function<
        bool(const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId> &,
             const std::shared_ptr<
                 vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
             CachedInfo &)> &cached_info_creator,
    const std::function<bool(
        const std::pair<vslam_types_refactor::FactorType,
                        vslam_types_refactor::FeatureFactorId> &,
        const std::shared_ptr<
            vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
        const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
        const std::function<bool(
            const std::pair<vslam_types_refactor::FactorType,
                            vslam_types_refactor::FeatureFactorId> &,
            const std::shared_ptr<
                vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
            CachedInfo &)> &,
        ceres::Problem *,
        ceres::ResidualBlockId &,
        CachedInfo &)> &long_term_map_residual_creator,
    ceres::Problem *problem,
    ceres::ResidualBlockId &residual_id,
    CachedInfo &cached_info,
    const bool &debug = false) {
  if (factor_info.first == kPairwiseErrorFactorTypeId) {
    LOG(ERROR) << "Pairwise error observation type not supported with a "
                  "reprojection error factor graph";
    return false;
  } else if (factor_info.first == kObjectObservationFactorTypeId) {
    return createObjectObservationResidual(factor_info.second,
                                           pose_graph,
                                           residual_params,
                                           cached_info_creator,
                                           problem,
                                           residual_id,
                                           cached_info,
                                           debug);

  } else if (factor_info.first == kReprojectionErrorFactorTypeId) {
    return createReprojectionErrorResidual(factor_info.second,
                                           pose_graph,
                                           residual_params,
                                           cached_info_creator,
                                           problem,
                                           residual_id,
                                           cached_info);
  } else if (factor_info.first == kShapeDimPriorFactorTypeId) {
    return createObjectShapeDimPriorResidual(factor_info.second,
                                             pose_graph,
                                             residual_params,
                                             cached_info_creator,
                                             problem,
                                             residual_id,
                                             cached_info);
  } else if (factor_info.first == kLongTermMapFactorTypeId) {
    return long_term_map_residual_creator(factor_info,
                                          pose_graph,
                                          residual_params,
                                          cached_info_creator,
                                          problem,
                                          residual_id,
                                          cached_info);
  } else if (factor_info.first == kPairwiseRobotPoseFactorTypeId) {
    return createRelPoseResidual(factor_info.second,
                                 pose_graph,
                                 residual_params,
                                 cached_info_creator,
                                 problem,
                                 residual_id,
                                 cached_info);
  } else {
    LOG(ERROR) << "Unrecognized factor type " << factor_info.first
               << "; not adding residual";
    return false;
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_RESIDUAL_CREATOR_H
