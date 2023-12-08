//
// Created by amanda on 1/6/23.
//

#ifndef UT_VSLAM_PENDING_OBJECT_ESTIMATOR_H
#define UT_VSLAM_PENDING_OBJECT_ESTIMATOR_H

#include <refactoring/bounding_box_frontend/bounding_box_front_end.h>
#include <refactoring/optimization/optimization_solver_params.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <unordered_map>

namespace vslam_types_refactor {

struct PendingObjectEstimatorParams {
  pose_graph_optimization::ObjectResidualParams object_residual_params_;
  pose_graph_optimization::OptimizationSolverParams solver_params_;

  bool operator==(const PendingObjectEstimatorParams &rhs) const {
    return (object_residual_params_ == rhs.object_residual_params_) &&
           (solver_params_ == rhs.solver_params_);
  }

  bool operator!=(const PendingObjectEstimatorParams &rhs) const {
    return !operator==(rhs);
  }
};

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
    const CameraId &camera_id);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_PENDING_OBJECT_ESTIMATOR_H
