//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
#define UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H

#include <ceres/problem.h>
#include <refactoring/object_pose_graph.h>
#include <refactoring/object_pose_graph_optimizer.h>

namespace vslam_types_refactor {

enum VisualizationTypeEnum {
  BEFORE_ANY_OPTIMIZATION,
  BEFORE_EACH_OPTIMIZATION,
  AFTER_EACH_OPTIMIZATION,
  AFTER_ALL_OPTIMIZATION
};

struct OptimizationFactorsEnabledParams {
  bool include_object_factors_;
  bool include_visual_factors_;
  bool fix_poses_;
  bool fix_objects_;
  bool fix_visual_features_;
  bool use_pom_;
};

template <typename InputProblemData,
          typename VisualFeatureFactorType,
          typename OutputProblemData,
          typename CachedFactorInfo>
class OfflineProblemRunner {
 public:
  OfflineProblemRunner(
      const std::function<bool()> &continue_opt_checker,
      const std::function<FrameId(const FrameId &)> &window_provider_func,
      const std::function<
          bool(const std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId>,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &pose_graph,
               const CachedFactorInfo &factor_info)> &refresh_residual_checker,
      const std::function<
          bool(const std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId>,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               ceres::Problem *,
               ceres::ResidualBlockId &,
               CachedFactorInfo &factor_info)> &residual_creator,
      const std::function<
          void(const InputProblemData &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &)> &pose_graph_creator,
      const std::function<
          void(const InputProblemData &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               const FrameId &)> &frame_data_adder,
      const std::function<
          void(const InputProblemData &,
               const std::shared_ptr<
                   const vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               OutputProblemData &)> &output_data_extractor,
      const std::function<
          std::vector<std::shared_ptr<ceres::IterationCallback>>(
              const InputProblemData &,
              const std::shared_ptr<
                  vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                      VisualFeatureFactorType>> &,
              const FrameId &,
              const FrameId &)> &ceres_callback_creator,
      const std::function<
          void(const InputProblemData &,
               const std::shared_ptr<
                   vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                       VisualFeatureFactorType>> &,
               const FrameId &,
               const FrameId &,
               const VisualizationTypeEnum &)> &visualization_callback,
      const pose_graph_optimization::OptimizationSolverParams &solver_params)
      : continue_opt_checker_(continue_opt_checker),
        window_provider_func_(window_provider_func),
        optimizer_(refresh_residual_checker, residual_creator),
        pose_graph_creator_(pose_graph_creator),
        frame_data_adder_(frame_data_adder),
        output_data_extractor_(output_data_extractor),
        ceres_callback_creator_(ceres_callback_creator),
        visualization_callback_(visualization_callback),
        solver_params_(solver_params) {}

  bool runOptimization(const InputProblemData &problem_data,
                       const OptimizationFactorsEnabledParams
                           &optimization_factors_enabled_params,
                       OutputProblemData &output_problem_data) {
    std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        VisualFeatureFactorType>>
        pose_graph;

    ceres::Problem problem;
    pose_graph_creator_(problem_data, pose_graph);
    frame_data_adder_(problem_data, pose_graph, 0);
    FrameId max_frame_id = problem_data.getMaxFrameId();
    pose_graph_optimizer::OptimizationScopeParams optimization_scope_params;
    optimization_scope_params.fix_poses_ =
        optimization_factors_enabled_params.fix_poses_;
    optimization_scope_params.fix_objects_ =
        optimization_factors_enabled_params.fix_objects_;
    optimization_scope_params.fix_visual_features_ =
        optimization_factors_enabled_params.fix_visual_features_;
    optimization_scope_params.include_visual_factors_ =
        optimization_factors_enabled_params.include_visual_factors_;
    optimization_scope_params.include_object_factors_ =
        optimization_factors_enabled_params.include_object_factors_;
    optimization_scope_params.use_pom_ =
        optimization_factors_enabled_params.use_pom_;

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,  // Should this be max or 0
                            VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION);
    for (FrameId next_frame_id = 1; next_frame_id <= max_frame_id;
         next_frame_id++) {
      if (!continue_opt_checker_) {
        LOG(WARNING)
            << "Halted optimization due to continue checker reporting false";
        return false;
      }
      // This function is also responsible for adjusting the initial estimate
      // for the pose at next_frame_id given the optimized pose at next_frame_id
      // - 1
      frame_data_adder_(problem_data, pose_graph, next_frame_id);
      FrameId start_opt_with_frame = window_provider_func_(next_frame_id);
      optimization_scope_params.min_frame_id_ = start_opt_with_frame;
      optimization_scope_params.max_frame_id_ = next_frame_id;

      optimizer_.buildPoseGraphOptimization(
          optimization_scope_params, pose_graph, &problem);

      visualization_callback_(problem_data,
                              pose_graph,
                              start_opt_with_frame,
                              next_frame_id,
                              VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
      std::vector<ceres::IterationCallback *> ceres_callbacks =
          ceres_callback_creator_(
              problem_data, pose_graph, start_opt_with_frame, next_frame_id);
      bool opt_success = optimizer_.solveOptimization(
          problem, solver_params_, ceres_callbacks);

      visualization_callback_(problem_data,
                              start_opt_with_frame,
                              next_frame_id,
                              VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
      if (!opt_success) {
        // TODO do we want to quit or just silently let this iteration fail?
        LOG(ERROR) << "Optimization failed at max frame id " << next_frame_id;
        return false;
      }
    }

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,
                            VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION);
    output_data_extractor_(problem_data, pose_graph, output_problem_data);
    return true;
  }

 private:
  pose_graph_optimizer::ObjectPoseGraphOptimizer<VisualFeatureFactorType,
                                                 CachedFactorInfo>
      optimizer_;

  std::function<bool()> continue_opt_checker_;
  std::function<FrameId(const FrameId &)> window_provider_func_;

  std::function<void(const InputProblemData &,
                     const std::shared_ptr<
                         vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                             VisualFeatureFactorType>> &)>
      pose_graph_creator_;

  std::function<void(const InputProblemData &,
                     const std::shared_ptr<
                         vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                             VisualFeatureFactorType>> &,
                     const FrameId &)>
      frame_data_adder_;
  std::function<void(
      const InputProblemData &,
      const std::shared_ptr<
          const vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &,
      OutputProblemData &)>
      output_data_extractor_;

  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const InputProblemData &,
      const std::shared_ptr<
          vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
              VisualFeatureFactorType>> &,
      const FrameId &,
      const FrameId &)>
      ceres_callback_creator_;
  std::function<void(const InputProblemData &,
                     const std::shared_ptr<
                         vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                             VisualFeatureFactorType>> &,
                     const FrameId &,
                     const FrameId &,
                     const VisualizationTypeEnum &)>
      visualization_callback_;
  pose_graph_optimization::OptimizationSolverParams solver_params_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
