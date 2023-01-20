//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
#define UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H

#include <ceres/problem.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/object_pose_graph_optimizer.h>

namespace vslam_types_refactor {

const double kFeatureOutlierPercentage = 0.1;

enum VisualizationTypeEnum {
  BEFORE_ANY_OPTIMIZATION,
  BEFORE_EACH_OPTIMIZATION,
  AFTER_EACH_OPTIMIZATION,
  AFTER_ALL_OPTIMIZATION
};

enum OptimTypeEnum {
  SLIDING_WINDOW,
  STATIC,
  TWOPHASE_SLIDING_WINDOW,
};

template <typename InputProblemData,
          typename VisualFeatureFactorType,
          typename OutputProblemData,
          typename CachedFactorInfo,
          typename PoseGraphType>
class OfflineProblemRunner {
 public:
  OfflineProblemRunner(
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
          &residual_params,
      const std::function<bool()> &continue_opt_checker,
      const std::function<FrameId(const FrameId &)> &window_provider_func,
      const std::function<
          bool(const std::pair<vslam_types_refactor::FactorType,
                               vslam_types_refactor::FeatureFactorId> &,
               const std::shared_ptr<PoseGraphType> &,
               const CachedFactorInfo &)> &refresh_residual_checker,
      const std::function<bool(
          const std::pair<vslam_types_refactor::FactorType,
                          vslam_types_refactor::FeatureFactorId> &,
          const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
          const std::shared_ptr<PoseGraphType> &,
          ceres::Problem *,
          ceres::ResidualBlockId &,
          CachedFactorInfo &)> &residual_creator,
      const std::function<void(const InputProblemData &,
                               std::shared_ptr<PoseGraphType> &)>
          &pose_graph_creator,
      const std::function<void(const InputProblemData &,
                               const std::shared_ptr<PoseGraphType> &,
                               const FrameId &)> &frame_data_adder,
      const std::function<
          void(const InputProblemData &,
               const std::shared_ptr<PoseGraphType> &,
               const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
               OutputProblemData &)> &output_data_extractor,
      const std::function<
          std::vector<std::shared_ptr<ceres::IterationCallback>>(
              const InputProblemData &,
              const std::shared_ptr<PoseGraphType> &,
              const FrameId &,
              const FrameId &)> &ceres_callback_creator,
      const std::function<void(const InputProblemData &,
                               const std::shared_ptr<PoseGraphType> &,
                               const FrameId &,
                               const FrameId &,
                               const VisualizationTypeEnum &)>
          &visualization_callback,
      const pose_graph_optimization::OptimizationSolverParams &solver_params)
      : residual_params_(residual_params),
        continue_opt_checker_(continue_opt_checker),
        window_provider_func_(window_provider_func),
        optimizer_(refresh_residual_checker, residual_creator),
        pose_graph_creator_(pose_graph_creator),
        frame_data_adder_(frame_data_adder),
        output_data_extractor_(output_data_extractor),
        ceres_callback_creator_(ceres_callback_creator),
        visualization_callback_(visualization_callback),
        solver_params_(solver_params) {}
  
  bool runOptimizationHelper(
    const FrameId start_frame_id,
    const FrameId end_frame_id,
    const InputProblemData &problem_data,
    ceres::Problem& problem,
    std::shared_ptr<PoseGraphType>& pose_graph,
    std::unordered_map<ceres::ResidualBlockId, double> 
        *block_ids_and_residuals_ptr) {
      std::vector<std::shared_ptr<ceres::IterationCallback>> ceres_callbacks =
        ceres_callback_creator_(
            problem_data, pose_graph, start_frame_id, end_frame_id);
      LOG(INFO) << "Solving optimization";
      bool opt_success = optimizer_.solveOptimization(
          &problem, solver_params_, ceres_callbacks, block_ids_and_residuals_ptr);
      if (!opt_success) {
        // TODO do we want to quit or just silently let this iteration fail?
        LOG(ERROR) << "Optimization failed at max frame id " << end_frame_id;
        return false;
      }
      return true;
  }

  bool runOptimization(
      const InputProblemData &problem_data,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams
          &optimization_factors_enabled_params,
      OutputProblemData &output_problem_data, 
      const OptimTypeEnum& optim_type = OptimTypeEnum::SLIDING_WINDOW) {
    
    ceres::Problem problem;
    std::shared_ptr<PoseGraphType> pose_graph;
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
    optimization_scope_params.fix_ltm_objects_ =
        optimization_factors_enabled_params.fix_ltm_objects_;
    optimization_scope_params.include_visual_factors_ =
        optimization_factors_enabled_params.include_visual_factors_;
    optimization_scope_params.include_object_factors_ =
        optimization_factors_enabled_params.include_object_factors_;
    optimization_scope_params.use_pom_ =
        optimization_factors_enabled_params.use_pom_;
    optimization_scope_params.poses_prior_to_window_to_keep_constant_ =
        optimization_factors_enabled_params
            .poses_prior_to_window_to_keep_constant_;
    
    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,  // Should this be max or 0
                            VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION);
    LOG(INFO) << "Ready to run optimization";
    bool optim_success;

    switch (optim_type) {
      case OptimTypeEnum::SLIDING_WINDOW: {
        for (FrameId next_frame_id = 1; next_frame_id <= max_frame_id;
            next_frame_id++) {
          if (!continue_opt_checker_) {
            LOG(WARNING)
                << "Halted optimization due to continue checker reporting false";
            return false;
          }
          FrameId start_frame_id = window_provider_func_(next_frame_id);
          FrameId end_frame_id = next_frame_id;
          optimization_scope_params.min_frame_id_ = start_frame_id;
          optimization_scope_params.max_frame_id_ = end_frame_id;
          frame_data_adder_(problem_data, pose_graph, end_frame_id);
          std::unordered_map<vslam_types_refactor::FactorType,
              std::unordered_map<vslam_types_refactor::FeatureFactorId,
                  ceres::ResidualBlockId>> current_residual_block_info = 
              optimizer_.buildPoseGraphOptimization(
                  optimization_scope_params, 
                  residual_params_, 
                  pose_graph, 
                  &problem);
          visualization_callback_(problem_data,
                              pose_graph,
                              start_frame_id,
                              end_frame_id,
                              VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
          optim_success = runOptimizationHelper(
            start_frame_id,
            end_frame_id,
            problem_data,
            problem,
            pose_graph,
            nullptr);
          if (!optim_success) { return optim_success; }
          visualization_callback_(problem_data,
                              pose_graph,
                              start_frame_id,
                              end_frame_id,
                              VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
        }
        break;
      }
      case OptimTypeEnum::STATIC: {
        FrameId start_frame_id = window_provider_func_(max_frame_id);
        FrameId end_frame_id = max_frame_id;
        optimization_scope_params.min_frame_id_ = start_frame_id;
        optimization_scope_params.max_frame_id_ = end_frame_id;
        for (FrameId frame_id = start_frame_id; frame_id <= end_frame_id; 
            ++frame_id) {
          frame_data_adder_(problem_data, pose_graph, frame_id);
        }
        std::unordered_map<vslam_types_refactor::FactorType,
              std::unordered_map<vslam_types_refactor::FeatureFactorId,
                  ceres::ResidualBlockId>> current_residual_block_info = 
              optimizer_.buildPoseGraphOptimization(
                  optimization_scope_params, 
                  residual_params_, 
                  pose_graph, 
                  &problem);
        visualization_callback_(problem_data,
                              pose_graph,
                              start_frame_id,
                              end_frame_id,
                              VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
        optim_success = runOptimizationHelper(
            start_frame_id,
            end_frame_id,
            problem_data,
            problem,
            pose_graph,
            nullptr);
        if (!optim_success) { return false; }
        visualization_callback_(problem_data,
                              pose_graph,
                              start_frame_id,
                              end_frame_id,
                              VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
        break;
      }
      case OptimTypeEnum::TWOPHASE_SLIDING_WINDOW: {
        bool optim_success;
        for (FrameId next_frame_id = 1; next_frame_id <= max_frame_id;
            next_frame_id++) {
          if (!continue_opt_checker_) {
            LOG(WARNING)
                << "Halted optimization due to continue checker reporting false";
            return false;
          }
          FrameId start_frame_id = window_provider_func_(next_frame_id);
          FrameId end_frame_id = next_frame_id;
          optimization_scope_params.min_frame_id_ = start_frame_id;
          optimization_scope_params.max_frame_id_ = end_frame_id;
          frame_data_adder_(problem_data, pose_graph, end_frame_id);

          // Phase I
          std::unordered_map<vslam_types_refactor::FactorType,
              std::unordered_map<vslam_types_refactor::FeatureFactorId,
                  ceres::ResidualBlockId>> current_residual_block_info = 
              optimizer_.buildPoseGraphOptimization(
                  optimization_scope_params, 
                  residual_params_, 
                  pose_graph, 
                  &problem);
          visualization_callback_(problem_data,
                              pose_graph,
                              start_frame_id,
                              end_frame_id,
                              VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);
          std::unordered_map<ceres::ResidualBlockId, double> 
              block_ids_and_residuals;
          optim_success = runOptimizationHelper(
              start_frame_id,
              end_frame_id,
              problem_data,
              problem,
              pose_graph,
              &block_ids_and_residuals);
          if (!optim_success) { return false; }

          // Phase II
          std::vector<std::pair<ceres::ResidualBlockId, double>> block_ids_and_residuals_pairs;
          for (const auto&block_id_and_residual : block_ids_and_residuals) {
            block_ids_and_residuals_pairs.emplace_back(
                block_id_and_residual.first, block_id_and_residual.second);
          }
          std::sort(block_ids_and_residuals_pairs.begin(), 
                    block_ids_and_residuals_pairs.end(),
                    [] (std::pair<ceres::ResidualBlockId, double> p1,
                        std::pair<ceres::ResidualBlockId, double> p2) {
                          return p1.second > p2.second;
                        });
          size_t n_outliers = 
              (size_t)(block_ids_and_residuals_pairs.size() * kFeatureOutlierPercentage);
          std::unordered_set<vslam_types_refactor::FeatureFactorId> feature_factor_ids_to_remove;
          // TODO fix the brutal fix backtrack!!
          LOG(INFO) << "current_residual_block_info size: " << current_residual_block_info.size();
          for (size_t i = 0; i < n_outliers; ++i) {
            // TODO emplace back feature_factor_ids_to_remove
            const ceres::ResidualBlockId& block_id = block_ids_and_residuals_pairs[i].first;
            for (const auto &factor_types_and_info : current_residual_block_info) {
              for (const auto& factor_id_and_block_id : factor_types_and_info.second) {
                if (factor_id_and_block_id.second == block_id) {
                  feature_factor_ids_to_remove.insert(factor_id_and_block_id.first);
                }
              }
            }
          }
          optimizer_.buildPoseGraphOptimization(
              optimization_scope_params, 
              residual_params_, 
              pose_graph, 
              &problem,
              feature_factor_ids_to_remove);
          // TODO extract feature_factor_ids_to_remove from block_ids_and_residuals
          optim_success = runOptimizationHelper(
              start_frame_id,
              end_frame_id,
              problem_data,
              problem,
              pose_graph,
              nullptr);
          if (!optim_success) { return false; }
          visualization_callback_(problem_data,
                              pose_graph,
                              start_frame_id,
                              end_frame_id,
                              VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
        }
        break;
      }
      default:
        break;
    }

    visualization_callback_(problem_data,
                            pose_graph,
                            0,
                            max_frame_id,
                            VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION);
    output_data_extractor_(problem_data,
                           pose_graph,
                           optimization_factors_enabled_params,
                           output_problem_data);
    return true;
  }

 private:
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params_;

  std::function<bool()> continue_opt_checker_;
  std::function<FrameId(const FrameId &)> window_provider_func_;
  pose_graph_optimizer::ObjectPoseGraphOptimizer<VisualFeatureFactorType,
                                                 CachedFactorInfo,
                                                 PoseGraphType>
      optimizer_;
  std::function<void(const InputProblemData &,
                     std::shared_ptr<PoseGraphType> &)>
      pose_graph_creator_;

  std::function<void(const InputProblemData &,
                     const std::shared_ptr<PoseGraphType> &,
                     const FrameId &)>
      frame_data_adder_;
  std::function<void(
      const InputProblemData &,
      const std::shared_ptr<PoseGraphType> &,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
      OutputProblemData &)>
      output_data_extractor_;

  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const InputProblemData &,
      const std::shared_ptr<PoseGraphType> &,
      const FrameId &,
      const FrameId &)>
      ceres_callback_creator_;
  std::function<void(const InputProblemData &,
                     const std::shared_ptr<PoseGraphType> &,
                     const FrameId &,
                     const FrameId &,
                     const VisualizationTypeEnum &)>
      visualization_callback_;
  pose_graph_optimization::OptimizationSolverParams solver_params_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OFFLINE_PROBLEM_RUNNER_H
