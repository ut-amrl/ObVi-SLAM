//
// Created by amanda on 6/14/22.
//

#ifndef UT_VSLAM_OPTIMIZATION_SOLVER_PARAMS_H
#define UT_VSLAM_OPTIMIZATION_SOLVER_PARAMS_H

namespace pose_graph_optimization {
struct OptimizationSolverParams {
  int max_num_iterations_ = 100;
  // TODO
};

// TOOD revise all residual param defaults
struct ObjectResidualParams {
  double object_observation_huber_loss_param_ = 1;
  double shape_dim_prior_factor_huber_loss_param_ = 1;
};

struct PairwiseLongTermMapResidualParams {
  double pair_huber_loss_param_ = 1;
};

struct VisualFeaturePoseGraphResidualParams {
  double reprojection_error_huber_loss_param_ = 1;
};

struct ObjectVisualPoseGraphResidualParams {
  ObjectResidualParams object_residual_params_;
  VisualFeaturePoseGraphResidualParams visual_residual_params_;
  // TODO maybe make this template so it's flexible to different types of
  // long-term maps
  PairwiseLongTermMapResidualParams long_term_map_params_;
};
}  // namespace pose_graph_optimization

#endif  // UT_VSLAM_OPTIMIZATION_SOLVER_PARAMS_H
