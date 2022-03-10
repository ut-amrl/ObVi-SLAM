#ifndef UT_VSLAM_SLAM_SOLVER_OPTIMIZER_PARAMS_H
#define UT_VSLAM_SLAM_SOLVER_OPTIMIZER_PARAMS_H

namespace vslam_solver {

/**
 * Contains parameters related to solving general SLAM problems that are not
 * specific to any particular instantiation of the SLAM problem.
 *
 * This will contain default values that can be overwritten before actually
 * constructing the SLAM solver.
 */
struct SLAMSolverOptimizerParams {
  /**
   * Maximum number of iterations to run the optimizer for.
   */
  int max_iterations = 10000; // TODO

  /**
   * Minimizer type for ceres optimization.
   */
  ceres::MinimizerType minimizer_type = ceres::MinimizerType::TRUST_REGION;
  size_t max_num_consecutive_invalid_steps = 20;
};

}  // namespace vslam_solver

#endif  // UT_VSLAM_SLAM_SOLVER_OPTIMIZER_PARAMS_H
