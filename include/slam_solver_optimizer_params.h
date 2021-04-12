#ifndef UT_VSLAM_SLAM_SOLVER_OPTIMIZER_PARAMS_H
#define UT_VSLAM_SLAM_SOLVER_OPTIMIZER_PARAMS_H

namespace vslam_solver {

/**
 * Contains parameters related to solving the SLAM problem that are not specific
 * to any particular instantiation of the SLAM problem.
 *
 * This will contain default values that can be overwritten before actually
 * constructing the SLAM solver.
 */
struct SLAMSolverOptimizerParams {
  /**
   * Standard deviation for the epipolar error of a feature that is seen in
   * two different images.
   */
  // TODO set this to a useful value
  double epipolar_error_std_dev = 1.0;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_SLAM_SOLVER_OPTIMIZER_PARAMS_H
