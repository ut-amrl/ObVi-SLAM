#ifndef UT_VSLAM_STRUCTURED_SLAM_PROBLEM_PARAMS_H
#define UT_VSLAM_STRUCTURED_SLAM_PROBLEM_PARAMS_H

namespace vslam_solver {

/**
 * Contains parameters related to solving the structured SLAM problem that are
 * not specific to any particular instantiation of the SLAM problem.
 *
 * This will contain default values that can be overwritten before actually
 * constructing the SLAM solver.
 */
struct StructuredSlamProblemParams {
  /**
   * Standard deviation for the reprojection error.
   */
  // TODO set this to a useful value
  double reprojection_error_std_dev = 1.0;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_STRUCTURED_SLAM_PROBLEM_PARAMS_H
