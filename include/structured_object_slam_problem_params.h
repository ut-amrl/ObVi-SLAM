#ifndef UT_VSLAM_STRUCTURED_OBJECT_SLAM_PROBLEM_PARAMS_H
#define UT_VSLAM_STRUCTURED_OBJECT_SLAM_PROBLEM_PARAMS_H

#include <object_slam_problem_params.h>
#include <structured_slam_problem_params.h>

namespace vslam_solver {

/**
 * Contains parameters related to solving the structured SLAM problem with
 * object estimates that are not specific to any particular instantiation of
 * the SLAM problem.
 *
 * This will contain default values that can be overwritten before actually
 * constructing the SLAM solver.
 */
struct StructuredObjectSlamProblemParams {
  /**
   * Parameters relating to the object constraints.
   */
  ObjectSlamProblemParams object_slam_params;

  /**
   * Parameters relating to the low-level feature observations and structured
   * visual SLAM problem.
   */
  StructuredSlamProblemParams structured_slam_params;
};
}  // namespace vslam_solver

#endif  // UT_VSLAM_STRUCTURED_OBJECT_SLAM_PROBLEM_PARAMS_H
