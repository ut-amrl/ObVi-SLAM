#ifndef UT_VSLAM_SLAM_BACKEND_SOLVER_H
#define UT_VSLAM_SLAM_BACKEND_SOLVER_H

#include <ceres/ceres.h>
#include <slam_solver_optimizer_params.h>
#include <structured_slam_problem_params.h>
#include <structureless_slam_problem_params.h>
#include <vslam_types.h>

namespace vslam_solver {

/**
 * Class for updating the robot poses given the SLAM problem constraints between
 * poses.
 */
class SLAMSolver {
 public:
  /**
   * SLAM Solver constructor.
   *
   * @param solver_optimization_params  Optimizer params (params not specific to
   *                                    any particular configuration of
   *                                    constraints or the type of problem).
   */
  SLAMSolver(const SLAMSolverOptimizerParams &solver_optimization_params)
      : solver_optimization_params_(solver_optimization_params){};

  /**
   * Optimize the robot poses given the information about the SLAM problem.
   *
   * @tparam FeatureTrackType           Type of the vision feature information.
   * @tparam ProblemParams              Type of the problem params.
   * @param vision_constraint_adder[in] Function that adds vision constraints to
   *                                    the ceres optimization problem.
   * @param callback_creator[in]        Function that creates a callback for
   *                                    visualization.
   * @param problem_params[in]          Parameters specific to the type of SLAM
   *                                    problem, but not to the specific
   *                                    instance of the SLAM problem.
   * @param slam_problem[in/out]        SLAM problem that provides constraints
   *                                    between poses.
   * @param updated_robot_poses[in/out] Robot poses to be updated. This contains
   *                                    initial estimates for the robot poses
   *                                    that will be updated after optimization.
   *
   * @return True if the SLAM solver converged, false if it didn't.
   */
  // TODO I'm wondering if we should have two different params data structures,
  //  one for general params that apply to all variants of the solver
  //  (basically Ceres params) and another that is dependent on the type of the
  //  constraints that we're adding. For now I'm going to keep it as is, but
  //  wanted to note it here
  template <typename FeatureTrackType, typename ProblemParams>
  bool SolveSLAM(
      const std::function<void(const ProblemParams &,
                               vslam_types::UTSLAMProblem<FeatureTrackType> &,
                               ceres::Problem &,
                               std::vector<vslam_types::SLAMNode> *)>
          vision_constraint_adder,
      const std::function<std::shared_ptr<ceres::IterationCallback>(
          const vslam_types::UTSLAMProblem<FeatureTrackType> &,
          std::vector<vslam_types::SLAMNode> *)> callback_creator,
      const ProblemParams &problem_params,
      vslam_types::UTSLAMProblem<FeatureTrackType> &slam_problem,
      std::vector<vslam_types::RobotPose> &updated_robot_poses);

 private:
  /**
   * Solver optimization params.
   *
   * Requires param settings needed for optimization that are not dependent
   * on any particular instantiation of a SLAM problem.
   */
  SLAMSolverOptimizerParams solver_optimization_params_;
};

/**
 * Add vision factors to the ceres optimization problem.
 *
 * @param solver_optimization_params[in]Solver optimization parameters
 * @param slam_problem[in/out]          SLAM problem defining constraints
 *                                      between frames.
 * @param ceres_problem[in/out]         Ceres problem that will have residual
 *                                      blocks added to it.
 * @param updated_solved_nodes[in/out]  Nodes in the SLAM problem that will be
 *                                      updated during optimization. The values
 *                                      will not change here, but they will be
 *                                      tied to the ceres problem.
 */
void AddStructurelessVisionFactors(
    const StructurelessSlamProblemParams &solver_optimization_params,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *updated_solved_nodes);

/**
 * Add structured vision factors to the ceres optimization problem.
 *
 * @param solver_optimization_params[in]Solver optimization parameters
 * @param slam_problem[in/out]          SLAM problem defining constraints
 *                                      between frames. Poses of feature points
 *                                      will be attached to ceres optimization.
 * @param ceres_problem[in/out]         Ceres problem that will have residual
 *                                      blocks added to it.
 * @param updated_solved_nodes[in/out]  Nodes in the SLAM problem that will be
 *                                      updated during optimization. The values
 *                                      will not change here, but they will be
 *                                      tied to the ceres problem.
 */
void AddStructuredVisionFactors(
    const StructuredSlamProblemParams &solver_optimization_params,
    vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack>
        &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *updated_solved_nodes);
}  // namespace vslam_solver

#endif  // UT_VSLAM_SLAM_BACKEND_SOLVER_H
