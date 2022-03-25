#ifndef UT_VSLAM_OBJECT_SLAM_BACKEND_SOLVER_H
#define UT_VSLAM_OBJECT_SLAM_BACKEND_SOLVER_H

#include <ceres/ceres.h>
#include <slam_solver_optimizer_params.h>
#include <structured_object_slam_problem_params.h>
#include <structureless_object_slam_problem_params.h>
#include <vslam_types.h>

namespace vslam_solver {

/**
 * Find ellipsoid estimates based only on bounding box observations and semantic
 * priors. Does not try to optimize for robot poses.
 *
 * @param ut_slam_problem[in]               SLAM problem defining observations.
 * @param solver_params[in]                 Params for solving the optimization
 *                                          problem.
 * @param callback_creator[in]              Callback creator.
 * @param problem_params[in]                SLAM problem parameters.
 * @param updated_ellipsoid_estimates[out]  Ellipsoid estimates after
 *                                          optimization.
 *
 * @return True if the SLAM solver converged, false if it didn't.
 */
bool findEllipsoidEstimates(
    vslam_types::UTObjectSLAMProblem<vslam_types::StructuredVisionFeatureTrack>
        &ut_slam_problem,
    const SLAMSolverOptimizerParams &solver_params,
    const std::function<std::shared_ptr<ceres::IterationCallback>(
        const vslam_types::UTObjectSLAMProblem<
            vslam_types::StructuredVisionFeatureTrack> &,
        std::vector<vslam_types::SLAMNode> *)> callback_creator,
    const StructuredObjectSlamProblemParams &problem_params,
    std::vector<vslam_types::EllipsoidEstimate> &updated_ellipsoid_estimates);

/**
 * Add ellipsoid factors (semantic shape prior + bounding box) to the ceres
 * problem.
 *
 * @tparam FeatureTrackType     Feature track type.
 *
 * @param object_slam_params[in]        Object SLAM specific parameters.
 * @param slam_problem[in]              SLAM problem. Defines the camera
 *                                      parameters and bounding box observations
 *                                      to incorporate.
 * @param ceres_problem[out]            Ceres problem to add residuals to.
 * @param robot_pose_nodes_ptr[in/out]  Robot pose nodes that will be included
 *                                      in residuals and optimized as part of
 *                                      the ceres optimization.
 * @param ellipsoid_nodes_ptr[in/out]   Ellipsoid estimate nodes that will be
 *                                      included in residuals and optimized as
 */
template <typename FeatureTrackType>
void AddEllipsoidFactors(
    const ObjectSlamProblemParams &object_slam_params,
    vslam_types::UTObjectSLAMProblem<FeatureTrackType> &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *robot_pose_nodes_ptr,
    std::vector<vslam_types::EllipsoidEstimateNode> *ellipsoid_nodes_ptr);

/**
 * Class for updating the robot poses given the SLAM problem constraints between
 * poses.
 */
class ObjectSLAMSolver {
 public:
  /**
   * SLAM Solver constructor.
   *
   * @param solver_optimization_params  Optimizer params (params not specific to
   *                                    any particular configuration of
   *                                    constraints or the type of problem).
   */
  ObjectSLAMSolver(const SLAMSolverOptimizerParams &solver_optimization_params)
      : solver_optimization_params_(solver_optimization_params){};

  /**
   * Optimize the robot poses given the information about the SLAM problem.
   *
   * @tparam FeatureTrackType           Type of the vision feature information.
   * @tparam ProblemParams              Type of the problem params.
   * @param vision_constraint_adder[in]         Function that adds vision
   *                                            constraints to the ceres
   *                                            optimization problem.
   * @param ellipsoid_constraint_creator[in]    Function that adds constraints
   *                                            for ellipsoids (non-POM) to the
   *                                            ceres optimization problem.
   * @param callback_creator[in]                Function that creates a callback
   *                                            for visualization.
   * @param problem_params[in]                  Parameters specific to the type
   *                                            of SLAM problem, but not to the
   *                                            specific instance of the SLAM
   *                                            problem.
   * @param slam_problem[in/out]                SLAM problem that provides
   *                                            constraints between poses and
   *                                            ellipsoids.
   * @param updated_robot_poses[in/out]         Robot poses to be updated. This
   *                                            contains initial estimates for
   *                                            the robot poses that will be
   *                                            updated after optimization.
   * @param updated_ellipsoid_estimates[in/out] Ellipsoid estimates to be
   *                                            updated. This contains initial
   *                                            estimates for the ellipsoids
   *                                            that will be updated after
   *                                            optimization.
   *
   * @return True if the SLAM solver converged, false if it didn't.
   */
  template <typename FeatureTrackType, typename ProblemParams>
  bool SolveObjectSLAM(
      const bool &use_pom,
      const bool &hold_robot_poses_constant,
      const std::function<
          void(const ProblemParams &,
               vslam_types::UTObjectSLAMProblem<FeatureTrackType> &,
               ceres::Problem &,
               std::vector<vslam_types::SLAMNode> *)> vision_constraint_adder,
      const std::function<
          void(const ProblemParams &,
               vslam_types::UTObjectSLAMProblem<FeatureTrackType> &,
               ceres::Problem &,
               std::vector<vslam_types::SLAMNode> *,
               std::vector<vslam_types::EllipsoidEstimateNode> *)>
          ellipsoid_constraint_creator,
      const std::function<std::shared_ptr<ceres::IterationCallback>(
          const vslam_types::UTObjectSLAMProblem<FeatureTrackType> &,
          std::vector<vslam_types::SLAMNode> *)> callback_creator,
      const ProblemParams &problem_params,
      vslam_types::UTObjectSLAMProblem<FeatureTrackType> &slam_problem,
      std::vector<vslam_types::RobotPose> &updated_robot_poses,
      std::vector<vslam_types::EllipsoidEstimate> &updated_ellipsoid_estimates);

 private:
  /**
   * Solver optimization params.
   *
   * Requires param settings needed for optimization that are not dependent
   * on any particular instantiation of a SLAM problem.
   */
  SLAMSolverOptimizerParams solver_optimization_params_;
};

}  // namespace vslam_solver

#endif  // UT_VSLAM_OBJECT_SLAM_BACKEND_SOLVER_H
