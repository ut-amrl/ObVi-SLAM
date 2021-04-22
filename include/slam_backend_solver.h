#ifndef UT_VSLAM_SLAM_BACKEND_SOLVER_H
#define UT_VSLAM_SLAM_BACKEND_SOLVER_H

#include <ceres/ceres.h>
#include <slam_solver_optimizer_params.h>
#include <vslam_types.h>

namespace vslam_solver {

/**
 * Create an SLAM node from a robot pose data structure.
 *
 * @param robot_pose Robot pose.
 *
 * @return SLAM node.
 */
vslam_types::SLAMNode FromRobotPose(const vslam_types::RobotPose &robot_pose);

/**
 * Create a robot pose from an SLAM node.
 *
 * @param slam_node SLAM node.
 *
 * @return Robot pose data structure.
 */
vslam_types::RobotPose FromSLAMNode(const vslam_types::SLAMNode &slam_node);

/**
 * Add nodes to the nodes list that correspond to the information in the robot
 * poses. Will only add to the nodes vector.
 *
 * @param robot_poses[in]   Robot poses that should have slam nodes
 *                          created for them.
 * @param nodes[out]        Nodes list that now contains entries for each
 *                          robot pose in the robot poses vector.
 */
void RobotPosesToSLAMNodes(
    const std::vector<vslam_types::RobotPose> &robot_poses,
    std::vector<vslam_types::SLAMNode> &nodes);

/**
 * Clear the updated poses list and create new entries from the SLAM nodes.
 *
 * @param slam_nodes[in]      SLAM nodes that contain the robot poses after
 *                            optimization.
 * @param updated_poses[out]  Vector to update with optimized robot poses.
 */
void SLAMNodesToRobotPoses(const std::vector<vslam_types::SLAMNode> &slam_nodes,
                           std::vector<vslam_types::RobotPose> &updated_poses);

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
   *                                    constraints).
   */
  SLAMSolver(const SLAMSolverOptimizerParams &solver_optimization_params)
      : solver_optimization_params_(solver_optimization_params){};

  /**
   * Optimize the robot poses given the information about the SLAM problem.
   *
   * @tparam FeatureTrackType           Type of the vision feature information.
   * @param intrinsics[in]              Camera intrinsics.
   * @param extrinsics[in]              Camera extrinsics (camera pose relative
   *                                    to the robot).
   * @param slam_problem[in]            SLAM problem that provides constraints
   *                                    between poses.
   * @param vision_constraint_adder[in] Function that adds vision constraints to
   *                                    the ceres optimization problem.
   * @param callback_creator[in]        Function that creates a callback for
   *                                    visualization.
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
  template <typename FeatureTrackType>
  bool SolveSLAM(
      const vslam_types::CameraIntrinsics &intrinsics,
      const vslam_types::CameraExtrinsics &extrinsics,
      const vslam_types::UTSLAMProblem<FeatureTrackType> &slam_problem,
      const std::function<
          void(const vslam_types::UTSLAMProblem<FeatureTrackType> &,
               const vslam_types::CameraIntrinsics &,
               const vslam_types::CameraExtrinsics &,
               const SLAMSolverOptimizerParams &,
               ceres::Problem &,
               std::vector<vslam_types::SLAMNode> *)> vision_constraint_adder,
      const std::function<std::shared_ptr<ceres::IterationCallback>(
          const vslam_types::CameraIntrinsics &,
          const vslam_types::CameraExtrinsics &,
          const vslam_types::UTSLAMProblem<FeatureTrackType> &,
          std::vector<vslam_types::SLAMNode> *)> callback_creator,
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
 * @param slam_problem                  SLAM problem defining constraints
 *                                      between frames.
 * @param intrinsics                    Camera intrinsics.
 * @param extrinsics                    Camera extrinsics (camera pose relative
 *                                      to the robot).
 * @param solver_optimization_params    Solver optimization parameters
 * @param ceres_problem[in/out]         Ceres problem that will have residual
 *                                      blocks added to it.
 * @param updated_solved_nodes[in/out]  Nodes in the SLAM problem that will be
 *                                      updated during optimization. The values
 *                                      will not change here, but they will be
 *                                      tied to the ceres problem.
 */
void AddStructurelessVisionFactors(
    const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>
        &slam_problem,
    const vslam_types::CameraIntrinsics &intrinsics,
    const vslam_types::CameraExtrinsics &extrinsics,
    const SLAMSolverOptimizerParams &solver_optimization_params,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *updated_solved_nodes);
}  // namespace vslam_solver

#endif  // UT_VSLAM_SLAM_BACKEND_SOLVER_H
