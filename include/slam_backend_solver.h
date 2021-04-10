#ifndef UT_VSLAM_SLAM_BACKEND_SOLVER_H
#define UT_VSLAM_SLAM_BACKEND_SOLVER_H

#include <ceres/ceres.h>

#include <slam_solver_optimizer_params.h>
#include <vslam_types.h>

namespace vslam_solver {

/**
 * SLAM Node that can be updated during optimization.
 */
struct UpdatableSLAMNode {

  /**
   * Node index.
   */
  uint64_t node_idx;

  /**
   * 6DOF parameters: tx, ty, tx, angle_x, angle_y, angle_z. Note that angle_*
   * are the coordinates in scaled angle-axis form.
   */
  double pose[6];

  /**
   * Default constructor.
   */
  UpdatableSLAMNode() = default;

  /**
   * Constructor that takes in index, translation, and rotation details for the
   * node.
   *
   * @param index           Index of the node.
   * @param pose_transl     Translation of the node.
   * @param pose_rot        Rotation of the node.
   */
  UpdatableSLAMNode(const uint64_t &index,
                    const Eigen::Vector3f &pose_transl,
                    const Eigen::AngleAxisf &pose_rot) {
    node_idx = index;
    pose[0] = pose_transl.x();
    pose[1] = pose_transl.y();
    pose[2] = pose_transl.z();
    pose[3] = pose_rot.axis().x() * pose_rot.angle();
    pose[4] = pose_rot.axis().y() * pose_rot.angle();
    pose[5] = pose_rot.axis().z() * pose_rot.angle();
  }
};

/**
 * Create an updatable SLAM node from a robot pose data structure.
 *
 * @param robot_pose Robot pose.
 *
 * @return Updatable SLAM node.
 */
UpdatableSLAMNode fromRobotPose(const vslam_types::RobotPose &robot_pose);

/**
 * Create a robot pose from an updatable SLAM node.
 *
 * @param slam_node Updatable SLAM node.
 *
 * @return Robot pose data structure.
 */
vslam_types::RobotPose fromUpdatableSLAMNode(const UpdatableSLAMNode &slam_node);

/**
 * Add nodes to the updatable nodes list that correspond to the information
 * in the robot poses. Will only add to the updatable_nodes vector.
 *
 * @param robot_poses[in]         Robot poses that should have updatable slam
 *                                nodes created for them.
 * @param updatable_nodes[out]    Nodes list that now contains entries for
 *                                each robot pose in the robot poses vector.
 */
void RobotPosesToUpdatableSLAMNodes(
    const std::vector<vslam_types::RobotPose> &robot_poses,
    std::vector<UpdatableSLAMNode> &updatable_nodes);

/**
 * Clear the updated poses list and create new entries from the SLAM nodes.
 * @param slam_nodes[in]      SLAM nodes that contain the robot poses after
 *                            optimization.
 * @param updated_poses[out]  Vector to update with optimized robot poses.
 */
void SLAMNodesToRobotPoses(
    const std::vector<UpdatableSLAMNode> &slam_nodes,
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
  SLAMSolver(const SLAMSolverOptimizerParams &solver_optimization_params) :
      solver_optimization_params_(solver_optimization_params) {};

  /**
   * Optimize the robot poses given the information about the SLAM problem.
   *
   * @param intrinsics[in]              Camera intrinsics.
   * @param extrinsics[in]              Camera extrinsics (camera pose relative
   *                                    to the robot).
   * @param slam_problem[in]            SLAM problem that provides constraints
   *                                    between poses.
   * @param updated_robot_poses[in/out] Robot poses to be updated. This contains
   *                                    initial estimates for the robot poses
   *                                    that will be updated after optimization.
   *
   * @return True if the SLAM solver converged, false if it didn't.
   */
  bool SolveSLAM(const vslam_types::CameraIntrinsics &intrinsics,
                 const vslam_types::CameraExtrinsics &extrinsics,
                 const vslam_types::UTSLAMProblem &slam_problem,
                 std::vector<vslam_types::RobotPose> &updated_robot_poses);
 private:

  /**
   * Solver optimization params.
   *
   * Requires param settings needed for optimization that are not dependent
   * on any particular instantiation of a SLAM problem.
   */
  SLAMSolverOptimizerParams solver_optimization_params_;

  /**
   * Add vision factors to the ceres optimization problem.
   *
   * @param slam_problem                    SLAM problem defining constraints
   *                                        between frames.
   * @param intrinsics                      Camera intrinsics.
   * @param extrinsics                      Camera extrinsics (camera pose
   *                                        relative to the robot).
   * @param ceres_problem[in/out]           Ceres problem that will have
   *                                        residual blocks added to it.
   * @param updated_solved_nodes[in/out]    Nodes in the SLAM problem that will
   *                                        be updated during optimization.
   *                                        The values will not change here,
   *                                        but they will be tied to the ceres
   *                                        problem.
   */
  void AddVisionFactors(
      const vslam_types::UTSLAMProblem &slam_problem,
      const vslam_types::CameraIntrinsics &intrinsics,
      const vslam_types::CameraExtrinsics &extrinsics,
      ceres::Problem *ceres_problem,
      std::vector<UpdatableSLAMNode> *updated_solved_nodes);
};
}

#endif  // UT_VSLAM_SLAM_BACKEND_SOLVER_H
