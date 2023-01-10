#ifndef UT_VSLAM_SLAM_TYPE_CONVERSION_UTIL_H
#define UT_VSLAM_SLAM_TYPE_CONVERSION_UTIL_H

#include <glog/logging.h>
#include <vslam_math_util.h>
#include <vslam_types.h>

#include <eigen3/Eigen/Dense>
#include <vector>

namespace vslam_util {

/**
 * Create an ellipsoid node from a ellipsoid estimate data structure.
 *
 * @param ellipsoid_estimate Ellispoid estimate.
 *
 * @return Ellipsoid estimate node.
 */
vslam_types::EllipsoidEstimateNode FromEllipsoidEstimate(
    const vslam_types::EllipsoidEstimate& ellipsoid_estimate);

/**
 * Create an ellipsoid estimate from an ellipsoid estimate node.
 *
 * @param ellipsoid_node Ellipsoid node.
 *
 * @return Ellipsoid estimate.
 */
vslam_types::EllipsoidEstimate FromEllipsoidNode(
    const vslam_types::EllipsoidEstimateNode& ellipsoid_node);

/**
 * Add nodes to the nodes list that correspond to the information in the
 * ellipsoid estimates.
 *
 * @param ellispoid_estimates[in]   Ellipsoid estimates that should have slam
 *                                  nodes created for them.
 * @param nodes[out]                Nodes list that now contains entries for
 *                                  each ellipsoid estimate.
 */
void EllipsoidEstimatesToNodes(
    const std::vector<vslam_types::EllipsoidEstimate>& ellipsoid_estimates,
    std::vector<vslam_types::EllipsoidEstimateNode>& nodes);

/**
 * Clear the updated estimates list and create new entries from the nodes.
 *
 * @param ellipsoid_nodes[in]       SLAM nodes that contain the ellipsoid
 *                                  estimates poses after optimization.
 * @param updated_estimates[out]    Vector to update with optimized ellipsoid
 *                                  estimates.
 */
void EllipsoidNodesToEllipsoidEstimates(
    const std::vector<vslam_types::EllipsoidEstimateNode>& ellipsoid_nodes,
    std::vector<vslam_types::EllipsoidEstimate>& updated_estimates);

/**
 * Create an SLAM node from a robot pose data structure.
 *
 * @param robot_pose Robot pose.
 *
 * @return SLAM node.
 */
vslam_types::SLAMNode FromRobotPose(const vslam_types::RobotPose& robot_pose);

/**
 * Create a robot pose from an SLAM node.
 *
 * @param slam_node SLAM node.
 *
 * @return Robot pose data structure.
 */
vslam_types::RobotPose FromSLAMNode(const vslam_types::SLAMNode& slam_node);

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
    const std::vector<vslam_types::RobotPose>& robot_poses,
    std::vector<vslam_types::SLAMNode>& nodes);

/**
 * Clear the updated poses list and create new entries from the SLAM nodes.
 *
 * @param slam_nodes[in]      SLAM nodes that contain the robot poses after
 *                            optimization.
 * @param updated_poses[out]  Vector to update with optimized robot poses.
 */
void SLAMNodesToRobotPoses(const std::vector<vslam_types::SLAMNode>& slam_nodes,
                           std::vector<vslam_types::RobotPose>& updated_poses);

template <typename T>
Eigen::Matrix<T, 4, 1> cornerLocationsPairToVector(
    const std::pair<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>>&
        corner_pair) {
  return Eigen::Matrix<T, 4, 1>(corner_pair.first.x(),
                                corner_pair.second.x(),
                                corner_pair.first.y(),
                                corner_pair.second.y());
}

template <typename T>
std::pair<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>>
cornerLocationsVectorToPair(const Eigen::Matrix<T, 4, 1>& corner_vec) {
  // TODO is this the right order?
  return std::make_pair(Eigen::Matrix<T, 2, 1>(corner_vec(0), corner_vec(2)),
                        Eigen::Matrix<T, 2, 1>(corner_vec(1), corner_vec(3)));
}

}  // namespace vslam_util

#endif  // UT_VSLAM_SLAM_TYPE_CONVERSION_UTIL_H