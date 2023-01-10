#include <vslam_type_conversion_util.h>

namespace vslam_util {

vslam_types::EllipsoidEstimateNode FromEllipsoidEstimate(
    const vslam_types::EllipsoidEstimate& ellipsoid_estimate) {
  return vslam_types::EllipsoidEstimateNode(ellipsoid_estimate.loc,
                                            ellipsoid_estimate.orientation,
                                            ellipsoid_estimate.ellipsoid_dim,
                                            ellipsoid_estimate.semantic_class,
                                            ellipsoid_estimate.ellipsoid_idx);
}

vslam_types::EllipsoidEstimate FromEllipsoidNode(
    const vslam_types::EllipsoidEstimateNode& ellipsoid_node) {
  Eigen::Vector3f rotation_axis(
      ellipsoid_node.pose[3], ellipsoid_node.pose[4], ellipsoid_node.pose[5]);

  Eigen::AngleAxisf rotation_aa = VectorToAxisAngle(rotation_axis);

  Eigen::Vector3f transl(
      ellipsoid_node.pose[0], ellipsoid_node.pose[1], ellipsoid_node.pose[2]);
  Eigen::Vector3f dim(
      ellipsoid_node.pose[6], ellipsoid_node.pose[7], ellipsoid_node.pose[8]);
  return vslam_types::EllipsoidEstimate(transl,
                                        rotation_aa,
                                        dim,
                                        ellipsoid_node.semantic_class,
                                        ellipsoid_node.ellipsoid_idx);
}

void EllipsoidEstimatesToNodes(
    const std::vector<vslam_types::EllipsoidEstimate>& ellipsoid_estimates,
    std::vector<vslam_types::EllipsoidEstimateNode>& nodes) {
  for (const vslam_types::EllipsoidEstimate& estimate : ellipsoid_estimates) {
    nodes.emplace_back(FromEllipsoidEstimate(estimate));
  }
}

void EllipsoidNodesToEllipsoidEstimates(
    const std::vector<vslam_types::EllipsoidEstimateNode>& ellipsoid_nodes,
    std::vector<vslam_types::EllipsoidEstimate>& updated_estimates) {
  updated_estimates.clear();
  for (const vslam_types::EllipsoidEstimateNode& node : ellipsoid_nodes) {
    updated_estimates.emplace_back(FromEllipsoidNode(node));
  }
}

vslam_types::SLAMNode FromRobotPose(const vslam_types::RobotPose& robot_pose) {
  return vslam_types::SLAMNode(
      robot_pose.frame_idx, robot_pose.loc, robot_pose.angle);
}

vslam_types::RobotPose FromSLAMNode(const vslam_types::SLAMNode& slam_node) {
  Eigen::Vector3f rotation_axis(
      slam_node.pose[3], slam_node.pose[4], slam_node.pose[5]);

  Eigen::AngleAxisf rotation_aa = VectorToAxisAngle(rotation_axis);

  Eigen::Vector3f transl(
      slam_node.pose[0], slam_node.pose[1], slam_node.pose[2]);
  return vslam_types::RobotPose(slam_node.node_idx, transl, rotation_aa);
}

void RobotPosesToSLAMNodes(
    const std::vector<vslam_types::RobotPose>& robot_poses,
    std::vector<vslam_types::SLAMNode>& nodes) {
  for (const vslam_types::RobotPose& robot_pose : robot_poses) {
    nodes.emplace_back(FromRobotPose(robot_pose));
  }
}

void SLAMNodesToRobotPoses(const std::vector<vslam_types::SLAMNode>& slam_nodes,
                           std::vector<vslam_types::RobotPose>& updated_poses) {
  updated_poses.clear();
  for (const vslam_types::SLAMNode& node : slam_nodes) {
    updated_poses.emplace_back(FromSLAMNode(node));
  }
}
}  // namespace vslam_util