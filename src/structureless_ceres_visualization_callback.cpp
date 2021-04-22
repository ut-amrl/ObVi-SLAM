#include <glog/logging.h>
#include <structureless_ceres_visualization_callback.h>

namespace vslam_viz {

StructurelessCeresVisualizationCallback::
    StructurelessCeresVisualizationCallback(
        const vslam_types::CameraIntrinsics &intrinsics,
        const vslam_types::CameraExtrinsics &extrinsics,
        const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>
            &slam_problem,
        std::vector<vslam_types::SLAMNode> *slam_nodes)
    : intrinsics_(intrinsics),
      extrinsics_(extrinsics),
      slam_problem_(slam_problem),
      slam_nodes_(slam_nodes) {}

ceres::CallbackReturnType StructurelessCeresVisualizationCallback::operator()(
    const ceres::IterationSummary &summary) {
  // DO NOT MODIFY THESE -- they are being updated during optimization
  std::vector<vslam_types::SLAMNode> nodes = *slam_nodes_;

  for (int i = 0; i < nodes.size(); i++) {
    // TODO this can be removed once we add visualization -- just to demonstrate
    // that it is called
    const vslam_types::SLAMNode node = nodes[i];
    LOG(INFO) << "Node " << i << ": " << node.pose[0] << ", " << node.pose[1]
              << ", " << node.pose[2] << ", " << node.pose[3] << ", "
              << node.pose[4] << ", " << node.pose[5];
  }

  return ceres::SOLVER_CONTINUE;
}

}  // namespace vslam_viz