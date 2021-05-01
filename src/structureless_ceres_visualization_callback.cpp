#include <draw_epipolar_lines.h>
#include <glog/logging.h>
#include <structureless_ceres_visualization_callback.h>
#include <vslam_types.h>

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
  const std::vector<vslam_types::SLAMNode> nodes = *slam_nodes_;

  // TODO add to class?
  Eigen::Affine3d cam_to_robot_tf =
      Eigen::Translation3d(extrinsics_.translation.cast<double>()) *
      extrinsics_.rotation.cast<double>();

  // Visualize epipolar lines and the epipolar error (width of lines)
  if (true) {
    VisualizeEpipolarError(cam_to_robot_tf, intrinsics_, slam_problem_, nodes);
  }

  // Visualize robot poses
  if (true) {
    pose_viz_.drawPoses(nodes);
  }

  return ceres::SOLVER_CONTINUE;
}

}  // namespace vslam_viz