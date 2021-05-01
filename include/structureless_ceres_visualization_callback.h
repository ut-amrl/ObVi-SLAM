#ifndef UT_VSLAM_CERES_VISUALIZATION_CALLBACK_H
#define UT_VSLAM_CERES_VISUALIZATION_CALLBACK_H

#include <ceres/iteration_callback.h>
#include <pose_viewer.h>
#include <vslam_types.h>

#include <memory>

namespace vslam_viz {

/**
 * Callback that is run on each iteration of the ceres optimization.
 *
 * Used for visualization of structureless SLAM.
 */
class StructurelessCeresVisualizationCallback
    : public ceres::IterationCallback {
 public:
  /**
   * Constructor.
   *
   * TODO -- if we need more in this constructor for visualization, that's fine.
   *
   * @param intrinsics      Camera intrinsics.
   * @param extrinsics      Camera extrinsics.
   * @param slam_problem    SLAM problem.
   * @param slam_nodes      SLAM nodes being optimized by ceres. These should
   *                        only be read, not modified.
   */
  StructurelessCeresVisualizationCallback(
      const vslam_types::CameraIntrinsics &intrinsics,
      const vslam_types::CameraExtrinsics &extrinsics,
      const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>
          &slam_problem,
      std::vector<vslam_types::SLAMNode> *slam_nodes);

  /**
   * Function that is called during ceres optimization.
   *
   * @param summary Ceres optimization summary.
   *
   * @return Return type indicating if ceres should continue.
   */
  ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary);

  /**
   * Static function used to create a pointer to this visualization callback.
   *
   * TODO -- if we need more in this constructor for visualization, that's fine.
   *
   * @param intrinsics      Camera intrinsics.
   * @param extrinsics      Camera extrinsics.
   * @param slam_problem    SLAM problem.
   * @param slam_nodes      SLAM nodes being optimized by ceres. These should
   *                        only be read, not modified.
   *
   * @return Pointer to visualization callback.
   */
  static std::shared_ptr<StructurelessCeresVisualizationCallback> create(
      const vslam_types::CameraIntrinsics &intrinsics,
      const vslam_types::CameraExtrinsics &extrinsics,
      const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack>
          &slam_problem,
      std::vector<vslam_types::SLAMNode> *slam_nodes) {
    return std::make_shared<StructurelessCeresVisualizationCallback>(
        intrinsics, extrinsics, slam_problem, slam_nodes);
  }

 private:
  /**
   * Camera intrinsics.
   */
  vslam_types::CameraIntrinsics intrinsics_;

  /**
   * Camera extrinsics.
   */
  vslam_types::CameraExtrinsics extrinsics_;

  /**
   * SLAM problem with relevant image data.
   */
  vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> slam_problem_;

  /**
   * SLAM nodes that are being optimized. These should only be read, not
   * modified.
   */
  std::vector<vslam_types::SLAMNode> *slam_nodes_;

  vslam_viz::PoseViewer pose_viz_;
};
}  // namespace vslam_viz

#endif  // UT_VSLAM_CERES_VISUALIZATION_CALLBACK_H
