#ifndef UT_VSLAM_CERES_VISUALIZATION_CALLBACK_H
#define UT_VSLAM_CERES_VISUALIZATION_CALLBACK_H

#include <ceres/iteration_callback.h>
#include <draw_epipolar_lines.h>
#include <pose_viewer.h>
#include <vslam_types.h>

#include <memory>
#include <opencv2/core/eigen.hpp>

namespace vslam_viz {

/**
 * Callback that is run on each iteration of the ceres optimization.
 *
 * Used for visualization of structureless SLAM.
 */
template <typename FeatureTrackType>
class VisualSlamCeresVisualizationCallback : public ceres::IterationCallback {
 public:
  /**
   * Constructor.
   *
   * @param slam_problem        SLAM problem.
   * @param feature_retriever   Retrieves visual features from a feature track.
   * @param gt_robot_poses      Ground truth robot poses to display.
   * @param slam_nodes          SLAM nodes being optimized by ceres. These
   *                            should only be read, not modified.
   */
  VisualSlamCeresVisualizationCallback(
      const vslam_types::UTSLAMProblem<FeatureTrackType> &slam_problem,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const FeatureTrackType &)> &feature_retriever,
      const std::vector<vslam_types::RobotPose> &gt_robot_poses,
      std::vector<vslam_types::SLAMNode> *slam_nodes)
      : slam_problem_(slam_problem),
        feature_retriever_(feature_retriever),
        gt_robot_poses_(gt_robot_poses),
        slam_nodes_(slam_nodes),
        pose_viz_(gt_robot_poses_) {}

  /**
   * Function that is called during ceres optimization.
   *
   * @param summary Ceres optimization summary.
   *
   * @return Return type indicating if ceres should continue.
   */
  ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) {
    // DO NOT MODIFY THESE -- they are being updated during optimization
    const std::vector<vslam_types::SLAMNode> nodes = *slam_nodes_;

    // Visualize epipolar lines and the epipolar error (width of lines)
    if (true) {
      VisualizeEpipolarError(slam_problem_, nodes, feature_retriever_);
    }

    // Visualize robot poses
    if (true) {
      pose_viz_.drawPoses(nodes);
    }

    return ceres::SOLVER_CONTINUE;
  }

  /**
   * Static function used to create a pointer to this visualization callback.
   *
   * @param slam_problem        SLAM problem.
   * @param feature_retriever   Retrieves visual features from a feature track.
   * @param gt_robot_poses      Ground truth robot poses to display.
   * @param slam_nodes          SLAM nodes being optimized by ceres. These
   *                            should only be read, not modified.
   *
   * @return Pointer to visualization callback.
   */
  template <typename TrackType>
  static std::shared_ptr<VisualSlamCeresVisualizationCallback> create(
      const vslam_types::UTSLAMProblem<TrackType> &slam_problem,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const TrackType &)> &feature_retriever,
      const std::vector<vslam_types::RobotPose> &gt_robot_poses,
      std::vector<vslam_types::SLAMNode> *slam_nodes) {
    return std::make_shared<VisualSlamCeresVisualizationCallback<TrackType>>(
        slam_problem, feature_retriever, gt_robot_poses, slam_nodes);
  }

 private:
  /**
   * SLAM problem with relevant image data.
   */
  vslam_types::UTSLAMProblem<FeatureTrackType> slam_problem_;

  /**
   * Function for retrieving features from a feature track.
   */
  std::function<std::vector<vslam_types::VisionFeature>(
      const FeatureTrackType &)>
      feature_retriever_;

  /**
   * Ground truth robot poses to display.
   */
  std::vector<vslam_types::RobotPose> gt_robot_poses_;

  /**
   * SLAM nodes that are being optimized. These should only be read, not
   * modified.
   */
  std::vector<vslam_types::SLAMNode> *slam_nodes_;

  vslam_viz::PoseViewer pose_viz_;
};
}  // namespace vslam_viz

#endif  // UT_VSLAM_CERES_VISUALIZATION_CALLBACK_H
