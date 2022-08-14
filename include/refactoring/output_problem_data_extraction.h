//
// Created by amanda on 6/18/22.
//

#ifndef UT_VSLAM_OUTPUT_PROBLEM_DATA_EXTRACTION_H
#define UT_VSLAM_OUTPUT_PROBLEM_DATA_EXTRACTION_H

#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/output_problem_data.h>

namespace vslam_types_refactor {

// TODO can we make this generic to any object pose graph?
void extractEllipsoidEstimates(
    const std::shared_ptr<
        const vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    EllipsoidResults &output_data) {
  std::unordered_map<ObjectId, std::pair<std::string, RawEllipsoid<double>>>
      raw_ests;
  pose_graph->getObjectEstimates(raw_ests);
  for (const auto &raw_est : raw_ests) {
    output_data.ellipsoids_[raw_est.first] = std::make_pair(
        raw_est.second.first, convertToEllipsoidState(raw_est.second.second));
  }
}

// TODO can we make this generic to any object pose graph?
void extractRobotPoseEstimates(
    const std::shared_ptr<
        const vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    RobotPoseResults &output_data) {
  std::unordered_map<FrameId, RawPose3d<double>> raw_ests;
  pose_graph->getRobotPoseEstimates(raw_ests);
  for (const auto &raw_est : raw_ests) {
    output_data.robot_poses_[raw_est.first] = convertToPose3D(raw_est.second);
  }
}

void extractVisualFeaturePositionEstimates(
    const std::shared_ptr<
        const vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    VisualFeatureResults &output_data) {
  pose_graph->getVisualFeatureEstimates(output_data.visual_feature_positions_);
}

void extractSpatialEstimateOnlyResults(
    const std::shared_ptr<
        const vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    vslam_types_refactor::SpatialEstimateOnlyResults &output_data) {
  extractEllipsoidEstimates(pose_graph, output_data.ellipsoid_results_);
  extractRobotPoseEstimates(pose_graph, output_data.robot_pose_results_);
  extractVisualFeaturePositionEstimates(pose_graph,
                                        output_data.visual_feature_results_);
}

template <typename LongTermObjectMap>
void extractLongTermObjectMapAndResults(
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    ceres::Problem *problem,
    const std::function<
        bool(const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
             ceres::Problem *,
             LongTermObjectMap &)> long_term_object_map_extractor,
    vslam_types_refactor::LongTermObjectMapAndResults<LongTermObjectMap>
        &output_data) {
  long_term_object_map_extractor(
      pose_graph, problem, output_data.long_term_map_);
  extractRobotPoseEstimates(pose_graph, output_data.robot_pose_results_);
  extractVisualFeaturePositionEstimates(pose_graph,
                                        output_data.visual_feature_results_);
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OUTPUT_PROBLEM_DATA_EXTRACTION_H
