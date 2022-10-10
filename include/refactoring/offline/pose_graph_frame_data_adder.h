//
// Created by amanda on 6/20/22.
//

#ifndef UT_VSLAM_POSE_GRAPH_FRAME_DATA_ADDER_H
#define UT_VSLAM_POSE_GRAPH_FRAME_DATA_ADDER_H

#include <refactoring/bounding_box_frontend/bounding_box_front_end.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/optimization/object_pose_graph.h>

namespace vslam_types_refactor {

template <typename ProblemDataType>
void addVisualFeatureFactorsForFrame(
    const ProblemDataType &input_problem_data,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FrameId &frame_to_add,
    const std::function<
        double(const ProblemDataType &,
               const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
               const FrameId &,
               const FeatureId &,
               const CameraId &)> &reprojection_error_provider) {
  // Add visual factors for frame
  std::unordered_map<FeatureId, StructuredVisionFeatureTrack> visual_features =
      input_problem_data.getVisualFeatures();

  for (const auto &feat_id_to_track : visual_features) {
    FeatureId feature_id = feat_id_to_track.first;
    StructuredVisionFeatureTrack feature_track = feat_id_to_track.second;
    if (feature_track.feature_track.feature_observations_.find(frame_to_add) !=
        feature_track.feature_track.feature_observations_.end()) {
      VisionFeature feature =
          feature_track.feature_track.feature_observations_.at(frame_to_add);
      if (feature.frame_id_ == frame_to_add) {
        FrameId junk_frame_id;
        if (!pose_graph->getFirstObservedFrameForFeature(feature_id,
                                                         junk_frame_id)) {
          // The feature has not been added yet (this is the first observation)
          // so we need to add the feature
          pose_graph->addFeature(feature_id,
                                 feat_id_to_track.second.feature_pos_);
        }
        for (const auto &obs_by_camera : feature.pixel_by_camera_id) {
          ReprojectionErrorFactor vis_factor;
          vis_factor.camera_id_ = obs_by_camera.first;
          vis_factor.feature_id_ = feature_id;
          vis_factor.frame_id_ = feature.frame_id_;
          vis_factor.feature_pos_ = obs_by_camera.second;
          vis_factor.reprojection_error_std_dev_ =
              reprojection_error_provider(input_problem_data,
                                          pose_graph,
                                          frame_to_add,
                                          vis_factor.feature_id_,
                                          vis_factor.camera_id_);
          pose_graph->addVisualFactor(vis_factor);
        }
      }
    }
  }
}

// TODO maybe make generic to both types of object pose graphs
template <typename ObjectAssociationInfo,
          typename RawBoundingBoxContextInfo,
          typename RefinedBoundingBoxContextInfo,
          typename SingleBbContextInfo,
          typename ProblemDataType,
          typename FrontEndObjMapData>
void addFrameDataAssociatedBoundingBox(
    const ProblemDataType &input_problem_data,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FrameId &frame_to_add,
    const std::function<
        double(const ProblemDataType &,
               const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
               const FrameId &,
               const FeatureId &,
               const CameraId &)> &reprojection_error_provider,
    const std::function<std::shared_ptr<
        AbstractBoundingBoxFrontEnd<ReprojectionErrorFactor,
                                    ObjectAssociationInfo,
                                    RawBoundingBoxContextInfo,
                                    RefinedBoundingBoxContextInfo,
                                    SingleBbContextInfo,
                                    FrontEndObjMapData>>(
        const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
        const ProblemDataType &)> bb_associator_retriever,
    const std::function<std::pair<bool, RawBoundingBoxContextInfo>(
        const FrameId &, const CameraId &, const ProblemDataType &)>
        &bb_context_retriever) {
  Pose3D<double> pose_at_frame_init_est;
  if (!input_problem_data.getRobotPoseEstimateForFrame(
          frame_to_add, pose_at_frame_init_est)) {
    // TODO should this find the closest frame to this node as a fallback
    // instead of just failing?
    LOG(ERROR) << "Could not find initial estimate for frame " << frame_to_add
               << "; not adding frame";
    exit(1);
    return;
  }
  LOG(INFO) << "Adding frame " << frame_to_add;
  if (frame_to_add == 0) {
    pose_graph->addFrame(frame_to_add, pose_at_frame_init_est);
  } else {
    // Get difference between frame and previous and then use the latest pose
    // estimate for the previous frame to update the estimate for the current
    Pose3D<double> prev_pose_init;

    if (!input_problem_data.getRobotPoseEstimateForFrame(frame_to_add - 1,
                                                         prev_pose_init)) {
      // TODO should this find the closest frame to this node as a fallback
      // instead of just failing?
      LOG(ERROR) << "Could not find initial estimate for frame "
                 << frame_to_add - 1 << "; not adjusting subsequent";
      pose_graph->addFrame(frame_to_add, pose_at_frame_init_est);
    } else {
      std::optional<RawPose3d<double>> revised_prev_pose_raw =
          pose_graph->getRobotPose(frame_to_add - 1);
      if (revised_prev_pose_raw.has_value()) {
        // Add initial estimate for pose
        Pose3D<double> relative_pose = getPose2RelativeToPose1(
            convertToPose3D(revised_prev_pose_raw.value()),
            pose_at_frame_init_est);
        Pose3D<double> corrected_pose_at_frame =
            combinePoses(prev_pose_init, relative_pose);
        pose_graph->addFrame(frame_to_add, corrected_pose_at_frame);
      } else {
        LOG(ERROR) << "Could not find revised estimate for frame "
                   << frame_to_add - 1 << "; not adjusting subsequent";
        pose_graph->addFrame(frame_to_add, pose_at_frame_init_est);
      }
    }
  }

  // Get visual feature factors and the visual features that appear first in
  // this frame
  addVisualFeatureFactorsForFrame(input_problem_data,
                                  pose_graph,
                                  frame_to_add,
                                  reprojection_error_provider);

  // Add bounding box observations
  std::unordered_map<FrameId,
                     std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
      bb_obs = input_problem_data.getBoundingBoxes();
  std::shared_ptr<AbstractBoundingBoxFrontEnd<ReprojectionErrorFactor,
                                              ObjectAssociationInfo,
                                              RawBoundingBoxContextInfo,
                                              RefinedBoundingBoxContextInfo,
                                              SingleBbContextInfo,
                                              FrontEndObjMapData>>
      bb_associator = bb_associator_retriever(pose_graph, input_problem_data);

  if (bb_obs.find(frame_to_add) != bb_obs.end()) {
    for (const auto &cam_id_and_bbs : bb_obs.at(frame_to_add)) {
      std::pair<bool, RawBoundingBoxContextInfo> context = bb_context_retriever(
          frame_to_add, cam_id_and_bbs.first, input_problem_data);
      if (context.first) {
        bb_associator->addBoundingBoxObservations(frame_to_add,
                                                  cam_id_and_bbs.first,
                                                  cam_id_and_bbs.second,
                                                  context.second);
      }
    }
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_POSE_GRAPH_FRAME_DATA_ADDER_H
