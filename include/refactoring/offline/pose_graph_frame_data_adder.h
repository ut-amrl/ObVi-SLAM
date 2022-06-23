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
    for (size_t i = 0;
         i < feat_id_to_track.second.feature_track.feature_observations_.size();
         i++) {
      VisionFeature feature =
          feat_id_to_track.second.feature_track.feature_observations_[i];
      if (feature.frame_id_ == frame_to_add) {
        if (i == 0) {
          // The feature has not been added yet (this is the first observation)
          // so we need to add the feature
          pose_graph->addFeature(feat_id_to_track.first,
                                 feat_id_to_track.second.feature_pos_);
        }
        for (const auto &obs_by_camera : feature.pixel_by_camera_id) {
          ReprojectionErrorFactor vis_factor;
          vis_factor.camera_id_ = obs_by_camera.first;
          vis_factor.feature_id_ = feat_id_to_track.first;
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
      } else if (feature.frame_id_ > frame_to_add) {
        // Features are stored in their tracks sorted by frame id, so if we've
        // hit a frame greater than what we're trying to add, we won't find any
        // matching frames after this
        break;
      }
    }
  }
}

// TODO maybe make generic to both types of object pose graphs
template <typename ObjectAssociationInfo,
          typename RawBoundingBoxContextInfo,
          typename RefinedBoundingBoxContextInfo,
          typename SingleBbContextInfo,
          typename ProblemDataType>
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
                                    SingleBbContextInfo>>(
        const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
        const ProblemDataType &)> bb_associator_retriever,
    const std::function<RawBoundingBoxContextInfo(
        const FrameId &, const CameraId &, const ProblemDataType &)>
        &bb_context_retriever) {
  Pose3D<double> pose_at_frame;
  if (!input_problem_data.getRobotPoseEstimateForFrame(frame_to_add,
                                                       pose_at_frame)) {
    // TODO should this find the closest frame to this node as a fallback
    // instead of just failing?
    LOG(ERROR) << "Could not find initial estimate for frame " << frame_to_add
               << "; not adding frame";
    return;
  }

  // TODO tweak this to get difference between frame and previous and then use
  // the latest pose estimate for the previous frame to update the estimate for
  // the current

  // Add initial estimate for pose
  pose_graph->addFrame(frame_to_add, pose_at_frame);

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
                                              SingleBbContextInfo>>
      bb_associator = bb_associator_retriever(pose_graph, input_problem_data);

  if (bb_obs.find(frame_to_add) != bb_obs.end()) {
    for (const auto &cam_id_and_bbs : bb_obs.at(frame_to_add)) {
      bb_associator->addBoundingBoxObservations(
          frame_to_add,
          cam_id_and_bbs.first,
          cam_id_and_bbs.second,
          bb_context_retriever(
              frame_to_add, cam_id_and_bbs.first, input_problem_data));
    }
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_POSE_GRAPH_FRAME_DATA_ADDER_H
