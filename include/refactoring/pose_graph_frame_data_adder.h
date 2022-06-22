//
// Created by amanda on 6/20/22.
//

#ifndef UT_VSLAM_POSE_GRAPH_FRAME_DATA_ADDER_H
#define UT_VSLAM_POSE_GRAPH_FRAME_DATA_ADDER_H

#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/optimization/object_pose_graph.h>

namespace vslam_types_refactor {

void addVisualFeatureFactorsForFrame(
    const AssociatedBoundingBoxOfflineProblemData<StructuredVisionFeatureTrack,
                                                  RawBoundingBoxObservation,
                                                  EllipsoidState<double>>
        &input_problem_data,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FrameId &frame_to_add,
    const std::function<
        double(const AssociatedBoundingBoxOfflineProblemData<
                   StructuredVisionFeatureTrack,
                   RawBoundingBoxObservation,
                   EllipsoidState<double>> &,
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
void addFrameDataAssociatedBoundingBox(
    const AssociatedBoundingBoxOfflineProblemData<StructuredVisionFeatureTrack,
                                                  RawBoundingBoxObservation,
                                                  EllipsoidState<double>>
        &input_problem_data,
    const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
    const FrameId &frame_to_add,
    const std::function<
        double(const AssociatedBoundingBoxOfflineProblemData<
                   StructuredVisionFeatureTrack,
                   RawBoundingBoxObservation,
                   EllipsoidState<double>> &,
               const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
               const FrameId &,
               const FeatureId &,
               const CameraId &)> &reprojection_error_provider,
    const std::function<Covariance<double, 4>(
        const AssociatedBoundingBoxOfflineProblemData<
            StructuredVisionFeatureTrack,
            RawBoundingBoxObservation,
            EllipsoidState<double>> &,
        const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
        const FrameId &,
        const ObjectId &,
        const CameraId &)> &bb_covariance_provider) {
  Pose3D<double> pose_at_frame;
  if (!input_problem_data.getRobotPoseEstimateForFrame(frame_to_add,
                                                       pose_at_frame)) {
    // TODO should this find the closest frame to this node as a fallback
    // instead of just failing?
    LOG(ERROR) << "Could not find initial estimate for frame " << frame_to_add
               << "; not adding frame";
    return;
  }
  // Add initial estimate for pose
  pose_graph->addFrame(frame_to_add, pose_at_frame);

  // Get visual feature factors and the visual features that appear first in
  // this frame
  addVisualFeatureFactorsForFrame(input_problem_data,
                                  pose_graph,
                                  frame_to_add,
                                  reprojection_error_provider);

  // Add bounding box observations
  std::unordered_map<
      FrameId,
      std::unordered_map<
          CameraId,
          std::unordered_map<ObjectId, RawBoundingBoxObservation>>>
      bb_obs = input_problem_data.getBoundingBoxes();

  // Add initial ellipsoid estimate
  // TODO
  for (const auto &frame_id_and_bbs : bb_obs) {
    if (frame_id_and_bbs.first != frame_to_add) {
      continue;
    }
    for (const auto &cam_id_and_bbs : frame_id_and_bbs.second) {
      CameraId camera_id = cam_id_and_bbs.first;
      for (const auto &obj_id_and_bb : cam_id_and_bbs.second) {
        ObjectId obj_id = obj_id_and_bb.first;
        RawBoundingBoxObservation bb = obj_id_and_bb.second;
        ObjectObservationFactor obs_factor;
        obs_factor.object_id_ = obj_id;
        obs_factor.camera_id_ = camera_id;
        obs_factor.frame_id_ = frame_id_and_bbs.first;
        obs_factor.bounding_box_corners_ =
            cornerLocationsPairToVector(bb.bb_detection_.pixel_corner_locations_);
        obs_factor.bounding_box_corners_covariance_ = bb_covariance_provider(
            input_problem_data, pose_graph, frame_to_add, obj_id, camera_id);
      }
    }
  }
}
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_POSE_GRAPH_FRAME_DATA_ADDER_H
