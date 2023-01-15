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
          Pose3D<double> init_pose_est;
          std::optional<RawPose3d<double>> curr_pose_est_raw =
              pose_graph->getRobotPose(frame_to_add);
          if ((!input_problem_data.getRobotPoseEstimateForFrame(
                  frame_to_add, init_pose_est)) ||
              (!curr_pose_est_raw.has_value())) {
            LOG(WARNING) << "Could not find initial or current pose  estimate "
                            "for robot for frame "
                         << frame_to_add
                         << ", not adjusting initial feature position";

            // The feature has not been added yet (this is the first
            // observation) so we need to add the feature
            pose_graph->addFeature(feature_id,
                                   feat_id_to_track.second.feature_pos_);
          } else {
            Position3d<double> relative_initial_position =
                getPositionRelativeToPose(init_pose_est,
                                          feat_id_to_track.second.feature_pos_);
            Pose3D<double> curr_pose_est =
                convertToPose3D(curr_pose_est_raw.value());
            Position3d<double> adjusted_initial_position =
                combinePoseAndPosition(curr_pose_est,
                                       relative_initial_position);
            pose_graph->addFeature(feature_id, adjusted_initial_position);
          }
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
    const std::function<
        bool(const FrameId &,
             std::unordered_map<CameraId, std::vector<RawBoundingBox>> &)>
        &bb_retriever,
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

  Pose3D<double> curr_pose_init;
  if (!input_problem_data.getRobotPoseEstimateForFrame(
          frame_to_add, curr_pose_init)) {
    LOG(ERROR) << "Could not find pose initialization for frame " << frame_to_add
               << "; not adding frame";
    exit(1);
    return;
  }
  LOG(INFO) << "Adding frame " << frame_to_add;

  if (frame_to_add == 0) {
    pose_graph->addFrame(frame_to_add, curr_pose_init);
  } else {
    Pose3D<double> prev_pose_init;
    if (!input_problem_data.getRobotPoseEstimateForFrame(frame_to_add - 1,
                                                         prev_pose_init)) {
      LOG(ERROR) << "Could not find initial estimate for frame "
                 << frame_to_add - 1 << "; not adjusting subsequent";
      pose_graph->addFrame(frame_to_add, curr_pose_init);
    } else {
      Pose3D<double> prev_pose_est;
      std::optional<RawPose3d<double>> prev_raw_pose_est =
            pose_graph->getRobotPose(frame_to_add-1);
      // Only correct pose if there exist initial current pose, 
      // estimated current pose, and estimated previous pose
      if (prev_raw_pose_est.has_value()) {
        prev_pose_est = convertToPose3D(prev_raw_pose_est.value());
        Pose3D<double> relative_pose = getPose2RelativeToPose1(
            prev_pose_init, curr_pose_init);
        Pose3D<double> curr_pose_corrected_init =
            combinePoses(prev_pose_est, relative_pose);
        pose_graph->addFrame(frame_to_add, curr_pose_corrected_init);
      } else {
        LOG(ERROR) << "Could not find revised estimate for frame "
                   << frame_to_add - 1 << "; not adjusting subsequent";
        pose_graph->addFrame(frame_to_add, curr_pose_init);
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
  std::unordered_map<CameraId, std::vector<RawBoundingBox>>
      bounding_boxes_for_frame;
  if (!bb_retriever(frame_to_add, bounding_boxes_for_frame)) {
    LOG(WARNING) << "Could not get bounding boxes for frame " << frame_to_add;
    return;
  }

  std::shared_ptr<AbstractBoundingBoxFrontEnd<ReprojectionErrorFactor,
                                              ObjectAssociationInfo,
                                              RawBoundingBoxContextInfo,
                                              RefinedBoundingBoxContextInfo,
                                              SingleBbContextInfo,
                                              FrontEndObjMapData>>
      bb_associator = bb_associator_retriever(pose_graph, input_problem_data);

  if (!bounding_boxes_for_frame.empty()) {
    for (const auto &cam_id_and_bbs : bounding_boxes_for_frame) {
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
