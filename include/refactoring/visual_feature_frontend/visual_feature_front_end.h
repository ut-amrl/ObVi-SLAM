//
// Created by taijing on 2/07/22.
//
#ifndef UT_VSLAM_VISUAL_FEATURE_FRONT_END_H
#define UT_VSLAM_VISUAL_FEATURE_FRONT_END_H

#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

struct VisualFeatureCachedInfo {
  Position3d<double> initial_position_;
  std::map<FrameId, std::vector<ReprojectionErrorFactor>>
      frame_ids_and_reprojection_err_factors_;
  std::map<FrameId, std::optional<Pose3D<double>>> frame_ids_and_poses_;

  VisualFeatureCachedInfo() {}
  VisualFeatureCachedInfo(const Position3d<double> &initial_position)
      : initial_position_(initial_position) {}

  void addFactorsAndRobotPose(
      const FrameId &frame_id,
      const std::vector<ReprojectionErrorFactor> &reprojection_err_factors,
      const std::optional<Pose3D<double>> &robot_pose) {
    frame_ids_and_reprojection_err_factors_[frame_id] =
        reprojection_err_factors;
    frame_ids_and_poses_[frame_id] = robot_pose;
  }

  std::vector<FrameId> getOrderedFrameIdsGreaterThan(
      const FrameId &min_frame_id) const {
    std::vector<FrameId> frame_ids;
    for (const auto &frame_id_and_factor :
         frame_ids_and_reprojection_err_factors_) {
      if (frame_id_and_factor.first >= min_frame_id) {
        frame_ids.emplace_back(frame_id_and_factor.first);
      }
    }
    return frame_ids;
  }

  std::unordered_map<CameraId, PixelCoord<double>> getCamIdsAndPixelsByFrame(
      const FrameId &frame_id) const {
    std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels;
    for (const auto &factor :
         frame_ids_and_reprojection_err_factors_.at(frame_id)) {
      cam_ids_and_pixels[factor.camera_id_] = factor.feature_pos_;
    }
    return cam_ids_and_pixels;
  }
};

// TODO support general pose graph
template <typename ProblemDataType>
class VisualFeatureFrontend {
 public:
  VisualFeatureFrontend(
      const std::function<
          double(const ProblemDataType &,
                 const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
                 const FrameId &,
                 const FeatureId &,
                 const CameraId &)> &reprojection_error_provider,
      double min_visual_feature_parallax_pixel_requirement,
      double min_visual_feature_parallax_robot_transl_requirement,
      double min_visual_feature_parallax_robot_orient_requirement)
      : reprojection_error_provider_(reprojection_error_provider),
        min_visual_feature_parallax_pixel_requirement_(
            min_visual_feature_parallax_pixel_requirement),
        min_visual_feature_parallax_robot_transl_requirement_(
            min_visual_feature_parallax_robot_transl_requirement),
        min_visual_feature_parallax_robot_orient_requirement_(
            min_visual_feature_parallax_robot_orient_requirement) {}

  /**
   * @brief
   *
   * @param min_frame_id
   * @param max_frame_id assume this is the same as frame_to_add
   * @param pose_graph
   * @param input_problem_data
   */
  void addVisualFeatureObservations(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id) {
    // LOG(INFO) << "start addVisualFeatureObservations";
    std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
        visual_features = input_problem_data.getVisualFeatures();

    for (const auto &feat_id_to_track : visual_features) {
      FeatureId feature_id = feat_id_to_track.first;
      StructuredVisionFeatureTrack feature_track = feat_id_to_track.second;
      if (feature_track.feature_track.feature_observations_.find(
              max_frame_id) !=
          feature_track.feature_track.feature_observations_.end()) {
        VisionFeature feature =
            feature_track.feature_track.feature_observations_.at(max_frame_id);
        if (feature.frame_id_ != max_frame_id) {
          continue;
        }
        // LOG(INFO) << "start inner loop";
        // find feature to add
        // if the feature has not yet been added to pose graph, nor
        // has it been added to the pending ones
        bool is_first_time_feature_observation = false;
        bool is_feature_added_to_pose_graph = false;
        if (added_feature_ids_.find(feature_id) == added_feature_ids_.end() &&
            pending_feature_factors_.find(feature_id) ==
                pending_feature_factors_.end()) {
          is_first_time_feature_observation = true;
        }
        if (added_feature_ids_.find(feature_id) != added_feature_ids_.end()) {
          is_feature_added_to_pose_graph = true;
        }

        // handle initial feature 3d position and current robot pose
        Position3d<double> initial_position;
        std::optional<Pose3D<double>> curr_robot_pose;
        Pose3D<double> init_pose_est;
        std::optional<RawPose3d<double>> curr_pose_est_raw =
            pose_graph->getRobotPose(max_frame_id);
        if ((!input_problem_data.getRobotPoseEstimateForFrame(max_frame_id,
                                                              init_pose_est)) ||
            (!curr_pose_est_raw.has_value())) {
          LOG(WARNING) << "Could not find initial or current pose  estimate "
                          "for robot for frame "
                       << max_frame_id
                       << ", not adjusting initial feature position";
          if (is_first_time_feature_observation) {
            initial_position = feat_id_to_track.second.feature_pos_;
          } else {
            curr_robot_pose = std::nullopt;
          }
        } else {
          Position3d<double> relative_initial_position =
              getPositionRelativeToPose(init_pose_est,
                                        feat_id_to_track.second.feature_pos_);
          Pose3D<double> curr_pose_est =
              convertToPose3D(curr_pose_est_raw.value());
          Position3d<double> adjusted_initial_position =
              combinePoseAndPosition(curr_pose_est, relative_initial_position);
          if (is_first_time_feature_observation) {
            initial_position = adjusted_initial_position;
          } else {
            curr_robot_pose = std::optional<Pose3D<double>>{curr_pose_est};
          }
        }
        // LOG(INFO) << "found robot pose";

        std::vector<ReprojectionErrorFactor> reprojection_error_factors;
        for (const auto &obs_by_camera : feature.pixel_by_camera_id) {
          ReprojectionErrorFactor vis_factor;
          vis_factor.camera_id_ = obs_by_camera.first;
          vis_factor.feature_id_ = feature_id;
          vis_factor.frame_id_ = feature.frame_id_;
          vis_factor.feature_pos_ = obs_by_camera.second;
          vis_factor.reprojection_error_std_dev_ =
              reprojection_error_provider_(input_problem_data,
                                           pose_graph,
                                           max_frame_id,
                                           vis_factor.feature_id_,
                                           vis_factor.camera_id_);
          reprojection_error_factors.push_back(vis_factor);
        }
        // LOG(INFO) << "got reprojection_errosr_factors";
        if (is_feature_added_to_pose_graph) {
          for (const auto &vis_factor : reprojection_error_factors) {
            pose_graph->addVisualFactor(vis_factor);
          }
        } else {
          if (is_first_time_feature_observation) {
            pending_feature_factors_[feature_id] =
                VisualFeatureCachedInfo(initial_position);
          }
          pending_feature_factors_[feature_id].addFactorsAndRobotPose(
              max_frame_id, reprojection_error_factors, curr_robot_pose);
        }

        // handling pending factors
        // LOG(INFO) << "handling pending factors";
        if (!is_feature_added_to_pose_graph) {
          const auto &visual_feature_cache =
              pending_feature_factors_.at(feature_id);
          if (checkMinParallaxRequirements(min_frame_id,
                                           max_frame_id,
                                           visual_feature_cache,
                                           curr_robot_pose)) {
            pose_graph->addFeature(feature_id,
                                   visual_feature_cache.initial_position_);
            for (const auto &frame_id_and_factors :
                 visual_feature_cache.frame_ids_and_reprojection_err_factors_) {
              for (const auto &factor : frame_id_and_factors.second) {
                pose_graph->addVisualFactor(factor);
              }
            }
            pending_feature_factors_.erase(feature_id);
            added_feature_ids_.insert(feature_id);
          }
          // LOG(INFO) << "end inner loop";
        }
      }
    }
    // LOG(INFO) << "end addVisualFeatureObservations";
  }

 protected:
  const std::function<double(
      const ProblemDataType &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      const FrameId &,
      const FeatureId &,
      const CameraId &)> &reprojection_error_provider_;
  std::unordered_map<FeatureId, FrameId> first_observed_frame_by_feature_;
  std::unordered_set<FeatureId> added_feature_ids_;
  std::unordered_map<FeatureId, VisualFeatureCachedInfo>
      pending_feature_factors_;

  double min_visual_feature_parallax_pixel_requirement_ = 5;
  double min_visual_feature_parallax_robot_transl_requirement_ = 0.1;
  double min_visual_feature_parallax_robot_orient_requirement_ = 0.05;

 private:
  bool checkMinParallaxRequirements(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      const VisualFeatureCachedInfo &visualFeatureCachedInfo,
      const std::optional<Pose3D<double>> &robot_pose) const {
    std::vector<FrameId> frame_ids =
        visualFeatureCachedInfo.getOrderedFrameIdsGreaterThan(min_frame_id);
    if (frame_ids.size() <= 1) {
      return false;
    }

    for (size_t i = 0; i < frame_ids.size() - 1; ++i) {
      FrameId frame_id1 = frame_ids[i];
      std::optional<Pose3D<double>> robot_pose1 =
          visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id1);
      std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels1 =
          visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id1);
      for (size_t j = i + 1; j < frame_ids.size(); ++j) {
        FrameId frame_id2 = frame_ids[j];
        std::optional<Pose3D<double>> robot_pose2 =
            visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id2);
        std::optional<Pose3D<double>> relative_pose;
        if (robot_pose1.has_value() && robot_pose2.has_value()) {
          relative_pose = std::optional<Pose3D<double>>(getPose2RelativeToPose1(
              robot_pose1.value(), robot_pose2.value()));
        }
        std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels2 =
            visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id2);
        for (const auto cam_id_and_pixel1 : cam_ids_and_pixels1) {
          const PixelCoord<double> &pixel1 = cam_id_and_pixel1.second;
          for (const auto cam_id_and_pixel2 : cam_ids_and_pixels2) {
            const PixelCoord<double> &pixel2 = cam_id_and_pixel2.second;
            double pixel_displacement = (pixel1 - pixel2).norm();
            if (pixel_displacement >=
                min_visual_feature_parallax_pixel_requirement_) {
              if (relative_pose.has_value()) {
                if ((relative_pose.value().transl_.norm() >=
                     min_visual_feature_parallax_robot_transl_requirement_) &&
                    (relative_pose.value().orientation_.angle() >=
                     min_visual_feature_parallax_robot_orient_requirement_)) {
                  return true;
                }
              } else {
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  }
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VISUAL_FEATURE_FRONT_END_H