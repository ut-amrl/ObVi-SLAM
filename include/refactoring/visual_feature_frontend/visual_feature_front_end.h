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
  std::map<FrameId, std::vector<ReprojectionErrorFactor>>
      frame_ids_and_reprojection_err_factors_;
  std::map<FrameId, std::optional<Pose3D<double>>> frame_ids_and_poses_;

  VisualFeatureCachedInfo() {}

  void addFactorsAndRobotPose(
      const FrameId &frame_id,
      const std::vector<ReprojectionErrorFactor> &reprojection_err_factors,
      const std::optional<Pose3D<double>> &robot_pose) {
    frame_ids_and_reprojection_err_factors_[frame_id] =
        reprojection_err_factors;
    frame_ids_and_poses_[frame_id] = robot_pose;
  }

  FrameId getMinFrameId() const {
    return frame_ids_and_reprojection_err_factors_.begin()->first;
  }

  std::vector<FrameId> getOrderedFrameIdsGreaterThan(
      const FrameId &min_frame_id) const {
    // TODO we can directly locate iterate associated with the given frame_id
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
      const std::function<bool(const FrameId &)> &gba_checker,
      const std::function<
          double(const ProblemDataType &,
                 const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
                 const FrameId &,
                 const FeatureId &,
                 const CameraId &)> &reprojection_error_provider,
      const double min_visual_feature_parallax_pixel_requirement,
      const double min_visual_feature_parallax_robot_transl_requirement,
      const double min_visual_feature_parallax_robot_orient_requirement)
      : gba_checker_(gba_checker),
        reprojection_error_provider_(reprojection_error_provider),
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
    std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
        visual_features = input_problem_data.getVisualFeatures();

    for (const auto &feat_id_to_track : visual_features) {
      FeatureId feature_id = feat_id_to_track.first;
      StructuredVisionFeatureTrack feature_track = feat_id_to_track.second;
      if (feature_track.feature_track.feature_observations_.find(
              max_frame_id) ==
          feature_track.feature_track.feature_observations_.end()) {
        continue;
      }
      VisionFeature feature =
          feature_track.feature_track.feature_observations_.at(max_frame_id);
      if (feature.frame_id_ != max_frame_id) {
        continue;
      }
      bool is_feature_added_to_pose_graph = false;
      if (added_feature_ids_.find(feature_id) != added_feature_ids_.end()) {
        is_feature_added_to_pose_graph = true;
      }
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
      if (is_feature_added_to_pose_graph) {
        for (const auto &vis_factor : reprojection_error_factors) {
          pose_graph->addVisualFactor(vis_factor);
        }
      } else {
        // add reprojection factors and initial poses to pendding list
        std::optional<Pose3D<double>> init_robot_pose;
        Pose3D<double> init_pose_est;
        std::optional<RawPose3d<double>> curr_pose_est_raw =
            pose_graph->getRobotPose(max_frame_id);
        if (input_problem_data.getRobotPoseEstimateForFrame(max_frame_id,
                                                            init_pose_est)) {
          init_robot_pose = std::make_optional(init_pose_est);
        }
        if (pending_feature_factors_.find(feature_id) ==
            pending_feature_factors_.end()) {
          pending_feature_factors_[feature_id] = VisualFeatureCachedInfo();
        }
        pending_feature_factors_[feature_id].addFactorsAndRobotPose(
            max_frame_id, reprojection_error_factors, init_robot_pose);
        // handling pending poses
        const auto &visual_feature_cache =
            pending_feature_factors_.at(feature_id);
        if (checkMinParallaxRequirements_(
                min_frame_id, max_frame_id, visual_feature_cache)) {
          Position3d<double> initial_position;
          getInitialFeaturePosition_(input_problem_data,
                                     pose_graph,
                                     feature_id,
                                     feat_id_to_track.second.feature_pos_,
                                     initial_position);
          pose_graph->addFeature(feature_id, initial_position);
          for (const auto &frame_id_and_factors :
               visual_feature_cache.frame_ids_and_reprojection_err_factors_) {
            for (const auto &factor : frame_id_and_factors.second) {
              pose_graph->addVisualFactor(factor);
            }
          }
          pending_feature_factors_.erase(feature_id);
          added_feature_ids_.insert(feature_id);
        }
      }
    }
    // if (gba_checker_(max_frame_id)) {
    //   LOG(INFO) << "start gba";
    //   for (const auto &feat_id_and_cache : pending_feature_factors_) {
    //     const FeatureId &feature_id = feat_id_and_cache.first;
    //     const auto &visual_feature_cache = feat_id_and_cache.second;
    //     if (checkMinParallaxRequirements_(
    //             min_frame_id, max_frame_id, visual_feature_cache)) {
    //       LOG(INFO) << "satisfy min parallax requirements";
    //       Position3d<double> initial_position;
    //       getInitialFeaturePosition_(
    //           input_problem_data,
    //           pose_graph,
    //           feature_id,
    //           visual_features.at(feature_id).feature_pos_,
    //           initial_position);
    //       LOG(INFO) << "after get getInitialFeaturePosition_";
    //       pose_graph->addFeature(feature_id, initial_position);
    //       for (const auto &frame_id_and_factors :
    //            visual_feature_cache.frame_ids_and_reprojection_err_factors_)
    //            {
    //         for (const auto &factor : frame_id_and_factors.second) {
    //           pose_graph->addVisualFactor(factor);
    //         }
    //       }
    //       pending_feature_factors_.erase(feature_id);
    //       added_feature_ids_.insert(feature_id);
    //     }
    //   }
    // }
    // LOG(INFO) << "end gba";
  }

 protected:
  std::function<bool(const FrameId &)> gba_checker_;
  std::function<double(
      const ProblemDataType &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      const FrameId &,
      const FeatureId &,
      const CameraId &)>
      reprojection_error_provider_;
  std::unordered_map<FeatureId, FrameId> first_observed_frame_by_feature_;
  std::unordered_set<FeatureId> added_feature_ids_;
  std::unordered_map<FeatureId, VisualFeatureCachedInfo>
      pending_feature_factors_;

  double min_visual_feature_parallax_pixel_requirement_ = 5;
  double min_visual_feature_parallax_robot_transl_requirement_ = 0.1;
  double min_visual_feature_parallax_robot_orient_requirement_ = 0.05;
  bool enforce_min_pixel_parallax_requirement_ = true;
  bool enforce_min_robot_pose_parallax_requirement_ = true;

 private:
  bool getInitialFeaturePosition_(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FeatureId &feature_id,
      const Position3d<double> &unadjusted_feature_pos,
      Position3d<double> &adjusted_initial_position) {
    FrameId first_frame_id =
        pending_feature_factors_.at(feature_id).getMinFrameId();
    Pose3D<double> init_first_pose;
    std::optional<RawPose3d<double>> optim_first_raw_pose =
        pose_graph->getRobotPose(first_frame_id);
    if ((!input_problem_data.getRobotPoseEstimateForFrame(first_frame_id,
                                                          init_first_pose)) ||
        (!optim_first_raw_pose.has_value())) {
      adjusted_initial_position = unadjusted_feature_pos;
      return false;
    } else {
      Position3d<double> relative_initial_position =
          getPositionRelativeToPose(init_first_pose, unadjusted_feature_pos);
      Pose3D<double> optim_first_pose =
          convertToPose3D(optim_first_raw_pose.value());
      adjusted_initial_position =
          combinePoseAndPosition(optim_first_pose, relative_initial_position);
      return true;
    }
  }

  bool checkMinParallaxRequirements_(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      const VisualFeatureCachedInfo &visualFeatureCachedInfo) const {
    std::vector<FrameId> frame_ids =
        visualFeatureCachedInfo.getOrderedFrameIdsGreaterThan(min_frame_id);
    if (frame_ids.size() <= 1) {
      return false;
    }

    if (!enforce_min_robot_pose_parallax_requirement_) {
      goto check_pixel_requirement;
    }

    for (size_t i = 0; i < frame_ids.size() - 1; ++i) {
      FrameId frame_id1 = frame_ids[i];
      std::optional<Pose3D<double>> robot_pose1 =
          visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id1);
      for (size_t j = i + 1; j < frame_ids.size(); ++j) {
        FrameId frame_id2 = frame_ids[j];
        std::optional<Pose3D<double>> robot_pose2 =
            visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id2);
        if (robot_pose1.has_value() && robot_pose2.has_value()) {
          Pose3D<double> relative_pose =
              getPose2RelativeToPose1(robot_pose1.value(), robot_pose2.value());
          if ((relative_pose.transl_.norm() >=
               min_visual_feature_parallax_robot_transl_requirement_) ||
              (relative_pose.orientation_.angle() >=
               min_visual_feature_parallax_robot_orient_requirement_)) {
            return true;
          }
        }
      }
    }

    if (!enforce_min_robot_pose_parallax_requirement_) {
      goto exit;
    }

  check_pixel_requirement:
    for (size_t i = 0; i < frame_ids.size() - 1; ++i) {
      FrameId frame_id1 = frame_ids[i];
      std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels1 =
          visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id1);
      for (size_t j = i + 1; j < frame_ids.size(); ++j) {
        FrameId frame_id2 = frame_ids[j];
        std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels2 =
            visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id2);
        for (const auto &cam_id_and_pixel1 : cam_ids_and_pixels1) {
          const PixelCoord<double> &pixel1 = cam_id_and_pixel1.second;
          for (const auto &cam_id_and_pixel2 : cam_ids_and_pixels2) {
            const PixelCoord<double> &pixel2 = cam_id_and_pixel2.second;
            double pixel_displacement = (pixel1 - pixel2).norm();
            if (pixel_displacement >=
                min_visual_feature_parallax_pixel_requirement_) {
              return true;
            }
          }
        }
      }
    }

    // for (size_t i = 0; i < frame_ids.size() - 1; ++i) {
    //   FrameId frame_id1 = frame_ids[i];
    //   std::optional<Pose3D<double>> robot_pose1 =
    //       visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id1);
    //   std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels1 =
    //       visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id1);
    //   for (size_t j = i + 1; j < frame_ids.size(); ++j) {
    //     FrameId frame_id2 = frame_ids[j];
    //     std::optional<Pose3D<double>> robot_pose2 =
    //         visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id2);
    //     std::optional<Pose3D<double>> relative_pose;
    //     if (robot_pose1.has_value() && robot_pose2.has_value()) {
    //       relative_pose =
    //       std::optional<Pose3D<double>>(getPose2RelativeToPose1(
    //           robot_pose1.value(), robot_pose2.value()));
    //     }
    //     std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels2
    //     =
    //         visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id2);
    //     for (const auto cam_id_and_pixel1 : cam_ids_and_pixels1) {
    //       const PixelCoord<double> &pixel1 = cam_id_and_pixel1.second;
    //       for (const auto cam_id_and_pixel2 : cam_ids_and_pixels2) {
    //         const PixelCoord<double> &pixel2 = cam_id_and_pixel2.second;
    //         double pixel_displacement = (pixel1 - pixel2).norm();
    //         if (pixel_displacement >=
    //             min_visual_feature_parallax_pixel_requirement_) {
    //           if (relative_pose.has_value()) {
    //             if ((relative_pose.value().transl_.norm() >=
    //                  min_visual_feature_parallax_robot_transl_requirement_)
    //                  &&
    //                 (relative_pose.value().orientation_.angle() >=
    //                  min_visual_feature_parallax_robot_orient_requirement_))
    //                  {
    //               return true;
    //             }
    //           } else {
    //             return true;
    //           }
    //         }
    //       }
    //     }
    //   }
    // }
  exit:
    return false;
  }
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VISUAL_FEATURE_FRONT_END_H
