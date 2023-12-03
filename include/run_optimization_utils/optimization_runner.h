//
// Created by amanda on 7/12/23.
//

#ifndef UT_VSLAM_OPTIMIZATION_RUNNER_H
#define UT_VSLAM_OPTIMIZATION_RUNNER_H

#include <debugging/optimization_logger.h>
#include <file_io/cv_file_storage/object_and_reprojection_feature_pose_graph_file_storage_io.h>
#include <refactoring/configuration/full_ov_slam_config.h>
#include <refactoring/long_term_map/long_term_map_factor_creator.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/offline/pose_graph_frame_data_adder.h>
#include <refactoring/optimization/pose_graph_utils.h>
#include <refactoring/visual_feature_frontend/visual_feature_front_end.h>
#include <ros/ros.h>
#include <run_optimization_utils/run_opt_utils.h>

namespace vslam_types_refactor {

bool runFullOptimization(
    std::optional<OptimizationLogger> &opt_logger,
    const FullOVSLAMConfig &config,
    const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
        &camera_intrinsics_by_camera,
    const std::unordered_map<CameraId, CameraExtrinsics<double>>
        &camera_extrinsics_by_camera,
    const std::unordered_map<
        FrameId,
        std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
        &bounding_boxes,
    const std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
        &visual_features,
    const std::unordered_map<FrameId, Pose3D<double>> &robot_poses,
    const MainLtmPtr &long_term_map,
    const std::function<void(const MainProbData &, MainPgPtr &)>
        &pose_graph_creator,
    const std::unordered_map<
        FrameId,
        std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>> &images,
    const std::string &output_checkpoints_dir,
    const std::string &jacobian_debug_output_dir,
    const std::function<
        bool(const FrameId &,
             const MainProbData &input_prob_data,
             std::unordered_map<CameraId, std::vector<RawBoundingBox>> &)>
        &input_data_dependent_bb_retriever,
    const std::function<void(
        const std::unordered_map<CameraId, std::pair<double, double>> &,
        const std::shared_ptr<std::unordered_map<
            FrameId,
            std::unordered_map<CameraId,
                               std::vector<std::pair<BbCornerPair<double>,
                                                     std::optional<double>>>>>>
            &,
        const std::shared_ptr<std::unordered_map<
            FrameId,
            std::unordered_map<
                CameraId,
                std::unordered_map<ObjectId,
                                   std::pair<BbCornerPair<double>,
                                             std::optional<double>>>>>> &,
        const std::shared_ptr<std::vector<std::unordered_map<
            FrameId,
            std::unordered_map<CameraId,
                               std::pair<BbCornerPair<double>, double>>>>> &,
        const std::shared_ptr<std::vector<
            std::pair<std::string, std::optional<EllipsoidState<double>>>>> &,
        const std::unordered_map<
            FrameId,
            std::unordered_map<
                CameraId,
                std::unordered_map<FeatureId, PixelCoord<double>>>> &,
        const MainProbData &,
        const MainPgPtr &,
        const FrameId &,
        const FrameId &,
        const VisualizationTypeEnum &,
        const int &)> &visualization_callback,
    IndependentEllipsoidsLongTermObjectMapFactorCreator<util::EmptyStruct,
                                                        util::EmptyStruct>
        &ltm_factor_creator,
    LongTermObjectMapAndResults<MainLtm> &output_results,
    const FrameId &start_opt_at_frame = 0,
    const bool &run_data_adder_for_first_frame = true) {
#ifdef RUN_TIMERS
  // Create an instance so that the factory never goes out of scope
  CumulativeTimerFactory &instance = CumulativeTimerFactory::getInstance();
#endif

  pose_graph_optimization::OptimizationIterationParams
      local_ba_iteration_params = config.local_ba_iteration_params_;

  pose_graph_optimization::OptimizationIterationParams
      global_ba_iteration_params = config.global_ba_iteration_params_;

  pose_graph_optimization::OptimizationIterationParams
      final_opt_iteration_params = config.final_ba_iteration_params_;

  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params =
      config.object_visual_pose_graph_residual_params_;

  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<
          CameraId,
          std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>>>>
      all_observed_corner_locations_with_uncertainty =
          std::make_shared<std::unordered_map<
              FrameId,
              std::unordered_map<
                  CameraId,
                  std::vector<std::pair<BbCornerPair<double>,
                                        std::optional<double>>>>>>();
//  for (const auto &bounding_boxes_for_frame : bounding_boxes) {
//    for (const auto &bounding_boxes_for_frame_and_cam :
//         bounding_boxes_for_frame.second) {
//      std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>
//          observed_corners_for_frame_and_cam;
//      for (const RawBoundingBox &bb : bounding_boxes_for_frame_and_cam.second) {
//        observed_corners_for_frame_and_cam.emplace_back(std::make_pair(
//            bb.pixel_corner_locations_, bb.detection_confidence_));
//      }
//      (*all_observed_corner_locations_with_uncertainty)
//          [bounding_boxes_for_frame.first]
//          [bounding_boxes_for_frame_and_cam.first] =
//              observed_corners_for_frame_and_cam;
//    }
//  }

  std::unordered_map<CameraId, std::pair<double, double>>
      img_heights_and_widths;
  for (const auto &frame_and_imgs : images) {
    for (const auto &cam_and_img : frame_and_imgs.second) {
      if (img_heights_and_widths.find(cam_and_img.first) ==
          img_heights_and_widths.end()) {
        sensor_msgs::Image::ConstPtr img = cam_and_img.second;
        img_heights_and_widths[cam_and_img.first] =
            std::make_pair(img->height, img->width);
      }
    }
  }

  std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::unordered_map<FeatureId, PixelCoord<double>>>>
      low_level_features_map;

  for (const auto &feature_track : visual_features) {
    FeatureId feat_id = feature_track.first;
    for (const auto &feat_obs_by_frame :
         feature_track.second.feature_track.feature_observations_) {
      FrameId frame_id = feat_obs_by_frame.first;
      for (const auto &feat_obs_for_cam :
           feat_obs_by_frame.second.pixel_by_camera_id) {
        CameraId cam_id = feat_obs_for_cam.first;
        low_level_features_map[frame_id][cam_id][feat_id] =
            feat_obs_for_cam.second;
      }
    }
  }

  MainProbData input_problem_data(
      camera_intrinsics_by_camera,
      camera_extrinsics_by_camera,
      visual_features,
      robot_poses,
      config.shape_dimension_priors_.mean_and_cov_by_semantic_class_,
      bounding_boxes,
      long_term_map,
      images);

  std::function<bool(
      const FrameId &,
      std::unordered_map<CameraId, std::vector<RawBoundingBox>> &)>
      bb_retriever = std::bind(input_data_dependent_bb_retriever,
                               std::placeholders::_1,
                               input_problem_data,
                               std::placeholders::_2);

  std::function<bool()> continue_opt_checker = []() { return ros::ok(); };

  FrameId max_frame_id = 0;
  for (const auto &frame_and_pose : robot_poses) {
    max_frame_id = std::max(max_frame_id, frame_and_pose.first);
  }

  std::function<FrameId(const FrameId &)> window_provider_func =
      std::bind(provideOptimizationWindow,
                std::placeholders::_1,
                max_frame_id,
                config.sliding_window_params_);
  std::function<bool(const FrameId &)> gba_checker =
      [&](const FrameId &max_frame_to_opt) -> bool {
    FrameId min_frame_to_opt = window_provider_func(max_frame_to_opt);
    if (max_frame_to_opt - min_frame_to_opt <=
        config.sliding_window_params_.local_ba_window_size_) {
      return false;
    }
    return true;
  };
  std::function<pose_graph_optimization::OptimizationIterationParams(
      const FrameId &)>
      solver_params_provider_func = [&](const FrameId &max_frame_to_opt)
      -> pose_graph_optimization::OptimizationIterationParams {
    if (max_frame_to_opt == max_frame_id) {
      return final_opt_iteration_params;
    } else if ((max_frame_to_opt %
                config.sliding_window_params_.global_ba_frequency_) == 0) {
      return global_ba_iteration_params;
    } else {
      return local_ba_iteration_params;
    }
  };

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, const util::EmptyStruct &)>
      refresh_residual_checker = checkFactorRefresh;

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)>
      cached_info_creator = dummyCachedInfoCreator;
  std::function<bool(
      const MainFactorInfo &,
      const MainPgPtr &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::function<bool(
          const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      long_term_map_residual_creator_func =
          [&](const MainFactorInfo &factor_info,
              const MainPgPtr &pose_graph,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &residual_params,
              const std::function<bool(const MainFactorInfo &,
                                       const MainPgPtr &,
                                       util::EmptyStruct &)> &cached_inf_create,
              ceres::Problem *problem,
              ceres::ResidualBlockId &res_id,
              util::EmptyStruct &cached_inf) {
            return ltm_factor_creator.createResidual(factor_info,
                                                     pose_graph,
                                                     residual_params,
                                                     cached_inf_create,
                                                     problem,
                                                     res_id,
                                                     cached_inf);
          };
  std::function<bool(
      const MainFactorInfo &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const MainPgPtr &,
      const bool &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator = generateResidualCreator(
          long_term_map_residual_creator_func, cached_info_creator);
  std::function<bool(
      const MainFactorInfo &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const MainPgPtr &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      non_debug_residual_creator =
          [&](const MainFactorInfo &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const MainPgPtr &pose_graph,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return residual_creator(factor_id,
                                    solver_residual_params,
                                    pose_graph,
                                    false,
                                    problem,
                                    residual_id,
                                    cached_info);
          };

  std::function<double(const MainProbData &,
                       const MainPgPtr &,
                       const FrameId &,
                       const FeatureId &,
                       const CameraId &)>
      reprojection_error_provider = [&](const MainProbData &input_problem,
                                        const MainPgPtr &pose_graph,
                                        const FrameId &frame_id,
                                        const FeatureId &feature_id,
                                        const CameraId &camera_id) {
        // Probably need to do something more sophisticated here --
        // ORB-SLAM has a more advanced thing that I haven't looked
        // into
        return config.visual_feature_params_.reprojection_error_std_dev_;
      };
  VisualFeatureFrontend<MainProbData> visual_feature_fronted(
      gba_checker,
      reprojection_error_provider,
      config.visual_feature_params_
          .min_visual_feature_parallax_pixel_requirement_,
      config.visual_feature_params_
          .min_visual_feature_parallax_robot_transl_requirement_,
      config.visual_feature_params_
          .min_visual_feature_parallax_robot_orient_requirement_,
      config.visual_feature_params_.enforce_min_pixel_parallax_requirement_,
      config.visual_feature_params_
          .enforce_min_robot_pose_parallax_requirement_,
      config.visual_feature_params_.inlier_epipolar_err_thresh_,
      config.visual_feature_params_.check_past_n_frames_for_epipolar_err_,
      config.visual_feature_params_.enforce_epipolar_error_requirement_);
  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const FrameId &,
                     const FrameId &)>
      visual_feature_frame_data_adder =
          [&](const MainProbData &input_problem,
              const MainPgPtr &pose_graph,
              const FrameId &min_frame_id,
              const FrameId &max_frame_id_to_opt) {
#ifdef RUN_TIMERS
            CumulativeFunctionTimer::Invocation invoc(
                CumulativeTimerFactory::getInstance()
                    .getOrCreateFunctionTimer(kTimerNameVisualFrontendFunction)
                    .get());
#endif
            visual_feature_fronted.addVisualFeatureObservations(
                input_problem_data,
                pose_graph,
                min_frame_id,
                max_frame_id_to_opt);
          };

  std::function<Covariance<double, 6>(const Pose3D<double> &)>
      pose_deviation_cov_creator = [&](const Pose3D<double> &relative_pose) {
        return generateOdomCov(relative_pose,
                               residual_params.relative_pose_cov_params_
                                   .transl_error_mult_for_transl_error_,
                               residual_params.relative_pose_cov_params_
                                   .transl_error_mult_for_rot_error_,
                               residual_params.relative_pose_cov_params_
                                   .rot_error_mult_for_transl_error_,
                               residual_params.relative_pose_cov_params_
                                   .rot_error_mult_for_rot_error_);
      };

  std::unordered_map<ObjectId, util::EmptyStruct> long_term_map_front_end_data;
  if (long_term_map != nullptr) {
    long_term_map->getFrontEndObjMapData(long_term_map_front_end_data);
  }
  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId,
                                            std::pair<BbCornerPair<double>,
                                                      std::optional<double>>>>>>
      associated_observed_corner_locations =
          std::make_shared<std::unordered_map<
              FrameId,
              std::unordered_map<
                  CameraId,
                  std::unordered_map<ObjectId,
                                     std::pair<BbCornerPair<double>,
                                               std::optional<double>>>>>>();

  std::shared_ptr<std::vector<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId, std::pair<BbCornerPair<double>, double>>>>>
      bounding_boxes_for_pending_object =
          std::make_shared<std::vector<std::unordered_map<
              FrameId,
              std::unordered_map<CameraId,
                                 std::pair<BbCornerPair<double>, double>>>>>();
  std::shared_ptr<std::vector<
      std::pair<std::string, std::optional<EllipsoidState<double>>>>>
      pending_objects = std::make_shared<std::vector<
          std::pair<std::string, std::optional<EllipsoidState<double>>>>>();

  std::function<std::pair<bool, FeatureBasedContextInfo>(
      const FrameId &, const CameraId &, const MainProbData &)>
      bb_context_retriever = [&](const FrameId &frame_id,
                                 const CameraId &camera_id,
                                 const MainProbData &problem_data) {
        FeatureBasedContextInfo context;
        bool success = false;
        if (low_level_features_map.find(frame_id) !=
            low_level_features_map.end()) {
          if (low_level_features_map.at(frame_id).find(camera_id) !=
              low_level_features_map.at(frame_id).end()) {
            context.observed_features_ =
                low_level_features_map.at(frame_id).at(camera_id);
            success = true;
          }
        }
        return std::make_pair(success, context);
      };
  FeatureBasedBoundingBoxFrontEndCreator<ReprojectionErrorFactor>
      feature_based_associator_creator = generateFeatureBasedBbCreator(
          associated_observed_corner_locations,
          all_observed_corner_locations_with_uncertainty,
          bounding_boxes_for_pending_object,
          pending_objects,
          img_heights_and_widths,
          config.bounding_box_covariance_generator_params_,
          config.bounding_box_front_end_params_
              .geometric_similarity_scorer_params_,
          config.bounding_box_front_end_params_
              .feature_based_bb_association_params_,
          long_term_map_front_end_data);
  std::function<std::shared_ptr<AbstractBoundingBoxFrontEnd<
      ReprojectionErrorFactor,
      FeatureBasedFrontEndObjAssociationInfo,
      FeatureBasedFrontEndPendingObjInfo,
      FeatureBasedContextInfo,
      FeatureBasedContextInfo,
      FeatureBasedSingleBbContextInfo,
      util::EmptyStruct>>(const MainPgPtr &, const MainProbData &)>
      bb_associator_retriever =
          [&](const MainPgPtr &pg, const MainProbData &input_prob) {
            return feature_based_associator_creator.getDataAssociator(pg);
          };

  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const FrameId &,
                     const FrameId &)>
      frame_data_adder = [&](const MainProbData &problem_data,
                             const MainPgPtr &pose_graph,
                             const FrameId &min_frame_id,
                             const FrameId &frame_to_add) {
#ifdef RUN_TIMERS
        CumulativeFunctionTimer::Invocation invoc(
            CumulativeTimerFactory::getInstance()
                .getOrCreateFunctionTimer(kTimerNameFrameDataAdderTopLevel)
                .get());
#endif
        addFrameDataAssociatedBoundingBox(problem_data,
                                          pose_graph,
                                          min_frame_id,
                                          frame_to_add,
                                          reprojection_error_provider,
                                          visual_feature_frame_data_adder,
                                          pose_deviation_cov_creator,
                                          bb_retriever,
                                          bb_associator_retriever,
                                          bb_context_retriever);
      };

  CovarianceExtractorParams ltm_covariance_params;

  // TODO maybe replace params with something that will yield more accurate
  // results
  std::function<bool(const FactorType &, const FeatureFactorId &, ObjectId &)>
      long_term_map_obj_retriever = [&](const FactorType &factor_type,
                                        const FeatureFactorId &factor_id,
                                        ObjectId &object_id) {
        std::unordered_set<ObjectId> obj_ids;
        if (!ltm_factor_creator.getObjectIdsForFactor(
                factor_type, factor_id, obj_ids)) {
          return false;
        }
        if (obj_ids.size() != 1) {
          LOG(ERROR) << "This only works with factors that have one object";
          exit(1);
        }
        object_id = *obj_ids.begin();
        return true;
      };

  IndependentEllipsoidsLongTermObjectMapExtractor<
      //      std::unordered_map<ObjectId, RoshanAggregateBbInfo>>
      util::EmptyStruct>
      ltm_extractor(ltm_covariance_params,
                    residual_creator,
                    long_term_map_obj_retriever,
                    config.ltm_tunable_params_,
                    config.ltm_solver_residual_params_,
                    config.ltm_solver_params_);

  std::function<void(
      const MainProbData &,
      const MainPgPtr &,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
      LongTermObjectMapAndResults<MainLtm> &)>
      output_data_extractor = [&](const MainProbData &input_problem_data,
                                  const MainPgPtr &pose_graph,
                                  const pose_graph_optimizer::
                                      OptimizationFactorsEnabledParams
                                          &optimization_factors_enabled_params,
                                  LongTermObjectMapAndResults<MainLtm>
                                      &output_problem_data) {
        if (!output_checkpoints_dir.empty()) {
          // Write pose graph state to file
          outputPoseGraphToFile(pose_graph,
                                file_io::ensureDirectoryPathEndsWithSlash(
                                    output_checkpoints_dir) +
                                    kLtmCheckpointOutputFileBaseName +
                                    file_io::kJsonExtension);
        }

        std::function<bool(std::unordered_map<ObjectId, util::EmptyStruct> &)>
            front_end_map_data_extractor =
                [&](std::unordered_map<ObjectId, util::EmptyStruct>
                        &front_end_data) {
                  feature_based_associator_creator
                      .getDataAssociator(pose_graph)
                      ->getFrontEndObjMapData(front_end_data);
                  return true;
                };

        std::function<bool(
            const MainPgPtr &,
            const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
            MainLtm &)>
            long_term_object_map_extractor =
                [&](const MainPgPtr &ltm_pose_graph,
                    const pose_graph_optimizer::OptimizationFactorsEnabledParams
                        &ltm_optimization_factors_enabled_params,
                    MainLtm &ltm_extractor_out) {
#ifdef RUN_TIMERS
                  CumulativeFunctionTimer::Invocation invoc(
                      vslam_types_refactor::CumulativeTimerFactory::
                          getInstance()
                              .getOrCreateFunctionTimer(
                                  vslam_types_refactor::
                                      kTimerNameLongTermMapExtraction)
                              .get());
#endif
                  return ltm_extractor.extractLongTermObjectMap(
                      ltm_pose_graph,
                      ltm_optimization_factors_enabled_params,
                      front_end_map_data_extractor,
                      jacobian_debug_output_dir,
                      std::nullopt,
                      ltm_extractor_out);
                };
        extractLongTermObjectMapAndResults(pose_graph,
                                           optimization_factors_enabled_params,
                                           long_term_object_map_extractor,
                                           output_problem_data);
      };

  std::function<bool(
      const std::shared_ptr<MainPg> &,
      std::unordered_map<ObjectId, std::unordered_set<ObjectId>> &)>
      merge_decider =
          [&](const std::shared_ptr<MainPg> &pose_graph,
              std::unordered_map<ObjectId, std::unordered_set<ObjectId>>
                  &merge_results) {
            identifyMergeObjectsBasedOnCenterProximity(
                pose_graph,
                config.bounding_box_front_end_params_
                    .post_session_object_merge_params_.max_merge_distance_,
                config.bounding_box_front_end_params_
                    .post_session_object_merge_params_.x_y_only_merge_,
                merge_results);
            return true;
          };

  std::function<bool(const std::shared_ptr<MainPg> &)> object_merger =
      [&](const std::shared_ptr<MainPg> &pose_graph) {
        std::unordered_map<ObjectId, std::unordered_set<ObjectId>>
            merge_results;
        if (!merge_decider(pose_graph, merge_results)) {
          LOG(INFO) << "Merge decider failed";
        }
        if (merge_results.empty()) {
          return false;
        }
        std::shared_ptr<
            AbstractBoundingBoxFrontEnd<ReprojectionErrorFactor,
                                        FeatureBasedFrontEndObjAssociationInfo,
                                        FeatureBasedFrontEndPendingObjInfo,
                                        FeatureBasedContextInfo,
                                        FeatureBasedContextInfo,
                                        FeatureBasedSingleBbContextInfo,
                                        util::EmptyStruct>>
            front_end = bb_associator_retriever(pose_graph, input_problem_data);
        if (!mergeObjects(merge_results, pose_graph, front_end)) {
          LOG(ERROR) << "Error merging objects";
        }
        return true;
      };

  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const MainProbData &,
      const MainPgPtr &,
      const FrameId &,
      const FrameId &)>
      ceres_callback_creator = dummyCeresCallbackCreator;
  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const FrameId &,
                     const FrameId &,
                     const VisualizationTypeEnum &,
                     const int &)>
      bound_visualization_callback =
          std::bind(visualization_callback,
                    img_heights_and_widths,
                    all_observed_corner_locations_with_uncertainty,
                    associated_observed_corner_locations,
                    bounding_boxes_for_pending_object,
                    pending_objects,
                    low_level_features_map,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3,
                    std::placeholders::_4,
                    std::placeholders::_5,
                    std::placeholders::_6);

  OfflineProblemRunner<MainProbData,
                       ReprojectionErrorFactor,
                       LongTermObjectMapAndResults<MainLtm>,
                       util::EmptyStruct,
                       MainPg>
      offline_problem_runner(residual_params,
                             config.limit_traj_eval_params_,
                             config.pgo_solver_params_,
                             continue_opt_checker,
                             window_provider_func,
                             refresh_residual_checker,
                             non_debug_residual_creator,
                             pose_graph_creator,
                             frame_data_adder,
                             output_data_extractor,
                             ceres_callback_creator,
                             bound_visualization_callback,
                             solver_params_provider_func,
                             object_merger,
                             gba_checker);

  bool optimization_result = offline_problem_runner.runOptimization(
      input_problem_data,
      config.optimization_factors_enabled_params_,
      opt_logger,
      output_results,
      start_opt_at_frame,
      run_data_adder_for_first_frame);

  output_results.associated_observed_corner_locations_ =
      associated_observed_corner_locations;
  return optimization_result;
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OPTIMIZATION_RUNNER_H
