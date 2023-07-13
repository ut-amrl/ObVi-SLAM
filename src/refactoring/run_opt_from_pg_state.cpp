#include <base_lib/basic_utils.h>
#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <file_io/cv_file_storage/long_term_object_map_file_storage_io.h>
#include <file_io/cv_file_storage/object_and_reprojection_feature_pose_graph_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/configuration/full_ov_slam_config.h>
#include <refactoring/long_term_map/long_term_map_factor_creator.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/optimization/residual_creator.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/visualization/ros_visualization.h>
#include <ros/ros.h>
#include <run_optimization_utils/run_opt_utils.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

const std::string kCeresOptInfoLogFile = "ceres_opt_summary.csv";

typedef vtr::IndependentEllipsoidsLongTermObjectMap<
    //    std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
    util::EmptyStruct>
    MainLtm;
typedef std::shared_ptr<MainLtm> MainLtmPtr;
typedef vtr::UnassociatedBoundingBoxOfflineProblemData<
    vtr::StructuredVisionFeatureTrack,
    sensor_msgs::Image::ConstPtr,
    MainLtm>
    MainProbData;
typedef vtr::ObjectAndReprojectionFeaturePoseGraph MainPg;
typedef std::shared_ptr<MainPg> MainPgPtr;
typedef std::pair<vslam_types_refactor::FactorType,
                  vslam_types_refactor::FeatureFactorId>
    MainFactorInfo;

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(
    long_term_map_input,
    "",
    "File name that stores the long-term map to load. If empty, will start "
    "from scratch.");
DEFINE_string(long_term_map_output,
              "",
              "File name to output the long-term map to.");
DEFINE_string(ltm_opt_jacobian_info_directory,
              "",
              "Directory to write jacobian info from the LTM optimization for");
DEFINE_string(params_config_file, "", "config file containing tunable params");
DEFINE_string(logs_directory,
              "",
              "If specified, where logs are written (in addition to stderr)");
DEFINE_string(input_checkpoint_file, "", "File to read state from");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // TODO keep this?
  if (FLAGS_logs_directory.empty()) {
    FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  } else {
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = FLAGS_logs_directory;
  }
  FLAGS_colorlogtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  if (FLAGS_long_term_map_output.empty()) {
    LOG(ERROR) << "No long-term map output file provided";
    exit(1);
  }

  if (FLAGS_input_checkpoint_file.empty()) {
    LOG(ERROR) << "No input checkpoint file provided. Need this to "
                  "extract long-term map";
    exit(1);
  }

  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, "a_" + node_prefix + "run_opt_from_state");
  ros::NodeHandle node_handle;

  vtr::FullOVSLAMConfig config;
  vtr::readConfiguration(FLAGS_params_config_file, config);

  if ((!config.optimization_factors_enabled_params_
            .use_visual_features_on_global_ba_) &&
      (!config.optimization_factors_enabled_params_
            .use_pose_graph_on_global_ba_)) {
    LOG(ERROR) << "Must have either visual features or pose graph (or both) "
                  "for global ba; review/fix your config";
    exit(1);
  }
  if ((!config.optimization_factors_enabled_params_
            .use_visual_features_on_final_global_ba_) &&
      (!config.optimization_factors_enabled_params_
            .use_pose_graph_on_final_global_ba_)) {
    LOG(ERROR) << "Must have either visual features or pose graph (or both) "
                  "for final global ba; review/fix your config";
    exit(1);
  }

  // Read necessary data in from file
  // -----------------------------------------
  MainLtmPtr long_term_map;
  if (!FLAGS_long_term_map_input.empty()) {
    cv::FileStorage ltm_in_fs(FLAGS_long_term_map_input, cv::FileStorage::READ);
    vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
        //        std::unordered_map<vtr::ObjectId,
        //        vtr::RoshanAggregateBbInfo>,
        //        vtr::SerializableMap<vtr::ObjectId,
        //                             vtr::SerializableObjectId,
        //                             vtr::RoshanAggregateBbInfo,
        //                             vtr::SerializableRoshanAggregateBbInfo>>
        util::EmptyStruct,
        vtr::SerializableEmptyStruct>
        serializable_ltm;
    ltm_in_fs["long_term_map"] >> serializable_ltm;
    ltm_in_fs.release();
    MainLtm ltm_from_serializable = serializable_ltm.getEntry();
    vtr::EllipsoidResults ellipsoid_results_ltm;
    ltm_from_serializable.getLtmEllipsoidResults(ellipsoid_results_ltm);
    LOG(INFO) << "Long term map size "
              << ellipsoid_results_ltm.ellipsoids_.size();
    long_term_map = std::make_shared<MainLtm>(ltm_from_serializable);
  }

  vtr::ObjectAndReprojectionFeaturePoseGraphState pose_graph_state;
  vtr::readPoseGraphStateFromFile(FLAGS_input_checkpoint_file,
                                  pose_graph_state);

  vtr::IndependentEllipsoidsLongTermObjectMapFactorCreator<util::EmptyStruct,
                                                           util::EmptyStruct>
      ltm_factor_creator(long_term_map);

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)>
      cached_info_creator = [](const MainFactorInfo &factor_info,
                               const MainPgPtr &pose_graph,
                               util::EmptyStruct &cached_info) {
        return true;  // TODO maybe fill in with real info some day
      };
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
      residual_creator =
          vtr::generateResidualCreator(long_term_map_residual_creator_func);

  std::function<bool(
      const std::unordered_set<vtr::ObjectId> &,
      util::BoostHashMap<MainFactorInfo, std::unordered_set<vtr::ObjectId>> &)>
      long_term_map_factor_provider =
          [&](const std::unordered_set<vtr::ObjectId> &objects_to_include,
              util::BoostHashMap<MainFactorInfo,
                                 std::unordered_set<vtr::ObjectId>>
                  &factor_data) {
            return ltm_factor_creator.getFactorsToInclude(objects_to_include,
                                                          factor_data);
          };

  MainPgPtr pose_graph =
      MainPg::createObjectAndReprojectionFeaturePoseGraphFromState(
          pose_graph_state, long_term_map_factor_provider);

  vtr::CovarianceExtractorParams ltm_covariance_params;

  // TODO maybe replace params with something that will yield more accurate
  // results
  std::function<bool(
      const vtr::FactorType &, const vtr::FeatureFactorId &, vtr::ObjectId &)>
      long_term_map_obj_retriever = [&](const vtr::FactorType &factor_type,
                                        const vtr::FeatureFactorId &factor_id,
                                        vtr::ObjectId &object_id) {
        std::unordered_set<vtr::ObjectId> obj_ids;
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

  vtr::IndependentEllipsoidsLongTermObjectMapExtractor<
      //      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
      util::EmptyStruct>
      ltm_extractor(ltm_covariance_params,
                    residual_creator,
                    long_term_map_obj_retriever,
                    config.ltm_tunable_params_,
                    config.ltm_solver_residual_params_,
                    config.ltm_solver_params_);

  std::function<bool(
      const std::shared_ptr<MainPg> &,
      std::unordered_map<vtr::ObjectId, std::unordered_set<vtr::ObjectId>> &)>
      merge_decider = [&](const std::shared_ptr<MainPg> &pose_graph,
                          std::unordered_map<vtr::ObjectId,
                                             std::unordered_set<vtr::ObjectId>>
                              &merge_results) {
        vtr::identifyMergeObjectsBasedOnCenterProximity(
            pose_graph,
            config.bounding_box_front_end_params_
                .post_session_object_merge_params_.max_merge_distance_,
            merge_results);
        return true;
      };

  std::function<bool(const std::shared_ptr<MainPg> &)> object_merger =
      [&](const std::shared_ptr<MainPg> &pose_graph) {
        std::unordered_map<vtr::ObjectId, std::unordered_set<vtr::ObjectId>>
            merge_results;
        if (!merge_decider(pose_graph, merge_results)) {
          LOG(INFO) << "Merge decider failed";
        }
        if (merge_results.empty()) {
          return false;
        }
        std::shared_ptr<vtr::AbstractBoundingBoxFrontEnd<
            vtr::ReprojectionErrorFactor,
            vtr::FeatureBasedFrontEndObjAssociationInfo,
            vtr::FeatureBasedFrontEndPendingObjInfo,
            vtr::FeatureBasedContextInfo,
            vtr::FeatureBasedContextInfo,
            vtr::FeatureBasedSingleBbContextInfo,
            util::EmptyStruct>>
            front_end = bb_associator_retriever(pose_graph, input_problem_data);
        if (!mergeObjects(merge_results, pose_graph, front_end)) {
          LOG(ERROR) << "Error merging objects";
        }
        return true;
      };

  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const vtr::FrameId &,
                     const vtr::FrameId &,
                     const vtr::VisualizationTypeEnum &,
                     const int &)>
      visualization_callback = [&](const MainProbData &input_problem_data,
                                   const MainPgPtr &pose_graph,
                                   const vtr::FrameId &min_frame_id,
                                   const vtr::FrameId &max_frame_id_to_opt,
                                   const vtr::VisualizationTypeEnum
                                       &visualization_type,
                                   const int &attempt_num) {
        std::shared_ptr<
            vtr::ObjAndLowLevelFeaturePoseGraph<vtr::ReprojectionErrorFactor>>
            superclass_ptr = pose_graph;
        visualizationStub(
            vis_manager,
            save_to_file_visualizer,
            camera_extrinsics_by_camera,
            camera_intrinsics_by_camera,
            img_heights_and_widths,
            images,
            all_observed_corner_locations_with_uncertainty,
            associated_observed_corner_locations,
            bounding_boxes_for_pending_object,
            pending_objects,
            low_level_features_map,
            initial_feat_positions,
            input_problem_data,
            superclass_ptr,
            min_frame_id,
            max_frame_id_to_opt,
            visualization_type,
            config.bounding_box_covariance_generator_params_
                .near_edge_threshold_,
            config.bounding_box_front_end_params_
                .feature_based_bb_association_params_.min_observations_,
            gt_trajectory,
            effective_max_frame_id,
            FLAGS_output_checkpoints_dir,
            attempt_num);
      };
  if ((!config.optimization_factors_enabled_params_
            .use_visual_features_on_global_ba_) &&
      (!config.optimization_factors_enabled_params_
            .use_pose_graph_on_global_ba_)) {
    LOG(ERROR) << "Must have either visual features or pose graph (or both) "
                  "for global ba; review/fix your config";
    exit(1);
  }
  if ((!config.optimization_factors_enabled_params_
            .use_visual_features_on_final_global_ba_) &&
      (!config.optimization_factors_enabled_params_
            .use_pose_graph_on_final_global_ba_)) {
    LOG(ERROR) << "Must have either visual features or pose graph (or both) "
                  "for final global ba; review/fix your config";
    exit(1);
  }

  std::optional<vtr::OptimizationLogger> opt_logger;

  std::function<bool(const vtr::FrameId &)> gba_checker =
      [&](const vtr::FrameId &max_frame_to_opt) -> bool {
    vtr::FrameId min_frame_to_opt = window_provider_func(max_frame_to_opt);
    if (max_frame_to_opt - min_frame_to_opt <=
        config.sliding_window_params_.local_ba_window_size_) {
      return false;
    }
    return true;
  };

  vtr::FrameId last_added_pose;  // TODO

  std::function<pose_graph_optimization::OptimizationIterationParams(
      const vtr::FrameId &)>
      solver_params_provider_func = [&](const vtr::FrameId &max_frame_to_opt)
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

  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const vtr::FrameId &,
                     const vtr::FrameId &)>
      frame_data_adder = [&](const MainProbData &problem_data,
                             const MainPgPtr &pose_graph,
                             const vtr::FrameId &min_frame_id,
                             const vtr::FrameId &frame_to_add) {
#ifdef RUN_TIMERS
        CumulativeFunctionTimer::Invocation invoc(
            vtr::CumulativeTimerFactory::getInstance()
                .getOrCreateFunctionTimer(vtr::kTimerNameFrameDataAdderTopLevel)
                .get());
#endif
        LOG(ERROR) << "Shouldn't be adding data in this function";
        //    vtr::addFrameDataAssociatedBoundingBox(problem_data,
        //                                           pose_graph,
        //                                           min_frame_id,
        //                                           frame_to_add,
        //                                           reprojection_error_provider,
        //                                           visual_feature_frame_data_adder,
        //                                           pose_deviation_cov_creator,
        //                                           bb_retriever,
        //                                           bb_associator_retriever,
        //                                           bb_context_retriever);
      };

  std::function<void(const MainProbData &, MainPgPtr &)> pose_graph_creator =
      [&](const MainProbData &, MainPgPtr &pose_graph_ptr) {
        pose_graph_ptr = pose_graph;
      };

  vtr::OfflineProblemRunner<MainProbData,
                            vtr::ReprojectionErrorFactor,
                            vtr::LongTermObjectMapAndResults<MainLtm>,
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
                             visualization_callback,
                             solver_params_provider_func,
                             object_merger,
                             gba_checker);

  //  vtr::SpatialEstimateOnlyResults output_results;
  vtr::LongTermObjectMapAndResults<MainLtm> output_results;
  offline_problem_runner.runOptimization(
      input_problem_data,
      config.optimization_factors_enabled_params_,
      opt_logger,
      output_results,
      last_added_pose,
      false);

  cv::FileStorage ltm_out_fs(FLAGS_long_term_map_output,
                             cv::FileStorage::WRITE);
  ltm_out_fs
      << "long_term_map"
      << vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
             //             std::unordered_map<vtr::ObjectId,
             //             vtr::RoshanAggregateBbInfo>,
             //             vtr::SerializableMap<vtr::ObjectId,
             //                                  vtr::SerializableObjectId,
             //                                  vtr::RoshanAggregateBbInfo,
             //                                  vtr::SerializableRoshanAggregateBbInfo>>(
             util::EmptyStruct,
             vtr::SerializableEmptyStruct>(output_results.long_term_map_);
  ltm_out_fs.release();
  //  LOG(INFO) << "Num ellipsoids "
  //            << output_results.ellipsoid_results_.ellipsoids_.size();

  // TODO fix long term map
  // Output robot poses
  // Output ellipsoids

  // TODO
  // Load in map state
  // Load in config
  // Run optimization then post processing

  return 0;
}