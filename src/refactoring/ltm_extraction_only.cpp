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
#include <ros/ros.h>
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
DEFINE_string(input_checkpoints_dir,
              "",
              "Directory to read checkpoints from. If not specified, "
              "optimization should start from the beginning.");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // TODO keep this?
  std::optional<vtr::OptimizationLogger> opt_logger;
  if (FLAGS_logs_directory.empty()) {
    FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  } else {
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = FLAGS_logs_directory;
    opt_logger = vtr::OptimizationLogger(
        file_io::ensureDirectoryPathEndsWithSlash(FLAGS_logs_directory) +
        kCeresOptInfoLogFile);
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

  if (FLAGS_input_checkpoints_dir.empty()) {
    LOG(ERROR) << "No input checkpoints directory provided. Need this to "
                  "extract long-term map";
    exit(1);
  }

  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, "a_" + node_prefix + "ltm_extractor");
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
  vtr::readPoseGraphStateFromFile(
      file_io::ensureDirectoryPathEndsWithSlash(FLAGS_input_checkpoints_dir) +
          vtr::kLtmCheckpointOutputFileBaseName + file_io::kJsonExtension,
      pose_graph_state);

  vtr::IndependentEllipsoidsLongTermObjectMapFactorCreator<
      util::EmptyStruct,
      //      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
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
          [&](const MainFactorInfo &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const MainPgPtr &pose_graph,
              const bool &debug,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return vtr::createResidual(factor_id,
                                       pose_graph,
                                       solver_residual_params,
                                       cached_info_creator,
                                       long_term_map_residual_creator_func,
                                       problem,
                                       residual_id,
                                       cached_info,
                                       debug);
          };

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

  std::function<void(
      const MainPgPtr &,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
      vtr::LongTermObjectMapAndResults<MainLtm> &)>
      output_data_extractor =
          [&](const MainPgPtr &pose_graph,
              const pose_graph_optimizer::OptimizationFactorsEnabledParams
                  &optimization_factors_enabled_params,
              vtr::LongTermObjectMapAndResults<MainLtm> &output_problem_data) {
            std::function<bool(
                std::unordered_map<vtr::ObjectId, util::EmptyStruct> &)>
                front_end_map_data_extractor =
                    [&](std::unordered_map<vtr::ObjectId, util::EmptyStruct>
                            &front_end_data) {
                      for (const auto &obj_entry :
                           pose_graph_state.obj_only_pose_graph_state_
                               .ellipsoid_estimates_) {
                        front_end_data[obj_entry.first] = util::EmptyStruct();
                      }
                      return true;
                    };

            std::function<bool(
                const MainPgPtr &,
                const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
                MainLtm &)>
                long_term_object_map_extractor =
                    [&](const MainPgPtr &ltm_pose_graph,
                        const pose_graph_optimizer::
                            OptimizationFactorsEnabledParams
                                &ltm_optimization_factors_enabled_params,
                        MainLtm &ltm_extractor_out) {
                      return ltm_extractor.extractLongTermObjectMap(
                          ltm_pose_graph,
                          ltm_optimization_factors_enabled_params,
                          front_end_map_data_extractor,
                          FLAGS_ltm_opt_jacobian_info_directory,
                          ltm_extractor_out);
                    };
            vtr::extractLongTermObjectMapAndResults(
                pose_graph,
                optimization_factors_enabled_params,
                long_term_object_map_extractor,
                output_problem_data);
          };

  vtr::LongTermObjectMapAndResults<MainLtm> output_results;
  output_data_extractor(
      pose_graph, config.optimization_factors_enabled_params_, output_results);

  cv::FileStorage ltm_out_fs(FLAGS_long_term_map_output,
                             cv::FileStorage::WRITE);
  ltm_out_fs
      << "long_term_map"
      << vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
             util::EmptyStruct,
             vtr::SerializableEmptyStruct>(output_results.long_term_map_);
  ltm_out_fs.release();

  return 0;
}