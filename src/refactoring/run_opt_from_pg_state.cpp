#include <base_lib/basic_utils.h>
#include <file_io/camera_info_io_utils.h>
#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <file_io/cv_file_storage/long_term_object_map_file_storage_io.h>
#include <file_io/cv_file_storage/object_and_reprojection_feature_pose_graph_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/bounding_box_retriever.h>
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
#include <run_optimization_utils/optimization_runner.h>
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
DEFINE_string(intrinsics_file, "", "File with camera intrinsics");
DEFINE_string(extrinsics_file, "", "File with camera extrinsics");
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

  vtr::FrameId last_frame_id = pose_graph->getMinMaxFrameId().second;

  std::function<void(const MainProbData &, MainPgPtr &)> pose_graph_creator =
      [&](const MainProbData &, MainPgPtr &created_pose_graph) {
        created_pose_graph = pose_graph;
      };

  std::optional<vtr::OptimizationLogger> opt_logger;

  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera =
          file_io::readCameraIntrinsicsByCameraFromFile(FLAGS_intrinsics_file);
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      camera_extrinsics_by_camera =
          file_io::readCameraExtrinsicsByCameraFromFile(FLAGS_extrinsics_file);

  vtr::YoloBoundingBoxQuerier bb_querier(node_handle);
  std::function<bool(
      const vtr::FrameId &,
      const MainProbData &input_prob_data,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>> &)>
      bb_retriever = [&](const vtr::FrameId &frame_id_to_query_for,
                         const MainProbData &input_prob_data,
                         std::unordered_map<vtr::CameraId,
                                            std::vector<vtr::RawBoundingBox>>
                             &bounding_boxes_by_cam) {
#ifdef RUN_TIMERS
        CumulativeFunctionTimer::Invocation invoc(
            vtr::CumulativeTimerFactory::getInstance()
                .getOrCreateFunctionTimer(vtr::kTimerNameBbQuerier)
                .get());
#endif
        return bb_querier.retrieveBoundingBoxes(
            frame_id_to_query_for, input_prob_data, bounding_boxes_by_cam);
      };

  std::function<void(
      const std::unordered_map<vtr::CameraId, std::pair<double, double>> &,
      const std::shared_ptr<std::unordered_map<
          vtr::FrameId,
          std::unordered_map<vtr::CameraId,
                             std::vector<std::pair<vtr::BbCornerPair<double>,
                                                   std::optional<double>>>>>> &,
      const std::shared_ptr<std::unordered_map<
          vtr::FrameId,
          std::unordered_map<
              vtr::CameraId,
              std::unordered_map<vtr::ObjectId,
                                 std::pair<vtr::BbCornerPair<double>,
                                           std::optional<double>>>>>> &,
      const std::shared_ptr<std::vector<std::unordered_map<
          vtr::FrameId,
          std::unordered_map<vtr::CameraId,
                             std::pair<vtr::BbCornerPair<double>, double>>>>> &,
      const std::shared_ptr<std::vector<
          std::pair<std::string, std::optional<vtr::EllipsoidState<double>>>>>
          &,
      const std::unordered_map<
          vtr::FrameId,
          std::unordered_map<
              vtr::CameraId,
              std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>> &,
      const MainProbData &,
      const MainPgPtr &,
      const vtr::FrameId &,
      const vtr::FrameId &,
      const vtr::VisualizationTypeEnum &,
      const int &)>
      visualization_callback =
          [&](const std::unordered_map<vtr::CameraId, std::pair<double, double>>
                  &img_heights_and_widths,
              const std::shared_ptr<std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::vector<std::pair<vtr::BbCornerPair<double>,
                                            std::optional<double>>>>>>
                  &all_observed_corner_locations_with_uncertainty,
              const std::shared_ptr<std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::unordered_map<vtr::ObjectId,
                                         std::pair<vtr::BbCornerPair<double>,
                                                   std::optional<double>>>>>>
                  &associated_observed_corner_locations,
              const std::shared_ptr<std::vector<std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::pair<vtr::BbCornerPair<double>, double>>>>>
                  &bounding_boxes_for_pending_object,
              const std::shared_ptr<std::vector<
                  std::pair<std::string,
                            std::optional<vtr::EllipsoidState<double>>>>>
                  &pending_objects,
              const std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::unordered_map<vtr::FeatureId,
                                         vtr::PixelCoord<double>>>>
                  &low_level_features_map,
              const MainProbData &input_problem_data,
              const MainPgPtr &pose_graph,
              const vtr::FrameId &min_frame_id,
              const vtr::FrameId &max_frame_id_to_opt,
              const vtr::VisualizationTypeEnum &visualization_type,
              const int &attempt_num) {

          };

  vtr::LongTermObjectMapAndResults<MainLtm> output_results;
  if (!runFullOptimization(opt_logger,
                           config,
                           camera_intrinsics_by_camera,
                           camera_extrinsics_by_camera,
                           {},  // bounding_boxes,
                           {},  // visual_features,
                           {},  // robot_poses,
                           long_term_map,
                           pose_graph_creator,
                           {},  // images,
                           {},  // FLAGS_output_checkpoints_dir,
                           FLAGS_ltm_opt_jacobian_info_directory,
                           bb_retriever,
                           visualization_callback,
                           ltm_factor_creator,
                           output_results,
                           last_frame_id,
                           false)) {
    LOG(ERROR) << "Optimization failed";
  }

  cv::FileStorage ltm_out_fs(FLAGS_long_term_map_output,
                             cv::FileStorage::WRITE);
  ltm_out_fs << "long_term_map"
             << vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
                    util::EmptyStruct,
                    vtr::SerializableEmptyStruct>(
                    output_results.long_term_map_);
  ltm_out_fs.release();
  LOG(INFO) << "Num ellipsoids "
            << output_results.ellipsoid_results_.ellipsoids_.size();

  return 0;
}