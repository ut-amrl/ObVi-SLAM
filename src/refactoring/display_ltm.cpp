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
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;


typedef vtr::IndependentEllipsoidsLongTermObjectMap<
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

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // TODO keep this?
  std::optional<vtr::OptimizationLogger> opt_logger;
  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  FLAGS_colorlogtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, "a_" + node_prefix + "ltm_displayer");
  ros::NodeHandle node_handle;

  // Read necessary data in from file
  // -----------------------------------------
  MainLtmPtr long_term_map;

  cv::FileStorage ltm_in_fs(FLAGS_long_term_map_input, cv::FileStorage::READ);
  vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
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

  std::shared_ptr<vtr::RosVisualization> vis_manager =
      std::make_shared<vtr::RosVisualization>(node_handle, param_prefix, node_prefix);

  std::unordered_map<
      vtr::ObjectId,
      std::pair<std::string,
                std::pair<vtr::EllipsoidState<double>,
                          vtr::Covariance<double,
                                          vtr::kEllipsoidParamterizationSize>>>>
      ltm_ellipsoids;
  vtr::EllipsoidResults ellipsoids_in_map;
  long_term_map->getEllipsoidResults(ellipsoids_in_map);

  std::unordered_map<
      vtr::ObjectId,
      vtr::Covariance<double, vtr::kEllipsoidParamterizationSize>>
      ellipsoid_covariances = long_term_map->getEllipsoidCovariances();
  for (const auto &ltm_ellipsoid : ellipsoids_in_map.ellipsoids_) {
    ltm_ellipsoids[ltm_ellipsoid.first] = std::make_pair(
        ltm_ellipsoid.second.first,
        std::make_pair(ltm_ellipsoid.second.second,
                       ellipsoid_covariances.at(ltm_ellipsoid.first)));
  }

  vis_manager->publishLongTermMap(ltm_ellipsoids);
  ros::Duration(5).sleep();
  vis_manager->publishLongTermMap(ltm_ellipsoids);
  ros::Duration(1).sleep();
  return 0;
}