#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/cv_file_storage/long_term_object_map_file_storage_io.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/roshan_bounding_box_front_end_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/roshan_bounding_box_front_end.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/long_term_map/long_term_map_factor_creator.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/offline/pose_graph_frame_data_adder.h>
#include <refactoring/optimization/residual_creator.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/visualization/ros_visualization.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

typedef vtr::IndependentEllipsoidsLongTermObjectMap<
    std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
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
DEFINE_string(bounding_boxes_by_node_id_file,
              "",
              "File with bounding box observations by node id");
DEFINE_string(poses_by_node_id_file,
              "",
              "File with initial robot pose estimates");
DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(
    long_term_map_input,
    "",
    "File name that stores the long-term map to load. If empty, will start "
    "from scratch.");
DEFINE_string(long_term_map_output,
              "",
              "File name to output the long-term map to.");
DEFINE_double(min_confidence, 0.2, "Minimum confidence");

std::string kCompressedImageSuffix = "compressed";

std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
readCameraIntrinsicsByCameraFromFile(const std::string &file_name) {
  std::vector<file_io::CameraIntrinsicsWithId> camera_intrinsics_by_cam_id;
  file_io::readCameraIntrinsicsWithIdsFromFile(file_name,
                                               camera_intrinsics_by_cam_id);
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      cam_intrinsics_map;
  for (const file_io::CameraIntrinsicsWithId &intrinsics_for_cam :
       camera_intrinsics_by_cam_id) {
    vtr::CameraIntrinsicsMat<double> intrinsics_mat;
    intrinsics_mat << intrinsics_for_cam.mat_00, intrinsics_for_cam.mat_01,
        intrinsics_for_cam.mat_02, intrinsics_for_cam.mat_10,
        intrinsics_for_cam.mat_11, intrinsics_for_cam.mat_12,
        intrinsics_for_cam.mat_20, intrinsics_for_cam.mat_21,
        intrinsics_for_cam.mat_22;
    cam_intrinsics_map[intrinsics_for_cam.camera_id] = intrinsics_mat;
    // TODO do we need to include the width/height anywhere?
  }
  return cam_intrinsics_map;
}

std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
readCameraExtrinsicsByCameraFromFile(const std::string &file_name) {
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      extrinsics_map;

  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  // TODO Verify that this is correct (and eventually just update the file to
  // have this)
  Eigen::Quaterniond extrinsics_orientation_switch_to_cam =
      Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5)
          .inverse();  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  std::vector<file_io::CameraExtrinsicsWithId> camera_extrinsics_by_cam_id;
  file_io::readCameraExtrinsicsWithIdsFromFile(file_name,
                                               camera_extrinsics_by_cam_id);
  for (const file_io::CameraExtrinsicsWithId &extrinsics_for_cam :
       camera_extrinsics_by_cam_id) {
    vtr::Position3d<double> extrinsics_pos(extrinsics_for_cam.transl_x,
                                           extrinsics_for_cam.transl_y,
                                           extrinsics_for_cam.transl_z);

    vtr::Orientation3D<double> extrinsics_orient(
        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
                           extrinsics_for_cam.quat_x,
                           extrinsics_for_cam.quat_y,
                           extrinsics_for_cam.quat_z) *
        extrinsics_orientation_switch_to_cam);
    vtr::CameraExtrinsics<double> extrinsics_obj(extrinsics_pos,
                                                 extrinsics_orient);

    extrinsics_map[extrinsics_for_cam.camera_id] = extrinsics_obj;
  }
  return extrinsics_map;
}

std::unordered_map<
    vtr::FrameId,
    std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
readBoundingBoxesFromFile(const std::string &file_name) {
  std::vector<file_io::BoundingBoxWithNodeId> bounding_boxes_by_node_id;
  file_io::readBoundingBoxesWithNodeIdFromFile(file_name,
                                               bounding_boxes_by_node_id);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bb_map;
  for (const file_io::BoundingBoxWithNodeId &raw_bb :
       bounding_boxes_by_node_id) {
    if (raw_bb.detection_confidence >= FLAGS_min_confidence) {
      vtr::RawBoundingBox bb;
      bb.pixel_corner_locations_ = std::make_pair(
          vtr::PixelCoord<double>(raw_bb.min_pixel_x, raw_bb.min_pixel_y),
          vtr::PixelCoord<double>(raw_bb.max_pixel_x, raw_bb.max_pixel_y));
      bb.semantic_class_ = raw_bb.semantic_class;
      bb.detection_confidence_ = raw_bb.detection_confidence;
      bb_map[raw_bb.node_id][raw_bb.camera_id].emplace_back(bb);
    }
  }
  return bb_map;
}

std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> readRobotPosesFromFile(
    const std::string &file_name) {
  std::vector<std::pair<uint64_t, pose::Pose3d>> robot_poses_by_node_id;
  file_io::readPose3dsAndNodeIdFromFile(file_name, robot_poses_by_node_id);
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses_by_node_num;
  for (const std::pair<uint64, pose::Pose3d> &pose_3d_with_frame :
       robot_poses_by_node_id) {
    vtr::Pose3D<double> pose(
        pose_3d_with_frame.second.first,
        vtr::Orientation3D<double>(pose_3d_with_frame.second.second));
    robot_poses_by_node_num[pose_3d_with_frame.first] = pose;
  }
  return robot_poses_by_node_num;
}

std::unordered_map<
    vtr::FrameId,
    std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
getImagesFromRosbag(const std::string &rosbag_file_name,
                    const std::string &nodes_by_timestamp_file,
                    const std::unordered_map<std::string, vtr::CameraId>
                        &camera_topic_to_camera_id) {
  std::vector<file_io::NodeIdAndTimestamp> nodes_by_timestamps_vec;
  util::BoostHashMap<pose::Timestamp, vtr::FrameId> nodes_for_timestamps_map;
  file_io::readNodeIdsAndTimestampsFromFile(FLAGS_nodes_by_timestamp_file,
                                            nodes_by_timestamps_vec);

  for (const file_io::NodeIdAndTimestamp &raw_node_id_and_timestamp :
       nodes_by_timestamps_vec) {
    nodes_for_timestamps_map[std::make_pair(
        raw_node_id_and_timestamp.seconds_,
        raw_node_id_and_timestamp.nano_seconds_)] =
        raw_node_id_and_timestamp.node_id_;
  }

  // Read the images
  rosbag::Bag bag;
  bag.open(FLAGS_rosbag_file, rosbag::bagmode::Read);
  // TODO do we want to make a new back with uncompressed images or handle the
  // compression here?

  std::vector<std::string> topics;
  for (const auto &camera_topic_and_id : camera_topic_to_camera_id) {
    topics.emplace_back(camera_topic_and_id.first);
    //    LOG(INFO) << "Checking topic " << camera_topic_and_id.first;
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images_by_frame_and_cam;
  for (const rosbag::MessageInstance &m : view) {
    sensor_msgs::Image::ConstPtr msg;
    if (m.getTopic().find(kCompressedImageSuffix) != std::string::npos) {
      sensor_msgs::CompressedImage::ConstPtr compressed_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      image_utils::decompressImage(compressed_msg, msg);
    } else {
      msg = m.instantiate<sensor_msgs::Image>();
    }
    //    LOG(INFO) << "Checking image message";

    pose::Timestamp img_timestamp =
        std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);
    if (nodes_for_timestamps_map.find(img_timestamp) !=
        nodes_for_timestamps_map.end()) {
      //      LOG(INFO) << "Found image for timestamp ";
      vtr::CameraId cam = camera_topic_to_camera_id.at(m.getTopic());
      vtr::FrameId frame_id = nodes_for_timestamps_map[img_timestamp];
      images_by_frame_and_cam[frame_id][cam] = msg;
    } else {
      //      LOG(INFO) << "No image for timestamp";
    }
  }
  return images_by_frame_and_cam;
}

std::vector<std::shared_ptr<ceres::IterationCallback>>
dummyCeresCallbackCreator(const MainProbData &input_problem_data,
                          const MainPgPtr &pose_graph,
                          const vtr::FrameId &min_frame_optimized,
                          const vtr::FrameId &max_frame_optimized) {
  // TODO replace with actual code later
  return {};
}

bool checkFactorRefresh(const MainFactorInfo &factor,
                        const MainPgPtr &,
                        const util::EmptyStruct &) {
  return false;
}

void createPoseGraph(
    const MainProbData &input_problem_data,
    const std::function<bool(util::BoostHashSet<MainFactorInfo> &)>
        &long_term_map_factor_provider,
    MainPgPtr &pose_graph) {
  std::unordered_map<vtr::ObjectId,
                     std::pair<std::string, vtr::RawEllipsoid<double>>>
      ltm_objects;
  vtr::EllipsoidResults ellipsoids_in_map;
  if (input_problem_data.getLongTermObjectMap() != nullptr) {
    input_problem_data.getLongTermObjectMap()->getEllipsoidResults(
        ellipsoids_in_map);
  }
  LOG(INFO) << "Ellipsoids size " << ellipsoids_in_map.ellipsoids_.size();
  for (const auto &ellipsoid_entry : ellipsoids_in_map.ellipsoids_) {
    ltm_objects[ellipsoid_entry.first] = std::make_pair(
        ellipsoid_entry.second.first,
        vtr::convertToRawEllipsoid(ellipsoid_entry.second.second));
  }
  LOG(INFO) << "Ltm objects size " << ltm_objects.size();
  LOG(INFO) << "Creating pose graph ";
  pose_graph =
      std::make_shared<MainPg>(input_problem_data.getObjDimMeanAndCovByClass(),
                               input_problem_data.getCameraExtrinsicsByCamera(),
                               input_problem_data.getCameraIntrinsicsByCamera(),
                               ltm_objects,
                               long_term_map_factor_provider);
}

void visualizationStub(
    const std::shared_ptr<vtr::RosVisualization> &vis_manager,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &extrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &intrinsics,
    const std::unordered_map<vtr::CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
        &images,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId,
                           std::vector<std::pair<vtr::BbCornerPair<double>,
                                                 std::optional<double>>>>>
        &all_observed_corner_locations_with_uncertainty,
    const std::shared_ptr<std::unordered_map<
        vtr::FrameId,
        std::unordered_map<
            vtr::CameraId,
            std::unordered_map<
                vtr::ObjectId,
                std::pair<vtr::BbCornerPair<double>, std::optional<double>>>>>>
        &observed_corner_locations,
    const MainProbData &input_problem_data,
    std::shared_ptr<
        vtr::ObjAndLowLevelFeaturePoseGraph<vtr::ReprojectionErrorFactor>>
        &pose_graph,
    const vtr::FrameId &min_frame_optimized,
    const vtr::FrameId &max_frame_optimized,
    const vtr::VisualizationTypeEnum &visualization_stage) {
  switch (visualization_stage) {
    case vtr::BEFORE_ANY_OPTIMIZATION:
      vis_manager->publishTransformsForEachCamera(
          input_problem_data.getMaxFrameId(),
          input_problem_data.getRobotPoseEstimates(),
          input_problem_data.getCameraExtrinsicsByCamera());

      sleep(3);
      break;
    case vtr::BEFORE_EACH_OPTIMIZATION:
      break;
    case vtr::AFTER_EACH_OPTIMIZATION: {
      std::unordered_map<vtr::FrameId, vtr::RawPose3d<double>>
          optimized_robot_pose_estimates;
      std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>
          initial_robot_pose_estimates =
              input_problem_data.getRobotPoseEstimates();
      pose_graph->getRobotPoseEstimates(optimized_robot_pose_estimates);
      std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>
          optimized_trajectory;
      for (const auto &frame_raw_pose : optimized_robot_pose_estimates) {
        optimized_trajectory[frame_raw_pose.first] =
            vtr::convertToPose3D(frame_raw_pose.second);
      }

      std::unordered_map<vtr::ObjectId,
                         std::pair<std::string, vtr::RawEllipsoid<double>>>
          object_estimates;
      pose_graph->getObjectEstimates(object_estimates);
      std::unordered_map<vtr::ObjectId, vtr::EllipsoidState<double>>
          optimized_ellipsoid_estimates;
      for (const auto &obj_and_raw_est : object_estimates) {
        optimized_ellipsoid_estimates[obj_and_raw_est.first] =
            vtr::convertToEllipsoidState(obj_and_raw_est.second.second);
      }
      std_msgs::ColorRGBA optimized_ellipsoid_color;
      optimized_ellipsoid_color.a = 0.5;
      //      optimized_ellipsoid_color.r = 1.0;
      optimized_ellipsoid_color.g = 1;
      vis_manager->visualizeEllipsoids(optimized_ellipsoid_estimates,
                                       vtr::ESTIMATED);
      if (input_problem_data.getLongTermObjectMap() != nullptr) {
        vtr::EllipsoidResults ellipsoids_in_map;
        input_problem_data.getLongTermObjectMap()->getEllipsoidResults(
            ellipsoids_in_map);
        std::unordered_map<vtr::ObjectId, vtr::EllipsoidState<double>>
            ltm_ellipsoids;
        for (const auto &ltm_ellipsoid : ellipsoids_in_map.ellipsoids_) {
          ltm_ellipsoids[ltm_ellipsoid.first] = ltm_ellipsoid.second.second;
        }
        vis_manager->visualizeEllipsoids(ltm_ellipsoids, vtr::INITIAL);
      }
      vis_manager->visualizeCameraObservations(
          max_frame_optimized,
          initial_robot_pose_estimates,
          optimized_trajectory,
          std::nullopt,
          std::nullopt,  // TODO should we extract the initial ellipsoid
                         // estimates from the front end?
          optimized_ellipsoid_estimates,
          std::nullopt,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images,
          *observed_corner_locations,
          {},
          false);

      vis_manager->publishDetectedBoundingBoxesWithUncertainty(
          max_frame_optimized,
          all_observed_corner_locations_with_uncertainty,
          images,
          intrinsics,
          img_heights_and_widths,
          vtr::PlotType::ESTIMATED);

      std::vector<vtr::Pose3D<double>> est_trajectory_vec;
      std::vector<vtr::Pose3D<double>> init_trajectory_vec;
      for (size_t i = 0; i <= (max_frame_optimized - min_frame_optimized);
           i++) {
        vtr::FrameId frame_id = i + min_frame_optimized;
        est_trajectory_vec.emplace_back(optimized_trajectory.at(frame_id));
        init_trajectory_vec.emplace_back(
            initial_robot_pose_estimates.at(frame_id));
      }
      vis_manager->visualizeTrajectory(init_trajectory_vec,
                                       vtr::PlotType::INITIAL);
      vis_manager->visualizeTrajectory(est_trajectory_vec,
                                       vtr::PlotType::ESTIMATED);
      break;
    }
    case vtr::AFTER_ALL_OPTIMIZATION:
      break;
    default:
      break;
  }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  if (FLAGS_extrinsics_file.empty()) {
    LOG(ERROR) << "No extrinsics file provided";
    exit(1);
  }

  if (FLAGS_intrinsics_file.empty()) {
    LOG(ERROR) << "No intrinsics file provided";
    exit(1);
  }

  if (FLAGS_bounding_boxes_by_node_id_file.empty()) {
    LOG(ERROR) << "No bounding box file provided";
    exit(1);
  }

  if (FLAGS_poses_by_node_id_file.empty()) {
    LOG(ERROR) << "No robot poses file provided";
    exit(1);
  }

  if (FLAGS_long_term_map_output.empty()) {
    LOG(ERROR) << "No long-term map output file provided";
    exit(1);
  }

  if (FLAGS_rosbag_file.empty()) {
    LOG(WARNING) << "No rosbag file provided";
  }

  if (FLAGS_nodes_by_timestamp_file.empty()) {
    LOG(WARNING) << "No nodes by timestamp file";
  }
  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, node_prefix + "ellipsoid_estimator_real_data");
  ros::NodeHandle n;

  // Hard-coded values -----------------------------------------------------
  std::unordered_map<std::string,
                     std::pair<vtr::ObjectDim<double>, vtr::ObjectDim<double>>>
      shape_mean_and_std_devs_by_semantic_class;
  Eigen::Vector3d chair_mean(0.62, 0.62, 0.975);
  Eigen::Vector3d chair_std_dev(0.05, 0.05, 0.05);
  std::string chair_class = "chair";
  shape_mean_and_std_devs_by_semantic_class[chair_class] =
      std::make_pair(chair_mean, chair_std_dev);
  Eigen::Vector3d cone_mean(0.29, 0.29, 0.48);
  Eigen::Vector3d cone_std_dev(0.001, 0.001, 0.01);
  std::string cone_class = "roadblock";
  shape_mean_and_std_devs_by_semantic_class[cone_class] =
      std::make_pair(cone_mean, cone_std_dev);

  pose_graph_optimization::OptimizationSolverParams solver_params;  // TODO
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      residual_params;  // TODO tune?

  vtr::RoshanBbAssociationParams roshan_associator_params;  // TODO tune these
  roshan_associator_params.saturation_histogram_bins_ = 50;
  roshan_associator_params.hue_histogram_bins_ = 60;
  roshan_associator_params.max_distance_for_associated_ellipsoids_ = 2.0;
  roshan_associator_params.min_observations_ = 6;
  roshan_associator_params.discard_candidate_after_num_frames_ = 80;

  Eigen::Vector4d bounding_box_std_devs;  // TODO maybe use different values
  bounding_box_std_devs(0) = 30;
  bounding_box_std_devs(1) = 30;
  bounding_box_std_devs(2) = 30;
  bounding_box_std_devs(3) = 30;
  vtr::Covariance<double, 4> bounding_box_covariance =
      vtr::createDiagCovFromStdDevs(bounding_box_std_devs);

  LOG(INFO) << "Here 2";

  // TODO read this from file
  std::unordered_map<std::string, vtr::CameraId> camera_topic_to_camera_id = {
      {"/camera/rgb/image_raw", 0}, {"/camera/rgb/image_raw/compressed", 0}};

  // Post-processing of the hard-coded values ---------------------------
  std::unordered_map<
      std::string,
      std::pair<vtr::ObjectDim<double>, vtr::Covariance<double, 3>>>
      mean_and_cov_by_semantic_class;
  for (const auto &shape_mean_and_std_dev_for_class :
       shape_mean_and_std_devs_by_semantic_class) {
    mean_and_cov_by_semantic_class[shape_mean_and_std_dev_for_class.first] =
        std::make_pair(shape_mean_and_std_dev_for_class.second.first,
                       vtr::createDiagCovFromStdDevs(
                           shape_mean_and_std_dev_for_class.second.second));
  }

  // Read necessary data in from file -----------------------------------------
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera =
          readCameraIntrinsicsByCameraFromFile(FLAGS_intrinsics_file);
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      camera_extrinsics_by_camera =
          readCameraExtrinsicsByCameraFromFile(FLAGS_extrinsics_file);
  std::unordered_map<vtr::FeatureId, vtr::StructuredVisionFeatureTrack>
      visual_features;  // TODO read this when we actually have this
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      init_bounding_boxes =
          readBoundingBoxesFromFile(FLAGS_bounding_boxes_by_node_id_file);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bounding_boxes;
  // tODO fix this
  for (const auto &frame_id_bb_entry : init_bounding_boxes) {
    for (const auto &camera_bb_entry : frame_id_bb_entry.second) {
      for (const vtr::RawBoundingBox &bb : camera_bb_entry.second) {
        if ((bb.pixel_corner_locations_.second.x() < 1280) &&
            (bb.pixel_corner_locations_.second.y() < 720)) {
          bounding_boxes[frame_id_bb_entry.first][camera_bb_entry.first]
              .emplace_back(bb);
        }
      }
    }
  }
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::vector<std::pair<vtr::BbCornerPair<double>,
                                               std::optional<double>>>>>
      all_observed_corner_locations_with_uncertainty;
  for (const auto &bounding_boxes_for_frame : bounding_boxes) {
    for (const auto &bounding_boxes_for_frame_and_cam :
         bounding_boxes_for_frame.second) {
      std::vector<std::pair<vtr::BbCornerPair<double>, std::optional<double>>>
          observed_corners_for_frame_and_cam;
      for (const vtr::RawBoundingBox &bb :
           bounding_boxes_for_frame_and_cam.second) {
        observed_corners_for_frame_and_cam.emplace_back(std::make_pair(
            bb.pixel_corner_locations_, bb.detection_confidence_));
      }
      all_observed_corner_locations_with_uncertainty
          [bounding_boxes_for_frame.first]
          [bounding_boxes_for_frame_and_cam.first] =
              observed_corners_for_frame_and_cam;
    }
  }

  //  LOG(INFO) << "Bounding boxes for " << bounding_boxes.size() << " frames ";
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses =
      readRobotPosesFromFile(FLAGS_poses_by_node_id_file);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = getImagesFromRosbag(FLAGS_rosbag_file,
                                   FLAGS_nodes_by_timestamp_file,
                                   camera_topic_to_camera_id);

  std::unordered_map<vtr::CameraId, std::pair<double, double>>
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

  LOG(INFO) << "Here 3";
  MainLtmPtr long_term_map;
  if (!FLAGS_long_term_map_input.empty()) {
    cv::FileStorage ltm_in_fs(FLAGS_long_term_map_input, cv::FileStorage::READ);
    vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
        std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>,
        vtr::SerializableMap<vtr::ObjectId,
                             vtr::SerializableObjectId,
                             vtr::RoshanAggregateBbInfo,
                             vtr::SerializableRoshanAggregateBbInfo>>
        serializable_ltm;
    ltm_in_fs["long_term_map"] >> serializable_ltm;
    ltm_in_fs.release();
    MainLtm ltm_from_serializable = serializable_ltm.getEntry();
    vtr::EllipsoidResults ellipsoid_results_ltm_v3;
    ltm_from_serializable.getEllipsoidResults(ellipsoid_results_ltm_v3);
    LOG(INFO) << "Second check results size "
              << ellipsoid_results_ltm_v3.ellipsoids_.size();
    long_term_map = std::make_shared<MainLtm>(ltm_from_serializable);
  }

  LOG(INFO) << "Here 4";
  MainProbData input_problem_data(camera_intrinsics_by_camera,
                                  camera_extrinsics_by_camera,
                                  visual_features,
                                  robot_poses,
                                  mean_and_cov_by_semantic_class,
                                  bounding_boxes,
                                  long_term_map,
                                  images);

  // Connect up functions needed for the optimizer --------------------------
  std::shared_ptr<vtr::RosVisualization> vis_manager =
      std::make_shared<vtr::RosVisualization>(n);

  vtr::IndependentEllipsoidsLongTermObjectMapFactorCreator<
      util::EmptyStruct,
      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
      ltm_factor_creator(long_term_map);

  std::function<bool()> continue_opt_checker = []() { return ros::ok(); };

  std::function<vtr::FrameId(const vtr::FrameId &)> window_provider_func =
      [](const vtr::FrameId &) {
        // For now, we'll just optimize the whole trajectory (so return 0 so we
        // start the optimization with node 0
        return 0;
      };

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, const util::EmptyStruct &)>
      refresh_residual_checker = checkFactorRefresh;

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
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator =
          [&](const MainFactorInfo &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const MainPgPtr &pose_graph,
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
                                       cached_info);
          };

  LOG(INFO) << "Here 5";
  std::function<bool(util::BoostHashSet<MainFactorInfo> &)>
      long_term_map_factor_provider =
          [&](util::BoostHashSet<MainFactorInfo> &factor_data) {
            return ltm_factor_creator.getFactorsToInclude(factor_data);
          };
  std::function<void(const MainProbData &, MainPgPtr &)> pose_graph_creator =
      std::bind(createPoseGraph,
                std::placeholders::_1,
                long_term_map_factor_provider,
                std::placeholders::_2);
  std::function<double(const MainProbData &,
                       const MainPgPtr &,
                       const vtr::FrameId &,
                       const vtr::FeatureId &,
                       const vtr::CameraId &)>
      reprojection_error_provider = [](const MainProbData &input_problem,
                                       const MainPgPtr &pose_graph,
                                       const vtr::FrameId &frame_id,
                                       const vtr::FeatureId &feature_id,
                                       const vtr::CameraId &camera_id) {
        // TODO replace with thought out function once we add in the visual
        // part
        return 0.0;
      };
  LOG(INFO) << "Here 6";
  std::function<std::pair<bool, std::optional<sensor_msgs::Image::ConstPtr>>(
      const vtr::FrameId &, const vtr::CameraId &, const MainProbData &)>
      bb_context_retriever = [](const vtr::FrameId &frame_id,
                                const vtr::CameraId &camera_id,
                                const MainProbData &problem_data) {
        //            LOG(INFO) << "Getting image for frame " << frame_id
        //                      << " and camera " << camera_id;
        std::optional<sensor_msgs::Image::ConstPtr> image =
            problem_data.getImageForFrameAndCamera(frame_id, camera_id);
        return std::make_pair(image.has_value(), image);
      };

  std::function<vtr::Covariance<double, 4>(const vtr::RawBoundingBox &,
                                           const vtr::FrameId &,
                                           const vtr::CameraId &,
                                           const vtr::RoshanImageSummaryInfo &)>
      covariance_generator = [&](const vtr::RawBoundingBox &,
                                 const vtr::FrameId &,
                                 const vtr::CameraId &,
                                 const vtr::RoshanImageSummaryInfo &) {
        // TODO consider checking if bb is close to image boundary and blowing
        // up covariance if that is the case
        return bounding_box_covariance;
      };

  LOG(INFO) << "Here 7";
  std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>
      long_term_map_front_end_data;
  if (long_term_map != nullptr) {
    long_term_map->getFrontEndObjMapData(long_term_map_front_end_data);
  }
  std::shared_ptr<std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::unordered_map<vtr::ObjectId,
                                            std::pair<vtr::BbCornerPair<double>,
                                                      std::optional<double>>>>>>
      associated_observed_corner_locations =
          std::make_shared<std::unordered_map<
              vtr::FrameId,
              std::unordered_map<
                  vtr::CameraId,
                  std::unordered_map<vtr::ObjectId,
                                     std::pair<vtr::BbCornerPair<double>,
                                               std::optional<double>>>>>>();
  vtr::RoshanBbFrontEndCreator<vtr::ReprojectionErrorFactor>
      roshan_associator_creator(roshan_associator_params,
                                associated_observed_corner_locations,
                                covariance_generator,
                                long_term_map_front_end_data);
  std::function<std::shared_ptr<vtr::AbstractBoundingBoxFrontEnd<
      vtr::ReprojectionErrorFactor,
      vtr::RoshanAggregateBbInfo,
      std::optional<sensor_msgs::Image::ConstPtr>,
      vtr::RoshanImageSummaryInfo,
      vtr::RoshanBbInfo,
      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>>(
      const MainPgPtr &, const MainProbData &)>
      bb_associator_retriever =
          [&](const MainPgPtr &pg, const MainProbData &input_prob) {
            return roshan_associator_creator.getDataAssociator(pg);
          };
  LOG(INFO) << "Here 8";
  std::function<void(
      const MainProbData &, const MainPgPtr &, const vtr::FrameId &)>
      frame_data_adder = [&](const MainProbData &problem_data,
                             const MainPgPtr &pose_graph,
                             const vtr::FrameId &frame_to_add) {
        vtr::addFrameDataAssociatedBoundingBox(problem_data,
                                               pose_graph,
                                               frame_to_add,
                                               reprojection_error_provider,
                                               bb_associator_retriever,
                                               bb_context_retriever);
      };

  //  std::function<void(
  //      const vtr::UnassociatedBoundingBoxOfflineProblemData<
  //          vtr::StructuredVisionFeatureTrack,
  //          sensor_msgs::Image::ConstPtr> &,
  //      const std::shared_ptr<const
  //      MainPg> &, ceres::Problem *,
  //      vtr::SpatialEstimateOnlyResults &)>
  //      output_data_extractor =
  //          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
  //                 vtr::StructuredVisionFeatureTrack,
  //                 sensor_msgs::Image::ConstPtr> &input_problem_data,
  //             const std::shared_ptr<
  //                 const MainPg>
  //                 &pose_graph,
  //             ceres::Problem *problem,
  //             vtr::SpatialEstimateOnlyResults &output_problem_data) {
  //            vtr::extractSpatialEstimateOnlyResults(pose_graph,
  //                                                   output_problem_data);
  //          };

  LOG(INFO) << "Here 9";
  vtr::CovarianceExtractorParams ltm_covariance_params;

  // TODO maybe replace params with something that will yield more accurate
  // results
  vtr::IndependentEllipsoidsLongTermObjectMapExtractor<
      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
      ltm_extractor(ltm_covariance_params,
                    residual_creator,
                    residual_params,
                    solver_params);

  std::function<void(
      const MainProbData &,
      const MainPgPtr &,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
      vtr::LongTermObjectMapAndResults<MainLtm> &)>
      output_data_extractor = [&](const MainProbData &input_problem_data,
                                  const MainPgPtr &pose_graph,
                                  const pose_graph_optimizer::
                                      OptimizationFactorsEnabledParams
                                          &optimization_factors_enabled_params,
                                  vtr::LongTermObjectMapAndResults<MainLtm>
                                      &output_problem_data) {
        std::function<bool(
            std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo> &)>
            front_end_map_data_extractor =
                [&](std::unordered_map<vtr::ObjectId,
                                       vtr::RoshanAggregateBbInfo>
                        &front_end_data) {
                  roshan_associator_creator.getDataAssociator(pose_graph)
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
                  return ltm_extractor.extractLongTermObjectMap(
                      ltm_pose_graph,
                      ltm_optimization_factors_enabled_params,
                      front_end_map_data_extractor,
                      ltm_extractor_out);
                };  // TODO!
        vtr::extractLongTermObjectMapAndResults(
            pose_graph,
            optimization_factors_enabled_params,
            long_term_object_map_extractor,
            output_problem_data);
      };

  LOG(INFO) << "Here 10";
  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const MainProbData &,
      const MainPgPtr &,
      const vtr::FrameId &,
      const vtr::FrameId &)>
      ceres_callback_creator = dummyCeresCallbackCreator;
  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const vtr::FrameId &,
                     const vtr::FrameId &,
                     const vtr::VisualizationTypeEnum &)>
      visualization_callback = [&](const MainProbData &input_problem_data,
                                   const MainPgPtr &pose_graph,
                                   const vtr::FrameId &min_frame_id,
                                   const vtr::FrameId &max_frame_id,
                                   const vtr::VisualizationTypeEnum
                                       &visualization_type) {
        std::shared_ptr<
            vtr::ObjAndLowLevelFeaturePoseGraph<vtr::ReprojectionErrorFactor>>
            superclass_ptr = pose_graph;
        visualizationStub(vis_manager,
                          camera_extrinsics_by_camera,
                          camera_intrinsics_by_camera,
                          img_heights_and_widths,
                          images,
                          all_observed_corner_locations_with_uncertainty,
                          associated_observed_corner_locations,
                          input_problem_data,
                          superclass_ptr,
                          min_frame_id,
                          max_frame_id,
                          visualization_type);
      };
  LOG(INFO) << "Here 11";
  vtr::OfflineProblemRunner<MainProbData,
                            vtr::ReprojectionErrorFactor,
                            vtr::LongTermObjectMapAndResults<MainLtm>,
                            util::EmptyStruct,
                            MainPg>
      offline_problem_runner(residual_params,
                             continue_opt_checker,
                             window_provider_func,
                             refresh_residual_checker,
                             residual_creator,
                             pose_graph_creator,
                             frame_data_adder,
                             output_data_extractor,
                             ceres_callback_creator,
                             visualization_callback,
                             solver_params);

  pose_graph_optimizer::OptimizationFactorsEnabledParams
      optimization_factors_enabled_params;
  optimization_factors_enabled_params.use_pom_ = false;
  optimization_factors_enabled_params.include_visual_factors_ = false;
  optimization_factors_enabled_params.fix_poses_ = true;
  optimization_factors_enabled_params.fix_objects_ = false;
  // TODO should we also optimize the poses?

  //  vtr::SpatialEstimateOnlyResults output_results;
  vtr::LongTermObjectMapAndResults<MainLtm> output_results;
  LOG(INFO) << "Here 12";
  offline_problem_runner.runOptimization(
      input_problem_data, optimization_factors_enabled_params, output_results);

  LOG(INFO) << "Here 13";
  cv::FileStorage ltm_out_fs(FLAGS_long_term_map_output,
                             cv::FileStorage::WRITE);
  ltm_out_fs
      << "long_term_map"
      << vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
             std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>,
             vtr::SerializableMap<vtr::ObjectId,
                                  vtr::SerializableObjectId,
                                  vtr::RoshanAggregateBbInfo,
                                  vtr::SerializableRoshanAggregateBbInfo>>(
             output_results.long_term_map_);
  ltm_out_fs.release();
  LOG(INFO) << "Here 14";
  //  LOG(INFO) << "Num ellipsoids "
  //            << output_results.ellipsoid_results_.ellipsoids_.size();

  // TODO save output results somewhere

  //  vtr::EllipsoidState<double> ellipsoid(
  //      vtr::Pose3D(Eigen::Vector3d(1, 2, 3),
  //                  Eigen::AngleAxisd(0.2, Eigen::Vector3d(0, 0, 1))),
  //      Eigen::Vector3d(4, 5, 6));
  //
  //  cv::FileStorage fs("test.json", cv::FileStorage::WRITE);
  //  fs << "ellipsoid" << vtr::SerializableEllipsoidState<double>(ellipsoid);
  //
  //  cv::FileStorage fs2("test.json", cv::FileStorage::READ);
  //  vtr::SerializableEllipsoidState<double> ellipsoid_state;
  //  fs2["ellipsoid"] >> ellipsoid_state;
  //  LOG(INFO) << "Read ellipsoid";
  //  LOG(INFO) << "Transl " << ellipsoid_state.getEntry().pose_.transl_;
  //  LOG(INFO) << "Orientation angle " <<
  //  ellipsoid_state.getEntry().pose_.orientation_.angle(); LOG(INFO) <<
  //  "Orientation axis" <<
  //  ellipsoid_state.getEntry().pose_.orientation_.axis(); LOG(INFO) <<
  //  "Dimensions " << ellipsoid_state.getEntry().dimensions_;

  return 0;
}