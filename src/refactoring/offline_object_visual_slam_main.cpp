#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/roshan_bounding_box_front_end.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/offline/pose_graph_frame_data_adder.h>
#include <refactoring/optimization/residual_creator.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

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
    vtr::RawBoundingBox bb;
    bb.pixel_corner_locations_ = std::make_pair(
        vtr::PixelCoord<double>(raw_bb.min_pixel_x, raw_bb.min_pixel_y),
        vtr::PixelCoord<double>(raw_bb.max_pixel_x, raw_bb.max_pixel_y));
    bb.semantic_class_ = raw_bb.semantic_class;
    bb_map[raw_bb.node_id][raw_bb.camera_id].emplace_back(bb);
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
  // TODO do we want to make a new back with uncompressed images or handle the compression here?

  std::vector<std::string> topics;
  for (const auto &camera_topic_and_id : camera_topic_to_camera_id) {
    topics.emplace_back(camera_topic_and_id.first);
    LOG(INFO) << "Checking topic " << camera_topic_and_id.first;
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images_by_frame_and_cam;
  for (const rosbag::MessageInstance &m : view) {
//    LOG(INFO) << "Checking image message";
    sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
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
dummyCeresCallbackCreator(
    const vtr::UnassociatedBoundingBoxOfflineProblemData<
        vtr::StructuredVisionFeatureTrack,
        sensor_msgs::Image::ConstPtr> &input_problem_data,
    const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const vtr::FrameId &min_frame_optimized,
    const vtr::FrameId &max_frame_optimized) {
  // TODO replace with actual code later
  return {};
}

bool checkFactorRefresh(
    const std::pair<vtr::FactorType, vtr::FeatureFactorId> &factor,
    const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
    const util::EmptyStruct &) {
  return false;
}

void createPoseGraph(
    const vtr::UnassociatedBoundingBoxOfflineProblemData<
        vtr::StructuredVisionFeatureTrack,
        sensor_msgs::Image::ConstPtr> &input_problem_data,
    std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &pose_graph) {
  pose_graph = std::make_shared<vtr::ObjectAndReprojectionFeaturePoseGraph>(
      input_problem_data.getObjDimMeanAndCovByClass(),
      input_problem_data.getCameraExtrinsicsByCamera(),
      input_problem_data.getCameraIntrinsicsByCamera());
}

void visualizationStub(const vtr::UnassociatedBoundingBoxOfflineProblemData<
                           vtr::StructuredVisionFeatureTrack,
                           sensor_msgs::Image::ConstPtr> &input_problem_data,
                       std::shared_ptr<vtr::ObjAndLowLevelFeaturePoseGraph<
                           vtr::ReprojectionErrorFactor>> &pose_graph,
                       const vtr::FrameId &min_frame_optimized,
                       const vtr::FrameId &max_frame_optimized,
                       const vtr::VisualizationTypeEnum &visualization_stage) {
  // TODO fill in with actual visualization
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

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

  if (FLAGS_rosbag_file.empty()) {
    LOG(WARNING) << "No rosbag file provided";
  }

  if (FLAGS_nodes_by_timestamp_file.empty()) {
    LOG(WARNING) << "No nodes by timestamp file";
  }

  // Hard-coded values -----------------------------------------------------
  std::unordered_map<std::string,
                     std::pair<vtr::ObjectDim<double>, vtr::ObjectDim<double>>>
      shape_mean_and_std_devs_by_semantic_class;
  Eigen::Vector3d chair_mean(0.62, 0.62, 0.975);
  Eigen::Vector3d chair_std_dev(0.26, 0.42, 0.33);
  std::string chair_class = "chair";
  shape_mean_and_std_devs_by_semantic_class[chair_class] =
      std::make_pair(chair_mean, chair_std_dev);

  pose_graph_optimization::OptimizationSolverParams solver_params;  // TODO
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      residual_params;  // TODO tune?

  vtr::RoshanBbAssociationParams roshan_associator_params;  // TODO tune these
  roshan_associator_params.saturation_histogram_bins = 50;
  roshan_associator_params.hue_histogram_bins = 60;
  roshan_associator_params.max_distance_for_associated_ellipsoids_ = 2.5;

  Eigen::Vector4d bounding_box_std_devs;  // TODO maybe use different values
  bounding_box_std_devs(0) = 30;
  bounding_box_std_devs(1) = 30;
  bounding_box_std_devs(2) = 30;
  bounding_box_std_devs(3) = 30;
  vtr::Covariance<double, 4> bounding_box_covariance =
      vslam_util::createDiagCovFromStdDevs(bounding_box_std_devs);

  // TODO read this from file
  std::unordered_map<std::string, vtr::CameraId> camera_topic_to_camera_id = {
      {"/camera/rgb/image_raw", 0}};

  // Post-processing of the hard-coded values ---------------------------
  std::unordered_map<
      std::string,
      std::pair<vtr::ObjectDim<double>, vtr::Covariance<double, 3>>>
      mean_and_cov_by_semantic_class;
  for (const auto &shape_mean_and_std_dev_for_class :
       shape_mean_and_std_devs_by_semantic_class) {
    mean_and_cov_by_semantic_class[shape_mean_and_std_dev_for_class.first] =
        std::make_pair(shape_mean_and_std_dev_for_class.second.first,
                       vslam_util::createDiagCovFromStdDevs(
                           shape_mean_and_std_dev_for_class.second.second));
  }

  // Connect up functions needed for the optimizer --------------------------
  std::function<bool()> continue_opt_checker = []() { return ros::ok(); };

  std::function<vtr::FrameId(const vtr::FrameId &)> window_provider_func =
      [](const vtr::FrameId &) {
        // For now, we'll just optimize the whole trajectory (so return 0 so we
        // start the optimization with node 0
        return 0;
      };

  std::function<bool(
      const std::pair<vtr::FactorType, vtr::FeatureFactorId> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const util::EmptyStruct &)>
      refresh_residual_checker = checkFactorRefresh;

  std::function<bool(
      const std::pair<vtr::FactorType, vtr::FeatureFactorId> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      util::EmptyStruct &)>
      cached_info_creator =
          [](const std::pair<vtr::FactorType, vtr::FeatureFactorId>
                 &factor_info,
             const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             util::EmptyStruct &cached_info) {
            return true;  // TODO maybe fill in with real info some day
          };
  std::function<bool(
      const std::pair<vtr::FactorType, vtr::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator =
          [&](const std::pair<vtr::FactorType, vtr::FeatureFactorId> &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                  &pose_graph,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return vtr::createResidual(factor_id,
                                       pose_graph,
                                       solver_residual_params,
                                       cached_info_creator,
                                       problem,
                                       residual_id,
                                       cached_info);
          };
  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &)>
      pose_graph_creator = createPoseGraph;
  std::function<double(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &,
      const vtr::FeatureId &,
      const vtr::CameraId &)>
      reprojection_error_provider =
          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
                 vtr::StructuredVisionFeatureTrack,
                 sensor_msgs::Image::ConstPtr> &input_problem,
             const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             const vtr::FrameId &frame_id,
             const vtr::FeatureId &feature_id,
             const vtr::CameraId &camera_id) {
            // TODO replace with thought out function once we add in the visual
            // part
            return 0.0;
          };
  std::function<std::optional<sensor_msgs::Image::ConstPtr>(
      const vtr::FrameId &,
      const vtr::CameraId &,
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &)>
      bb_context_retriever =
          [](const vtr::FrameId &frame_id,
             const vtr::CameraId &camera_id,
             const vtr::UnassociatedBoundingBoxOfflineProblemData<
                 vtr::StructuredVisionFeatureTrack,
                 sensor_msgs::Image::ConstPtr> &problem_data) {
            LOG(INFO) << "Getting image for frame " << frame_id
                      << " and camera " << camera_id;
            return problem_data.getImageForFrameAndCamera(frame_id, camera_id);
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

  vtr::RoshanBbFrontEndCreator<vtr::ReprojectionErrorFactor>
      roshan_associator_creator(roshan_associator_params, covariance_generator);
  std::function<std::shared_ptr<
      vtr::AbstractBoundingBoxFrontEnd<vtr::ReprojectionErrorFactor,
                                       vtr::RoshanAggregateBbInfo,
                                       std::optional<sensor_msgs::Image::ConstPtr>,
                                       vtr::RoshanImageSummaryInfo,
                                       vtr::RoshanBbInfo>>(
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &)>
      bb_associator_retriever =
          [&](const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                  &pg,
              const vtr::UnassociatedBoundingBoxOfflineProblemData<
                  vtr::StructuredVisionFeatureTrack,
                  sensor_msgs::Image::ConstPtr> &input_prob) {
            return roshan_associator_creator.getDataAssociator(pg);
          };

  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &)>
      frame_data_adder =
          [&](const vtr::UnassociatedBoundingBoxOfflineProblemData<
                  vtr::StructuredVisionFeatureTrack,
                  sensor_msgs::Image::ConstPtr> &problem_data,
              const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                  &pose_graph,
              const vtr::FrameId &frame_to_add) {
            vtr::addFrameDataAssociatedBoundingBox(problem_data,
                                                   pose_graph,
                                                   frame_to_add,
                                                   reprojection_error_provider,
                                                   bb_associator_retriever,
                                                   bb_context_retriever);
          };

  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<const vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      vtr::SpatialEstimateOnlyResults &)>
      output_data_extractor =
          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
                 vtr::StructuredVisionFeatureTrack,
                 sensor_msgs::Image::ConstPtr> &input_problem_data,
             const std::shared_ptr<
                 const vtr::ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
             vtr::SpatialEstimateOnlyResults &output_problem_data) {
            vtr::extractSpatialEstimateOnlyResults(pose_graph,
                                                   output_problem_data);
          };

  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &,
      const vtr::FrameId &)>
      ceres_callback_creator = dummyCeresCallbackCreator;
  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &,
      const vtr::FrameId &,
      const vtr::VisualizationTypeEnum &)>
      visualization_callback =
          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
                 vtr::StructuredVisionFeatureTrack,
                 sensor_msgs::Image::ConstPtr> &input_problem_data,
             const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             const vtr::FrameId &min_frame_id,
             const vtr::FrameId &max_frame_id,
             const vtr::VisualizationTypeEnum &visualization_type) {
            std::shared_ptr<vtr::ObjAndLowLevelFeaturePoseGraph<
                vtr::ReprojectionErrorFactor>>
                superclass_ptr = pose_graph;
            visualizationStub(input_problem_data,
                              superclass_ptr,
                              min_frame_id,
                              max_frame_id,
                              visualization_type);
          };

  vtr::OfflineProblemRunner<vtr::UnassociatedBoundingBoxOfflineProblemData<
                                vtr::StructuredVisionFeatureTrack,
                                sensor_msgs::Image::ConstPtr>,
                            vtr::ReprojectionErrorFactor,
                            vtr::SpatialEstimateOnlyResults,
                            util::EmptyStruct,
                            vtr::ObjectAndReprojectionFeaturePoseGraph>
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
      bounding_boxes =
          readBoundingBoxesFromFile(FLAGS_bounding_boxes_by_node_id_file);
  //  LOG(INFO) << "Bounding boxes for " << bounding_boxes.size() << " frames ";
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses =
      readRobotPosesFromFile(FLAGS_poses_by_node_id_file);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = getImagesFromRosbag(FLAGS_rosbag_file,
                                   FLAGS_nodes_by_timestamp_file,
                                   camera_topic_to_camera_id);

  LOG(INFO) << "Images for " << images.size() << " frames ";
  for (const auto &img_pair : images) {
    LOG(INFO) << "Frame " << img_pair.first << " has " << img_pair.second.size()
              << " cameras ";
    for (const auto &cam_img_pair : img_pair.second) {
      LOG(INFO) << "Cam " << cam_img_pair.first;
      break;
    }
    break;
  }

  vtr::UnassociatedBoundingBoxOfflineProblemData<
      vtr::StructuredVisionFeatureTrack,
      sensor_msgs::Image::ConstPtr>
      input_problem_data(camera_intrinsics_by_camera,
                         camera_extrinsics_by_camera,
                         visual_features,
                         robot_poses,
                         mean_and_cov_by_semantic_class,
                         bounding_boxes,
                         images);

  vtr::OptimizationFactorsEnabledParams optimization_factors_enabled_params;
  optimization_factors_enabled_params.use_pom_ = false;
  optimization_factors_enabled_params.include_visual_factors_ = false;
  optimization_factors_enabled_params.fix_poses_ = true;
  optimization_factors_enabled_params.fix_objects_ = false;
  // TODO should we also optimize the poses?

  vtr::SpatialEstimateOnlyResults output_results;

  offline_problem_runner.runOptimization(
      input_problem_data, optimization_factors_enabled_params, output_results);

  // TODO save output results somewhere

  return 0;
}