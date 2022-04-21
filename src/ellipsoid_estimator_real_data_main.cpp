
#include <base_lib/pose_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <object_slam_backend_solver.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <shape_prior_factor.h>
#include <visual_slam_ceres_visualization_callback.h>
#include <visualization/ros_visualization.h>

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(bounding_boxes_by_node_id_file,
              "",
              "File with bounding box observations by node id");
DEFINE_string(poses_by_node_id_file,
              "",
              "File with initial robot pose estimates");
DEFINE_string(intrinsics_file, "", "File with camera intrinsics");
DEFINE_string(extrinsics_file, "", "File with camera extrinsics");
DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");

struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(std::pair<T1, T2> const &pair) const {
    std::size_t h1 = std::hash<T1>()(pair.first);
    std::size_t h2 = std::hash<T2>()(pair.second);

    return h1 ^ h2;
  }
};

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  std::string chair_class = "chair";

  // TODO read these in
  std::unordered_map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>>
      shape_mean_and_std_devs_by_semantic_class;
  Eigen::Vector3f chair_mean(0.62, 0.62, 0.975);
  Eigen::Vector3f chair_std_dev(0.26, 0.42, 0.33);
  shape_mean_and_std_devs_by_semantic_class[chair_class] =
      std::make_pair(chair_mean, chair_std_dev);

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  if (FLAGS_bounding_boxes_by_node_id_file.empty()) {
    LOG(ERROR) << "No bounding box file provided";
    exit(1);
  }

  if (FLAGS_poses_by_node_id_file.empty()) {
    LOG(ERROR) << "No robot poses file provided";
    exit(1);
  }

  if (FLAGS_extrinsics_file.empty()) {
    LOG(ERROR) << "No extrinsics file provided";
    exit(1);
  }

  if (FLAGS_intrinsics_file.empty()) {
    LOG(ERROR) << "No intrinsics file provided";
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

  util_random::Random random_gen;

  std::shared_ptr<vslam_viz::RosVisualization> viz =
      std::make_shared<vslam_viz::RosVisualization>(n, param_prefix);

  LOG(INFO) << "Done creating viz obj";

  // TODO read this in
  std::vector<vslam_types::EllipsoidEstimate> initial_ests;
  Eigen::Vector3f ellipsoid_loc(2.1, 0, chair_mean.z() / 2);
  Eigen::AngleAxisf ellipsoid_orientation(0, Eigen::Vector3f(1, 0, 0));
  Eigen::Vector3f ellipsoid_dim = chair_mean;
  vslam_types::EllipsoidEstimate initial_est(
      ellipsoid_loc, ellipsoid_orientation, ellipsoid_dim, chair_class, 0);
  initial_ests.emplace_back(initial_est);

  // TODO read this in
  Eigen::Vector4f bounding_box_std_devs;  // TODO maybe use different values
  bounding_box_std_devs(0) = 30;
  bounding_box_std_devs(1) = 30;
  bounding_box_std_devs(2) = 30;
  bounding_box_std_devs(3) = 30;
  Eigen::Matrix4f bounding_box_covariance =
      vslam_util::createDiagCovFromStdDevs(bounding_box_std_devs);

  std::unordered_map<std::string, std::pair<Eigen::Vector3f, Eigen::Matrix3f>>
      mean_and_cov_by_semantic_class;
  for (const auto &shape_mean_and_std_dev_for_class :
       shape_mean_and_std_devs_by_semantic_class) {
    mean_and_cov_by_semantic_class[shape_mean_and_std_dev_for_class.first] =
        std::make_pair(shape_mean_and_std_dev_for_class.second.first,
                       vslam_util::createDiagCovFromStdDevs(
                           shape_mean_and_std_dev_for_class.second.second));
  }

  vslam_solver::StructuredObjectSlamProblemParams problem_params;
  problem_params.object_slam_params.ellipsoid_bounding_box_constraint_params
      .bounding_box_covariance = bounding_box_covariance;
  problem_params.object_slam_params.semantic_shape_prior_params
      .mean_and_cov_by_semantic_class = mean_and_cov_by_semantic_class;

  std::vector<file_io::BoundingBoxWithNodeIdAndId> bounding_boxes_by_node_id;
  file_io::readBoundingBoxWithNodeIdAndIdsFromFile(
      FLAGS_bounding_boxes_by_node_id_file, bounding_boxes_by_node_id);

  std::vector<vslam_types::ObjectImageBoundingBoxDetection>
      bounding_box_detections;

  // Outer key is robot pose, next is camera id, inner most is ellipsoid id
  std::unordered_map<
      uint64_t,
      std::unordered_map<
          vslam_types::CameraId,
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>>
      observed_corner_locations;
  for (const file_io::BoundingBoxWithNodeIdAndId &raw_bounding_box :
       bounding_boxes_by_node_id) {
    vslam_types::ObjectImageBoundingBoxDetection problem_bb;
    problem_bb.ellipsoid_idx = raw_bounding_box.ellipsoid_idx;
    problem_bb.pixel_corner_locations =
        std::make_pair(Eigen::Vector2f(raw_bounding_box.min_pixel_x,
                                       raw_bounding_box.min_pixel_y),
                       Eigen::Vector2f(raw_bounding_box.max_pixel_x,
                                       raw_bounding_box.max_pixel_y));
    problem_bb.camera_id = raw_bounding_box.camera_id;
    problem_bb.frame_idx = raw_bounding_box.node_id;
    problem_bb.semantic_class = raw_bounding_box.semantic_class;
    bounding_box_detections.emplace_back(problem_bb);

    observed_corner_locations[raw_bounding_box.node_id][problem_bb.camera_id]
                             [problem_bb.ellipsoid_idx] =
                                 problem_bb.pixel_corner_locations;
  }

  LOG(INFO) << "Done reading bounding boxes";

  std::vector<vslam_types::RobotPose> robot_poses;
  std::vector<std::pair<uint64_t, pose::Pose3d>> robot_poses_by_node_id;
  file_io::readPose3dsAndNodeIdFromFile(FLAGS_poses_by_node_id_file,
                                        robot_poses_by_node_id);
  std::unordered_map<uint64_t, vslam_types::RobotPose> robot_poses_by_node_num;
  for (const std::pair<uint64, pose::Pose3d> &pose_3d :
       robot_poses_by_node_id) {
    vslam_types::RobotPose robot_pose(
        pose_3d.first,
        pose_3d.second.first.cast<float>(),
        Eigen::AngleAxisf(pose_3d.second.second.cast<float>()));
    robot_poses_by_node_num[pose_3d.first] = robot_pose;
  }

  for (size_t node_num = 0; node_num < robot_poses_by_node_id.size();
       node_num++) {
    if (robot_poses_by_node_num.find(node_num) ==
        robot_poses_by_node_num.end()) {
      LOG(ERROR) << "No robot pose for node num " << node_num;
      exit(1);
    }
//    if (node_num >= 5) {
//      break;
//    }
    robot_poses.emplace_back(robot_poses_by_node_num[node_num]);
  }

  std::unordered_map<vslam_types::CameraId, vslam_types::CameraIntrinsics>
      intrinsics;
  std::vector<file_io::CameraIntrinsicsWithId> camera_intrinsics_by_cam_id;
  file_io::readCameraIntrinsicsWithIdsFromFile(FLAGS_intrinsics_file,
                                               camera_intrinsics_by_cam_id);
  for (const file_io::CameraIntrinsicsWithId &intrinsics_for_cam :
       camera_intrinsics_by_cam_id) {
    vslam_types::CameraIntrinsics intrinsics_obj;
    intrinsics_obj.camera_mat << intrinsics_for_cam.mat_00,
        intrinsics_for_cam.mat_01, intrinsics_for_cam.mat_02,
        intrinsics_for_cam.mat_10, intrinsics_for_cam.mat_11,
        intrinsics_for_cam.mat_12, intrinsics_for_cam.mat_20,
        intrinsics_for_cam.mat_21, intrinsics_for_cam.mat_22;
    intrinsics_obj.image_height = intrinsics_for_cam.img_height;
    intrinsics_obj.image_width = intrinsics_for_cam.img_width;
    LOG(INFO) << "Cam mat for cam " << intrinsics_for_cam.camera_id << ": "
              << intrinsics_obj.camera_mat;
    intrinsics[intrinsics_for_cam.camera_id] = intrinsics_obj;
  }

  LOG(INFO) << "Done reading intrinsics";

  std::vector<file_io::CameraExtrinsicsWithId> camera_extrinsics_by_cam_id;
  file_io::readCameraExtrinsicsWithIdsFromFile(FLAGS_extrinsics_file,
                                               camera_extrinsics_by_cam_id);
  std::unordered_map<vslam_types::CameraId, vslam_types::CameraExtrinsics>
      extrinsics;

  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  Eigen::Quaternionf extrinsics_orientation_switch_to_cam =
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse();  // [0 -1 0; 0 0 -1; 1 0 0]^-1
  for (const file_io::CameraExtrinsicsWithId &extrinsics_for_cam :
       camera_extrinsics_by_cam_id) {
    vslam_types::CameraExtrinsics extrinsics_obj;
    extrinsics_obj.translation = Eigen::Vector3f(extrinsics_for_cam.transl_x,
                                                 extrinsics_for_cam.transl_y,
                                                 extrinsics_for_cam.transl_z);

    // TODO Verify that this is correct (and eventually just update the file to
    // have this)
    extrinsics_obj.rotation = Eigen::Quaternionf(extrinsics_for_cam.quat_w,
                                                 extrinsics_for_cam.quat_x,
                                                 extrinsics_for_cam.quat_y,
                                                 extrinsics_for_cam.quat_z) *
                              extrinsics_orientation_switch_to_cam;
    extrinsics[extrinsics_for_cam.camera_id] = extrinsics_obj;
  }

  std::unordered_map<
      uint64_t,
      std::unordered_map<vslam_types::CameraId, sensor_msgs::ImagePtr>>
      images;

  if (!FLAGS_nodes_by_timestamp_file.empty() && !FLAGS_rosbag_file.empty()) {
    std::vector<file_io::NodeIdAndTimestamp> nodes_by_timestamps_vec;
    std::unordered_map<pose::Timestamp, uint64_t, pair_hash>
        nodes_for_timestamps_map;
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

    // TODO read in a mapping from camera topic to camera id instead
    vslam_types::CameraId cam_id = 0;
    std::vector<std::string> topics = {"/camera/rgb/image_raw"};

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance &m : view) {
      sensor_msgs::ImagePtr msg = m.instantiate<sensor_msgs::Image>();
      pose::Timestamp img_timestamp =
          std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);
      if (nodes_for_timestamps_map.find(img_timestamp) !=
          nodes_for_timestamps_map.end()) {
        images[nodes_for_timestamps_map[img_timestamp]][cam_id] = msg;
      }
    }
  }

  LOG(INFO) << "Done reading stuff";

  vslam_types::UTObjectSLAMProblem<vslam_types::StructuredVisionFeatureTrack>
      ut_slam_problem;
  ut_slam_problem.ellipsoid_estimates = initial_ests;
  ut_slam_problem.tracks = {};
  ut_slam_problem.robot_poses = robot_poses;
  ut_slam_problem.bounding_boxes = bounding_box_detections;
  ut_slam_problem.camera_extrinsics_by_camera = extrinsics;
  ut_slam_problem.camera_instrinsics_by_camera = intrinsics;

  vslam_solver::SLAMSolverOptimizerParams optimizer_params;

  std::vector<vslam_types::EllipsoidEstimate> updated_ellipsoid_estimates;

  std::function<std::vector<vslam_types::VisionFeature>(
      const vslam_types::StructuredVisionFeatureTrack &)>
      feature_retriever =
          [](const vslam_types::StructuredVisionFeatureTrack &feature_track) {
            return feature_track.feature_track.track;
          };

  // Got the casting solution from
  // https://stackoverflow.com/questions/30393285/stdfunction-fails-to-distinguish-overloaded-functions
  typedef std::shared_ptr<vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::StructuredVisionFeatureTrack>> (*funtype)(
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::StructuredVisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *);
  funtype func = vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::StructuredVisionFeatureTrack>::create;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::StructuredVisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *)>
      unbound_callback_creator = func;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      std::vector<vslam_types::SLAMNode> *)>
      callback_creator = std::bind(unbound_callback_creator,
                                   std::placeholders::_1,
                                   feature_retriever,
                                   robot_poses,
                                   std::placeholders::_2);

  LOG(INFO) << "Done with problem setup";

  for (const auto &intrinsics_for_cam : intrinsics) {
    LOG(INFO) << "Intrinsics for " << intrinsics_for_cam.first << ": " << intrinsics_for_cam.second.camera_mat;
  }

  std_msgs::ColorRGBA initial_ellipsoid_color;
  initial_ellipsoid_color.a = 0.5;
  initial_ellipsoid_color.b = 1.0;
  initial_ellipsoid_color.g = 0.5;
  viz->visualizeEllipsoids(
      initial_ests, "initial_ellipsoids", initial_ellipsoid_color);

  LOG(INFO) << "Visualized initial estimates";

  viz->visualizeCameraObservations(robot_poses,
                                   std::nullopt,
                                   std::nullopt,
                                   initial_ests,
                                   std::nullopt,
                                   std::nullopt,
                                   extrinsics,
                                   intrinsics,
                                   images,
                                   observed_corner_locations,
                                   {},
                                   true);

  LOG(INFO) << "Visualizing camera observations";

  bool slam_convergence_result =
      findEllipsoidEstimates(ut_slam_problem,
                             optimizer_params,
                             callback_creator,
                             problem_params,
                             updated_ellipsoid_estimates);

  LOG(INFO) << "Convergence result " << slam_convergence_result;

  std_msgs::ColorRGBA optimized_ellipsoid_color;
  optimized_ellipsoid_color.a = 0.5;
  optimized_ellipsoid_color.r = 1.0;
  viz->visualizeEllipsoids(updated_ellipsoid_estimates,
                           "estimated_ellipsoids",
                           optimized_ellipsoid_color);
  ros::Duration(1).sleep();
  viz->visualizeCameraObservations(robot_poses,
                                   std::nullopt,
                                   std::nullopt,
                                   initial_ests,
                                   updated_ellipsoid_estimates,
                                   std::nullopt,
                                   extrinsics,
                                   intrinsics,
                                   images,
                                   observed_corner_locations,
                                   {},
                                   true);

  LOG(INFO) << "Initial Vs Updated Estimates";
  for (size_t i = 0; i < initial_ests.size(); i++) {
    LOG(INFO) << "Entry " << i;
    vslam_types::EllipsoidEstimate initial_ellipsoid = initial_ests[i];
    vslam_types::EllipsoidEstimate updated_ellipsoid =
        updated_ellipsoid_estimates[i];
    LOG(INFO) << "loc x\t" << initial_ellipsoid.loc.x() << "   \t"
              << updated_ellipsoid.loc.x();
    LOG(INFO) << "loc y\t" << initial_ellipsoid.loc.y() << "   \t"
              << updated_ellipsoid.loc.y();
    LOG(INFO) << "loc z\t" << initial_ellipsoid.loc.z() << "   \t"
              << updated_ellipsoid.loc.z();

    LOG(INFO) << "loc ax\t"
              << initial_ellipsoid.orientation.axis().x() *
                     initial_ellipsoid.orientation.angle()
              << "   \t"
              << updated_ellipsoid.orientation.axis().x() *
                     updated_ellipsoid.orientation.angle();
    LOG(INFO) << "loc ay\t"
              << initial_ellipsoid.orientation.axis().y() *
                     initial_ellipsoid.orientation.angle()
              << "   \t"
              << updated_ellipsoid.orientation.axis().y() *
                     updated_ellipsoid.orientation.angle();
    LOG(INFO) << "loc az\t"
              << initial_ellipsoid.orientation.axis().z() *
                     initial_ellipsoid.orientation.angle()
              << "   \t"
              << updated_ellipsoid.orientation.axis().z() *
                     updated_ellipsoid.orientation.angle();

    LOG(INFO) << "dim x\t" << initial_ellipsoid.ellipsoid_dim.x() << "   \t"
              << updated_ellipsoid.ellipsoid_dim.x();
    LOG(INFO) << "dim y\t" << initial_ellipsoid.ellipsoid_dim.y() << "   \t"
              << updated_ellipsoid.ellipsoid_dim.y();
    LOG(INFO) << "dim z\t" << initial_ellipsoid.ellipsoid_dim.z() << "   \t"
              << updated_ellipsoid.ellipsoid_dim.z();

    LOG(INFO) << "Init difference vs estimated";
    float updated_pose_diff_norm =
        (initial_ellipsoid.loc - updated_ellipsoid.loc).norm();
    float updated_quat_angle_diff =
        Eigen::Quaternionf(initial_ellipsoid.orientation)
            .angularDistance(Eigen::Quaternionf(updated_ellipsoid.orientation));
    Eigen::Vector3f updated_dim_diff =
        initial_ellipsoid.ellipsoid_dim - updated_ellipsoid.ellipsoid_dim;

    LOG(INFO) << "Pose norm: \t" << updated_pose_diff_norm;
    LOG(INFO) << "Quat ang:  \t" << updated_quat_angle_diff;
    LOG(INFO) << "Dim x:     \t" << updated_dim_diff.x();
    LOG(INFO) << "Dim y:     \t" << updated_dim_diff.y();
    LOG(INFO) << "Dim z:     \t" << updated_dim_diff.z();
  }

  return 0;
}