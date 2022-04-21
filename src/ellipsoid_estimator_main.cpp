#include <gflags/gflags.h>
#include <glog/logging.h>
#include <object_slam_backend_solver.h>
#include <shape_prior_factor.h>
#include <synthetic_problem/noise_addition_utils.h>
#include <synthetic_problem/synthetic_problem_construction_utils.h>
#include <synthetic_problem/synthetic_problem_generator.h>
#include <visual_slam_ceres_visualization_callback.h>
#include <visualization/ros_visualization.h>

DEFINE_string(param_prefix, "", "param_prefix");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  std::string car_semantic_class_str = "car";

  std::unordered_map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>>
      shape_mean_and_std_devs_by_semantic_class;
//  shape_mean_and_std_devs_by_semantic_class[car_semantic_class_str] =
//      std::make_pair(Eigen::Vector3f(4.885, 1.84, 1.445),
//                     Eigen::Vector3f(1.0, 0.6, 0.5));
  shape_mean_and_std_devs_by_semantic_class[car_semantic_class_str] =
      std::make_pair(Eigen::Vector3f(4.885, 1.84, 1.445) / 2,
                     Eigen::Vector3f(1.0, 0.6, 0.5) / 2);


  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }
  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, node_prefix + "ellipsoid_estimator_est");
  ros::NodeHandle n;

  util_random::Random random_gen;

  std::shared_ptr<vslam_viz::RosVisualization> viz =
      std::make_shared<vslam_viz::RosVisualization>(n, param_prefix);

  std::vector<vslam_types::EllipsoidEstimate> ground_truth_ellipsoids;
  Eigen::Vector3f ellipsoid_loc(0, 0, 0);
  Eigen::AngleAxisf ellipsoid_orientation(0, Eigen::Vector3f(1, 0, 0));
  Eigen::Vector3f ellipsoid_dim =
      synthetic_problem::addNoiseToVectorDiagonalCov(
          shape_mean_and_std_devs_by_semantic_class[car_semantic_class_str]
              .first,
          shape_mean_and_std_devs_by_semantic_class[car_semantic_class_str]
              .second,
          random_gen);

  vslam_types::EllipsoidEstimate ellipsoid_est(ellipsoid_loc,
                                               ellipsoid_orientation,
                                               ellipsoid_dim,
                                               car_semantic_class_str,
                                               ground_truth_ellipsoids.size());
  ground_truth_ellipsoids.emplace_back(ellipsoid_est);

  Eigen::Vector3f ellipsoid_loc_2(2.5, 1.5, 0);
  Eigen::AngleAxisf ellipsoid_orientation_2(M_PI_4, Eigen::Vector3f(1, 0, 0));
  Eigen::Vector3f ellipsoid_dim_2 =
      synthetic_problem::addNoiseToVectorDiagonalCov(
          shape_mean_and_std_devs_by_semantic_class[car_semantic_class_str]
              .first,
          shape_mean_and_std_devs_by_semantic_class[car_semantic_class_str]
              .second,
          random_gen);

  vslam_types::EllipsoidEstimate ellipsoid_est_2(
      ellipsoid_loc_2,
      ellipsoid_orientation_2,
      ellipsoid_dim_2,
      car_semantic_class_str,
      ground_truth_ellipsoids.size());
  ground_truth_ellipsoids.emplace_back(ellipsoid_est_2);

  float viewing_radius = 10;
  std::vector<vslam_types::RobotPose> ground_truth_robot_poses;
  ground_truth_robot_poses.emplace_back(
      vslam_types::RobotPose(ground_truth_robot_poses.size(),
                             Eigen::Vector3f(viewing_radius, 0, 0),
                             synthetic_problem::createAngleAxisFromYaw(M_PI)));
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size(),
      Eigen::Vector3f(viewing_radius / M_SQRT2, viewing_radius / M_SQRT2, 0),
      synthetic_problem::createAngleAxisFromYaw(-3 * M_PI_4)));
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size(),
      Eigen::Vector3f(0, viewing_radius, 0),
      synthetic_problem::createAngleAxisFromYaw(-M_PI_2)));
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size(),
      Eigen::Vector3f(-viewing_radius / M_SQRT2, viewing_radius / M_SQRT2, 0),
      synthetic_problem::createAngleAxisFromYaw(-M_PI_4)));
  ground_truth_robot_poses.emplace_back(
      vslam_types::RobotPose(ground_truth_robot_poses.size(),
                             Eigen::Vector3f(-viewing_radius, 0, 0),
                             synthetic_problem::createAngleAxisFromYaw(0)));
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size(),
      Eigen::Vector3f(-viewing_radius / M_SQRT2, -viewing_radius / M_SQRT2, 0),
      synthetic_problem::createAngleAxisFromYaw(M_PI_4)));
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size(),
      Eigen::Vector3f(0, -viewing_radius, 0),
      synthetic_problem::createAngleAxisFromYaw(M_PI_2)));
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size(),
      Eigen::Vector3f(viewing_radius / M_SQRT2, -viewing_radius / M_SQRT2, 0),
      synthetic_problem::createAngleAxisFromYaw(3 * M_PI_4)));

  std::unordered_map<vslam_types::CameraId, vslam_types::CameraIntrinsics>
      intrinsics;
  vslam_types::CameraIntrinsics intrinsics_for_both;

  float fx = 585;  // TODO consider changing
  float fy = 585;
  float cx = 320;
  float cy = 240;
  intrinsics_for_both.camera_mat.setIdentity();
  intrinsics_for_both.camera_mat(0, 0) = fx;
  intrinsics_for_both.camera_mat(1, 1) = fy;
  intrinsics_for_both.camera_mat(0, 2) = cx;
  intrinsics_for_both.camera_mat(1, 2) = cy;
  intrinsics_for_both.image_width = intrinsics_for_both.camera_mat(0, 2) * 2;
  intrinsics_for_both.image_height = intrinsics_for_both.camera_mat(1, 2) * 2;
  intrinsics[0] = intrinsics_for_both;
  intrinsics[1] = intrinsics_for_both;

  Eigen::Vector4f bounding_box_std_devs;  // TODO
                                          //  bounding_box_std_devs(0) = 1e-5;
                                          //  bounding_box_std_devs(1) = 1e-5;
                                          //  bounding_box_std_devs(2) = 1e-5;
                                          //  bounding_box_std_devs(3) = 1e-5;
                                          //    bounding_box_std_devs(0) = 50;
                                          //    bounding_box_std_devs(1) = 50;
                                          //    bounding_box_std_devs(2) = 50;
                                          //    bounding_box_std_devs(3) = 50;

  bounding_box_std_devs(0) = 15;
  bounding_box_std_devs(1) = 15;
  bounding_box_std_devs(2) = 15;
  bounding_box_std_devs(3) = 15;

  //  Eigen::Matrix<float, 6, 1> ellipsoid_pose_estimate_noise =
  //  Eigen::Matrix<float, 6, 1>::Zero();  // TODO
  Eigen::Matrix<float, 6, 1> ellipsoid_pose_estimate_noise;
  ellipsoid_pose_estimate_noise << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
  Eigen::Matrix<float, 6, 1> robot_pose_std_devs =
      Eigen::Matrix<float, 6, 1>::Zero();  // TODO

  std::unordered_map<vslam_types::CameraId, vslam_types::CameraExtrinsics>
      extrinsics;
  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  Eigen::Quaternionf extrinsics_orientation =
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse();  // [0 -1 0; 0 0 -1; 1 0 0]^-1
  extrinsics[0] = {Eigen::Vector3f(0, 0, 0), extrinsics_orientation};
  // TODO Is translation before or after rotation? (i.e. should we use camera
  // frame axes or robot frame)
  extrinsics[1] = {Eigen::Vector3f(0, -0.2, 0), extrinsics_orientation};
  double bounding_box_detection_success_rate = 0.8;  // TODO change back to this
  bounding_box_detection_success_rate = 1;
  synthetic_problem::EllipsoidVisibilityParams ellipsoid_visibility_params;

  synthetic_problem::SyntheticProblemGeneratedData<
      vslam_types::StructuredVisionFeatureTrack>
      synthetic_prob_data = synthetic_problem::
          createEllipsoidOnlySyntheticProblemFromEllipsoidsAndCameraPoses<
              vslam_types::StructuredVisionFeatureTrack>(
              ground_truth_ellipsoids,
              ground_truth_robot_poses,
              intrinsics,
              extrinsics,
              shape_mean_and_std_devs_by_semantic_class,
              bounding_box_std_devs,
              ellipsoid_pose_estimate_noise,
              robot_pose_std_devs,
              bounding_box_detection_success_rate,
              ellipsoid_visibility_params,
              random_gen);

  LOG(INFO) << "Created problem with "
            << synthetic_prob_data.created_slam_problem.bounding_boxes.size()
            << " bounding box detections";

  vslam_solver::SLAMSolverOptimizerParams optimizer_params;
  vslam_solver::StructuredObjectSlamProblemParams problem_params;
  problem_params.object_slam_params =
      synthetic_prob_data.object_slam_problem_params;

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
                                   ground_truth_robot_poses,
                                   std::placeholders::_2);

  std_msgs::ColorRGBA ellipsoid_color;
  ellipsoid_color.a = 1.0;
  ellipsoid_color.g = 1.0;
  viz->visualizeEllipsoids(
      ground_truth_ellipsoids, "ground_truth_ellipsoids", ellipsoid_color);

  ros::Duration(2).sleep();
  //  viz->visualizeTrajectoryAndEllipsoidsWithTf(ground_truth_robot_poses,
  //                                              ground_truth_ellipsoids,
  //                                              extrinsics,
  //                                              intrinsics);
  ros::Duration(2).sleep();

  std_msgs::ColorRGBA initial_ellipsoid_color;
  initial_ellipsoid_color.a = 0.5;
  initial_ellipsoid_color.b = 1.0;
  initial_ellipsoid_color.g = 0.5;
  viz->visualizeEllipsoids(synthetic_prob_data.initial_ellipsoid_estimates,
                           "initial_ellipsoids",
                           initial_ellipsoid_color);

  viz->visualizeCameraObservations(
      ground_truth_robot_poses,
      std::nullopt,
      std::nullopt,
      synthetic_prob_data.initial_ellipsoid_estimates,
      std::nullopt,
      ground_truth_ellipsoids,
      extrinsics,
      intrinsics,
      {},
      synthetic_prob_data.observed_corner_locations,
      synthetic_prob_data.gt_corner_locations,
      true);

  bool slam_convergence_result =
      findEllipsoidEstimates(synthetic_prob_data.created_slam_problem,
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
  viz->visualizeCameraObservations(
      ground_truth_robot_poses,
      std::nullopt,
      std::nullopt,
      synthetic_prob_data.initial_ellipsoid_estimates,
      updated_ellipsoid_estimates,
      ground_truth_ellipsoids,
      extrinsics,
      intrinsics,
      {},
      synthetic_prob_data.observed_corner_locations,
      synthetic_prob_data.gt_corner_locations,
      true);

  LOG(INFO) << "Ground truth vs Initial Vs Updated Estimates";
  for (size_t i = 0; i < ground_truth_ellipsoids.size(); i++) {
    LOG(INFO) << "Entry " << i;
    vslam_types::EllipsoidEstimate gt_ellipsoid = ground_truth_ellipsoids[i];
    vslam_types::EllipsoidEstimate initial_ellipsoid =
        synthetic_prob_data.initial_ellipsoid_estimates[i];
    vslam_types::EllipsoidEstimate updated_ellipsoid =
        updated_ellipsoid_estimates[i];
    LOG(INFO) << "loc x\t" << gt_ellipsoid.loc.x() << "   \t"
              << initial_ellipsoid.loc.x() << "   \t"
              << updated_ellipsoid.loc.x();
    LOG(INFO) << "loc y\t" << gt_ellipsoid.loc.y() << "   \t"
              << initial_ellipsoid.loc.y() << "   \t"
              << updated_ellipsoid.loc.y();
    LOG(INFO) << "loc z\t" << gt_ellipsoid.loc.z() << "   \t"
              << initial_ellipsoid.loc.z() << "   \t"
              << updated_ellipsoid.loc.z();

    LOG(INFO) << "loc ax\t"
              << gt_ellipsoid.orientation.axis().x() *
                     gt_ellipsoid.orientation.angle()
              << "   \t"
              << initial_ellipsoid.orientation.axis().x() *
                     initial_ellipsoid.orientation.angle()
              << "   \t"
              << updated_ellipsoid.orientation.axis().x() *
                     updated_ellipsoid.orientation.angle();
    LOG(INFO) << "loc ay\t"
              << gt_ellipsoid.orientation.axis().y() *
                     gt_ellipsoid.orientation.angle()
              << "   \t"
              << initial_ellipsoid.orientation.axis().y() *
                     initial_ellipsoid.orientation.angle()
              << "   \t"
              << updated_ellipsoid.orientation.axis().y() *
                     updated_ellipsoid.orientation.angle();
    LOG(INFO) << "loc az\t"
              << gt_ellipsoid.orientation.axis().z() *
                     gt_ellipsoid.orientation.angle()
              << "   \t"
              << initial_ellipsoid.orientation.axis().z() *
                     initial_ellipsoid.orientation.angle()
              << "   \t"
              << updated_ellipsoid.orientation.axis().z() *
                     updated_ellipsoid.orientation.angle();

    LOG(INFO) << "dim x\t" << gt_ellipsoid.ellipsoid_dim.x() << "   \t"
              << initial_ellipsoid.ellipsoid_dim.x() << "   \t"
              << updated_ellipsoid.ellipsoid_dim.x();
    LOG(INFO) << "dim y\t" << gt_ellipsoid.ellipsoid_dim.y() << "   \t"
              << initial_ellipsoid.ellipsoid_dim.y() << "   \t"
              << updated_ellipsoid.ellipsoid_dim.y();
    LOG(INFO) << "dim z\t" << gt_ellipsoid.ellipsoid_dim.z() << "   \t"
              << initial_ellipsoid.ellipsoid_dim.z() << "   \t"
              << updated_ellipsoid.ellipsoid_dim.z();

    LOG(INFO) << "Init difference vs estimated";
    float init_pose_diff_norm =
        (gt_ellipsoid.loc - initial_ellipsoid.loc).norm();
    float updated_pose_diff_norm =
        (gt_ellipsoid.loc - updated_ellipsoid.loc).norm();
    float init_quat_angle_diff =
        Eigen::Quaternionf(gt_ellipsoid.orientation)
            .angularDistance(Eigen::Quaternionf(initial_ellipsoid.orientation));
    float updated_quat_angle_diff =
        Eigen::Quaternionf(gt_ellipsoid.orientation)
            .angularDistance(Eigen::Quaternionf(updated_ellipsoid.orientation));
    Eigen::Vector3f init_dim_diff =
        gt_ellipsoid.ellipsoid_dim - initial_ellipsoid.ellipsoid_dim;
    Eigen::Vector3f updated_dim_diff =
        gt_ellipsoid.ellipsoid_dim - updated_ellipsoid.ellipsoid_dim;

    LOG(INFO) << "Pose norm: \t" << init_pose_diff_norm << "  \t"
              << updated_pose_diff_norm;
    LOG(INFO) << "Quat ang:  \t" << init_quat_angle_diff << "  \t"
              << updated_quat_angle_diff;
    LOG(INFO) << "Dim x:     \t" << init_dim_diff.x() << "  \t"
              << updated_dim_diff.x();
    LOG(INFO) << "Dim y:     \t" << init_dim_diff.y() << "  \t"
              << updated_dim_diff.y();
    LOG(INFO) << "Dim z:     \t" << init_dim_diff.z() << "  \t"
              << updated_dim_diff.z();
  }

  return 0;
}