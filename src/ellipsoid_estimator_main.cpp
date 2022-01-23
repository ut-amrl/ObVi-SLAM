#include <gflags/gflags.h>
#include <glog/logging.h>
#include <object_slam_backend_solver.h>
#include <shape_prior_factor.h>
#include <synthetic_problem/synthetic_problem_construction_utils.h>
#include <synthetic_problem/synthetic_problem_generator.h>
#include <visual_slam_ceres_visualization_callback.h>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::string car_semantic_class_str = "car";

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  std::vector<vslam_types::EllipsoidEstimate> ground_truth_ellipsoids;
  vslam_types::EllipsoidEstimate ellipsoid_est(
      Eigen::Vector3f(0, 0, 0),
      Eigen::AngleAxisf(0, Eigen::Vector3f(1, 0, 0)),
      Eigen::Vector3f(3, 4, 5),
      car_semantic_class_str,
      0);
  ground_truth_ellipsoids.emplace_back(ellipsoid_est);

  std::vector<vslam_types::RobotPose> ground_truth_robot_poses;
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size() - 1,
      Eigen::Vector3f(20, 0, 0),
      synthetic_problem::createAngleAxisFromYaw(M_PI_2)));
  ground_truth_robot_poses.emplace_back(
      vslam_types::RobotPose(ground_truth_robot_poses.size() - 1,
                             Eigen::Vector3f(0, 20, 0),
                             synthetic_problem::createAngleAxisFromYaw(M_PI)));
  ground_truth_robot_poses.emplace_back(vslam_types::RobotPose(
      ground_truth_robot_poses.size() - 1,
      Eigen::Vector3f(-20, 0, 0),
      synthetic_problem::createAngleAxisFromYaw(-M_PI_2)));
  ground_truth_robot_poses.emplace_back(
      vslam_types::RobotPose(ground_truth_robot_poses.size() - 1,
                             Eigen::Vector3f(0, -20, 0),
                             synthetic_problem::createAngleAxisFromYaw(0)));

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
  intrinsics[0] = intrinsics_for_both;
  intrinsics[1] = intrinsics_for_both;

  Eigen::Vector4f bounding_box_std_devs;  // TODO
  bounding_box_std_devs(0) = 1e-5;
  bounding_box_std_devs(1) = 1e-5;
  bounding_box_std_devs(2) = 1e-5;
  bounding_box_std_devs(3) = 1e-5;

  Eigen::Matrix<float, 6, 1> ellipsoid_pose_estimate_noise;  // TODO
  Eigen::Matrix<float, 6, 1> robot_pose_std_devs;            // TODO

  std::unordered_map<std::string, std::pair<Eigen::Vector3f, Eigen::Vector3f>>
      shape_mean_and_std_devs_by_semantic_class;
  shape_mean_and_std_devs_by_semantic_class[car_semantic_class_str] =
      std::make_pair(Eigen::Vector3f(2.8, 4.3, 5.1),
                     Eigen::Vector3f(0.3, 0.4, 0.25));
  std::unordered_map<vslam_types::CameraId, vslam_types::CameraExtrinsics>
      extrinsics;
  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  Eigen::Quaternionf extrinsics_orientation =
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse();  // [0 -1 0; 0 0 -1; 1 0 0]^-1
  LOG(INFO) << "Extrinsics orientation " << extrinsics_orientation.w() << ", " << extrinsics_orientation.x() << ", " << extrinsics_orientation.y() << ", " << extrinsics_orientation.z();
  extrinsics[0] = {Eigen::Vector3f(0, 0, 0), extrinsics_orientation};
  // TODO Is translation before or after rotation? (i.e. should we use camera
  // frame axes or robot frame)
  extrinsics[1] = {Eigen::Vector3f(0, -0.2, 0), extrinsics_orientation};
  double bounding_box_detection_success_rate = 0.8;
  synthetic_problem::EllipsoidVisibilityParams ellipsoid_visibility_params;
  util_random::Random random_gen;

  std::pair<vslam_types::UTObjectSLAMProblem<
                vslam_types::StructuredVisionFeatureTrack>,
            vslam_solver::ObjectSlamProblemParams>
      slam_problem_and_params = synthetic_problem::
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

  LOG(INFO) << "Created problem with " << slam_problem_and_params.first.bounding_boxes.size() << " bounding box detections";

  vslam_solver::SLAMSolverOptimizerParams optimizer_params;
  vslam_solver::StructuredObjectSlamProblemParams problem_params;
  problem_params.object_slam_params = slam_problem_and_params.second;

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

  bool slam_convergence_result =
      findEllipsoidEstimates(slam_problem_and_params.first,
                             optimizer_params,
                             callback_creator,
                             problem_params,
                             updated_ellipsoid_estimates);

  LOG(INFO) << "Convergence result " << slam_convergence_result;

  return 0;
}