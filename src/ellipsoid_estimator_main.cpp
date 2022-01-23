#include <bounding_box_factor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <object_slam_backend_solver.h>
#include <shape_prior_factor.h>
#include <synthetic_problem/synthetic_problem_generator.h>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::string car_semantic_class_str = "car";

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // TODO fill in with ellipsoid estimation code (step 2)

  std::vector<vslam_types::EllipsoidEstimate> ground_truth_ellipsoids; 
  vslam_types::EllipsoidEstimate ellipsoid_est(
      Eigen::Vector3f(0, 0, 0),
      Eigen::AngleAxisf(0, Eigen::Vector3f(1, 0, 0)),
      Eigen::Vector3f(3, 4, 5),
      car_semantic_class_str,
      0);
  ground_truth_ellipsoids.emplace_back(ellipsoid_est);

  std::vector<vslam_types::RobotPose> ground_truth_robot_poses;  // TODO
  std::unordered_map<vslam_types::CameraId, vslam_types::CameraIntrinsics>
      intrinsics;                                            // TODO
  Eigen::Vector4f bounding_box_std_devs;                     // TODO
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
      slam_problem_and_params =
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

  return 0;
}