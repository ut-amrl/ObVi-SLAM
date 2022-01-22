//
// Created by amanda on 1/21/22.
//

#include <object_slam_problem_params.h>
#include <synthetic_problem/noise_addition_utils.h>
#include <synthetic_problem/synthetic_problem_generator.h>
#include <vslam_util.h>

using namespace vslam_types;

namespace synthetic_problem {

Eigen::Vector4f getGroundTruthBoundingBoxObservation(
    const vslam_types::RobotPose &gt_robot_pose,
    const vslam_types::EllipsoidEstimate &gt_ellipsoid,
    const vslam_types::CameraIntrinsics &cam_intrinsics,
    const vslam_types::CameraExtrinsics &cam_extrinsics) {

  // TODO make more generic 3d pose rep
  vslam_types::RobotPose extrinsics_pose(
      gt_robot_pose.frame_idx,
      cam_extrinsics.translation,
      Eigen::AngleAxisf(cam_extrinsics.rotation));
  vslam_types::RobotPose cam_pose_in_world =
      vslam_util::combinePoses(gt_robot_pose, extrinsics_pose);
  vslam_types::RobotPose ellipsoid_pose_in_world(
      gt_robot_pose.frame_idx, gt_ellipsoid.loc, gt_ellipsoid.orientation);

  vslam_types::RobotPose ellipsoid_pose_rel_cam =
      vslam_util::getPose2RelativeToPose1(cam_pose_in_world,
                                          ellipsoid_pose_in_world);
  Eigen::AffineCompact3f relative_ellipsoid_pose =
      ellipsoid_pose_rel_cam.RobotToWorldTF();

  Eigen::DiagonalMatrix<float, 4> transformed_ellipsoid_dual(
      Eigen::Vector4f(gt_ellipsoid.ellipsoid_dim.x(),
                      gt_ellipsoid.ellipsoid_dim.y(),
                      gt_ellipsoid.ellipsoid_dim.z(),
                      -1));

  Eigen::Matrix3f g_mat = cam_intrinsics.camera_mat *
                          relative_ellipsoid_pose.matrix() *
                          transformed_ellipsoid_dual *
                          (relative_ellipsoid_pose.matrix().transpose()) *
                          (cam_intrinsics.camera_mat.transpose());

  float u_sqrt_term = sqrt(pow(g_mat(0, 2), 2) - (g_mat(0, 0) * g_mat(2, 2)));
  float u_min = g_mat(0, 2) + u_sqrt_term;
  float u_max = g_mat(0, 2) - u_sqrt_term;
  float v_sqrt_term = sqrt(pow(g_mat(1, 2), 2) - (g_mat(1, 1) * g_mat(2, 2)));
  float v_min = g_mat(1, 2) + v_sqrt_term;
  float v_max = g_mat(1, 2) - v_sqrt_term;

  return Eigen::Vector4f(u_min, u_max, v_min, v_max);
}

bool doesBoundingBoxSatisfyVisibilityParams(
    const Eigen::Vector4f &bounding_box,
    const EllipsoidVisibilityParams &ellipsoid_visibility_params) {
  float min_x = bounding_box(0);
  float max_x = bounding_box(1);
  float min_y = bounding_box(2);
  float max_y = bounding_box(3);

  bool big_enough = (((max_x - min_x) >
           ellipsoid_visibility_params.minimum_bounding_box_size_pixels) &&
          ((max_y - min_y) >
           ellipsoid_visibility_params.minimum_bounding_box_size_pixels));

  bool in_frame = min_x >= 0;
  in_frame = in_frame && (max_x >= 0);
  in_frame = in_frame && (min_x <= ellipsoid_visibility_params.max_pixel_x);
  in_frame = in_frame && (max_x <= ellipsoid_visibility_params.max_pixel_x);
  in_frame = in_frame && (min_y >= 0);
  in_frame = in_frame && (max_y >= 0);
  in_frame = in_frame && (min_y <= ellipsoid_visibility_params.max_pixel_y);
  in_frame = in_frame && (max_y <= ellipsoid_visibility_params.max_pixel_y);

  return in_frame && big_enough;
}

std::vector<vslam_types::ObjectImageBoundingBoxDetection>
generateBoundingBoxDetectionsFromGroundTruthEllipsoidsAndPoses(
    const std::vector<vslam_types::EllipsoidEstimate> &ground_truth_ellipsoids,
    const std::vector<vslam_types::RobotPose> &ground_truth_robot_poses,
    const std::unordered_map<vslam_types::CameraId,
                             vslam_types::CameraIntrinsics> &intrinsics,
    const std::unordered_map<vslam_types::CameraId,
                             vslam_types::CameraExtrinsics> &extrinsics,
    const Eigen::Vector4f &bounding_box_std_devs,
    const double &bounding_box_detection_success_rate,
    const EllipsoidVisibilityParams &ellipsoid_visibility_params,
    util_random::Random &random_gen) {

  std::vector<vslam_types::ObjectImageBoundingBoxDetection>
      detected_bounding_boxes;
  for (uint64_t robot_pose_num = 0;
       robot_pose_num < ground_truth_robot_poses.size();
       robot_pose_num++) {
    vslam_types::RobotPose robot_pose =
        ground_truth_robot_poses[robot_pose_num];
    for (uint64_t ellipsoid_num = 0;
         ellipsoid_num < ground_truth_ellipsoids.size();
         ellipsoid_num++) {
      vslam_types::EllipsoidEstimate gt_ellipsoid =
          ground_truth_ellipsoids[ellipsoid_num];
      for (const auto &camera_id_and_intrinsics : intrinsics) {
        if (random_gen.UniformRandom() > bounding_box_detection_success_rate) {
          // This detection "failed" so we don't add a bounding box for it
          continue;
        }
        vslam_types::CameraId cam_id = camera_id_and_intrinsics.first;
        vslam_types::CameraIntrinsics curr_cam_intrinsics =
            camera_id_and_intrinsics.second;
        vslam_types::CameraExtrinsics curr_cam_extrinsics =
            extrinsics.at(cam_id);

        Eigen::Vector4f noiseless_predicted_bounding_box =
            getGroundTruthBoundingBoxObservation(robot_pose,
                                                 gt_ellipsoid,
                                                 curr_cam_intrinsics,
                                                 curr_cam_extrinsics);
        if (!doesBoundingBoxSatisfyVisibilityParams(
                noiseless_predicted_bounding_box,
                ellipsoid_visibility_params)) {
          continue;
        }

        Eigen::Vector4f noisy_bounding_box =
            addNoiseToVectorDiagonalCov(noiseless_predicted_bounding_box,
                                        bounding_box_std_devs,
                                        random_gen);
        if (!doesBoundingBoxSatisfyVisibilityParams(
                noisy_bounding_box, ellipsoid_visibility_params)) {
          continue;
        }

        vslam_types::ObjectImageBoundingBoxDetection bounding_box_detection;
        bounding_box_detection.frame_idx = robot_pose_num;
        bounding_box_detection.ellipsoid_idx = ellipsoid_num;
        bounding_box_detection.camera_id = cam_id;
        bounding_box_detection.semantic_class = gt_ellipsoid.semantic_class;

        std::pair<Eigen::Vector2f, Eigen::Vector2f> noisy_pixels =
            std::make_pair(
                Eigen::Vector2f(noisy_bounding_box(0), noisy_bounding_box(2)),
                Eigen::Vector2f(noisy_bounding_box(1), noisy_bounding_box(3)));
        bounding_box_detection.pixel_corner_locations = noisy_pixels;

        detected_bounding_boxes.emplace_back(bounding_box_detection);
      }
    }
  }
  return detected_bounding_boxes;
}

template <typename FeatureTrackType>
std::pair<UTObjectSLAMProblem<FeatureTrackType>,
          vslam_solver::ObjectSlamProblemParams>
createEllipsoidOnlySyntheticProblemFromEllipsoidsAndCameraPoses(
    const std::vector<vslam_types::EllipsoidEstimate> &ground_truth_ellipsoids,
    const std::vector<RobotPose> &ground_truth_robot_poses,
    const std::unordered_map<CameraId, CameraIntrinsics> &intrinsics,
    const std::unordered_map<CameraId, CameraExtrinsics> &extrinsics,
    const std::unordered_map<std::string,
                             std::pair<Eigen::Vector3f, Eigen::Vector3f>>
        &shape_mean_and_std_devs_by_semantic_class,
    const Eigen::Vector4f &bounding_box_std_devs,
    const Eigen::Matrix<float, 6, 1> &ellipsoid_pose_estimate_noise,
    const Eigen::Matrix<float, 6, 1> &robot_pose_std_devs,
    const double &bounding_box_detection_success_rate,
    const EllipsoidVisibilityParams &ellipsoid_visibility_params,
    util_random::Random &random_gen) {
  std::vector<ObjectImageBoundingBoxDetection> bounding_boxes =
      generateBoundingBoxDetectionsFromGroundTruthEllipsoidsAndPoses(
          ground_truth_ellipsoids,
          ground_truth_robot_poses,
          intrinsics,
          extrinsics,
          bounding_box_std_devs,
          bounding_box_detection_success_rate,
          ellipsoid_visibility_params,
          random_gen);

  std::vector<RobotPose> noisy_robot_poses =
      synthetic_problem::addOdomNoiseToTrajectory(
          ground_truth_robot_poses, robot_pose_std_devs, random_gen);

  std::vector<EllipsoidEstimate> noisy_ellipsoid_estimates;
  for (const EllipsoidEstimate &ellipsoid_est : ground_truth_ellipsoids) {
    Eigen::Matrix<float, 9, 1> ellipsoid_std_devs;
    ellipsoid_std_devs.topRows(6) = ellipsoid_pose_estimate_noise;
    std::pair<Eigen::Vector3f, Eigen::Vector3f>
        shape_prior_for_ellipsoid_class =
            shape_mean_and_std_devs_by_semantic_class.at(
                ellipsoid_est.semantic_class);

    ellipsoid_std_devs.bottomRows(3) = shape_prior_for_ellipsoid_class.second;
    noisy_ellipsoid_estimates.emplace_back(addNoiseToEllipsoidEstimate(
        ellipsoid_est, ellipsoid_std_devs, random_gen));
  }

  UTObjectSLAMProblem<StructuredVisionFeatureTrack> slam_problem;
  slam_problem.ellipsoid_estimates = noisy_ellipsoid_estimates;
  slam_problem.robot_poses = noisy_robot_poses;
  slam_problem.bounding_boxes = bounding_boxes;
  slam_problem.camera_instrinsics_by_camera = intrinsics;
  slam_problem.camera_extrinsics_by_camera = extrinsics;

  vslam_solver::ObjectSlamProblemParams object_slam_params;
  for (const auto &mean_and_std_devs_with_semantic_class :
       shape_mean_and_std_devs_by_semantic_class) {
    object_slam_params.semantic_shape_prior_params
        .mean_and_cov_by_semantic_class[mean_and_std_devs_with_semantic_class
                                            .first] =
        std::make_pair(
            mean_and_std_devs_with_semantic_class.second.first,
            vslam_util::createDiagCovFromStdDevs(
                mean_and_std_devs_with_semantic_class.second.second));
  }
  object_slam_params.ellipsoid_bounding_box_constraint_params
      .bounding_box_covariance =
      vslam_util::createDiagCovFromStdDevs(bounding_box_std_devs);

  return std::make_pair(slam_problem, object_slam_params);
}

template std::pair<UTObjectSLAMProblem<StructuredVisionFeatureTrack>,
                   vslam_solver::ObjectSlamProblemParams>
createEllipsoidOnlySyntheticProblemFromEllipsoidsAndCameraPoses(
    const std::vector<vslam_types::EllipsoidEstimate> &ground_truth_ellipsoids,
    const std::vector<RobotPose> &ground_truth_robot_poses,
    const std::unordered_map<CameraId, CameraIntrinsics> &intrinsics,
    const std::unordered_map<CameraId, CameraExtrinsics> &extrinsics,
    const std::unordered_map<std::string,
                             std::pair<Eigen::Vector3f, Eigen::Vector3f>>
        &shape_mean_and_std_devs_by_semantic_class,
    const Eigen::Vector4f &bounding_box_std_devs,
    const Eigen::Matrix<float, 6, 1> &ellipsoid_pose_estimate_noise,
    const Eigen::Matrix<float, 6, 1> &robot_pose_std_devs,
    const double &bounding_box_detection_success_rate,
    const EllipsoidVisibilityParams &ellipsoid_visibility_params,
    util_random::Random &random_gen);
}  // namespace synthetic_problem
