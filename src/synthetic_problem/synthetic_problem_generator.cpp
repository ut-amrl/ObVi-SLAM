//
// Created by amanda on 1/21/22.
//

#include <glog/logging.h>
#include <object_slam_problem_params.h>
#include <synthetic_problem/noise_addition_utils.h>
#include <synthetic_problem/synthetic_problem_generator.h>
#include <vslam_type_conversion_util.h>
#include <vslam_types_math_util.h>

using namespace vslam_types;

namespace synthetic_problem {

const float kMinBoundingBoxStdDev = 1e-4;

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
  //  LOG(INFO) << "Cam transl in world "
  //            << cam_pose_in_world.RobotToWorldTF().translation();
  //  LOG(INFO) << "Cam rot mat in world "
  //            << cam_pose_in_world.RobotToWorldTF().linear();
  vslam_types::RobotPose ellipsoid_pose_in_world(
      gt_robot_pose.frame_idx, gt_ellipsoid.loc, gt_ellipsoid.orientation);

  vslam_types::RobotPose ellipsoid_pose_rel_cam =
      vslam_util::getPose2RelativeToPose1(cam_pose_in_world,
                                          ellipsoid_pose_in_world);

  Eigen::Vector3f ellipsoid_extrema_neg_x_ellipsoid_frame(
      -gt_ellipsoid.ellipsoid_dim.x() / 2, 0, 0);
  Eigen::Vector3f ellipsoid_extrema_pos_x_ellipsoid_frame =
      -ellipsoid_extrema_neg_x_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_extrema_neg_y_ellipsoid_frame(
      0, -gt_ellipsoid.ellipsoid_dim.y() / 2, 0);
  Eigen::Vector3f ellipsoid_extrema_pos_y_ellipsoid_frame =
      -ellipsoid_extrema_neg_y_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_extrema_neg_z_ellipsoid_frame(
      0, 0, -gt_ellipsoid.ellipsoid_dim.z() / 2);
  Eigen::Vector3f ellipsoid_extrema_pos_z_ellipsoid_frame =
      -ellipsoid_extrema_neg_z_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_center_ellipsoid_frame(0, 0, 0);

  //  vslam_types::RobotPose cam_pose_rel_ellispoid =
  //      vslam_util::getPose2RelativeToPose1(ellipsoid_pose_in_world,
  //                                          cam_pose_in_world);

  //  LOG(INFO) << "Ellipsoid translation rel to cam " <<
  //  ellipsoid_pose_rel_cam.loc; Eigen::AffineCompact3f relative_ellipsoid_pose
  //  =
  //      ellipsoid_pose_rel_cam.RobotToWorldTF();

  // Pose of the ellipsoid relative to the camera
  Eigen::AffineCompact3f relative_cam_pose =
      ellipsoid_pose_rel_cam.RobotToWorldTF();

  Eigen::Vector3f ellipsoid_extrema_neg_x_cam_frame =
      relative_cam_pose * ellipsoid_extrema_neg_x_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_extrema_pos_x_cam_frame =
      relative_cam_pose * ellipsoid_extrema_pos_x_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_extrema_neg_y_cam_frame =
      relative_cam_pose * ellipsoid_extrema_neg_y_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_extrema_pos_y_cam_frame =
      relative_cam_pose * ellipsoid_extrema_pos_y_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_extrema_neg_z_cam_frame =
      relative_cam_pose * ellipsoid_extrema_neg_z_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_extrema_pos_z_cam_frame =
      relative_cam_pose * ellipsoid_extrema_pos_z_ellipsoid_frame;
  Eigen::Vector3f ellipsoid_center_cam_frame =
      relative_cam_pose * ellipsoid_center_ellipsoid_frame;

  //  LOG(INFO) << "Extrema rel camera";
  //  LOG(INFO) << ellipsoid_extrema_neg_x_cam_frame;
  //  LOG(INFO) << ellipsoid_extrema_pos_x_cam_frame;
  //  LOG(INFO) << ellipsoid_extrema_neg_y_cam_frame;
  //  LOG(INFO) << ellipsoid_extrema_pos_y_cam_frame;
  //  LOG(INFO) << ellipsoid_extrema_neg_z_cam_frame;
  //  LOG(INFO) << ellipsoid_extrema_pos_z_cam_frame;
  //  LOG(INFO) << ellipsoid_center_cam_frame;

  Eigen::Vector3f ellipsoid_extrema_neg_x_pixel =
      cam_intrinsics.camera_mat * ellipsoid_extrema_neg_x_cam_frame;
  Eigen::Vector3f ellipsoid_extrema_pos_x_pixel =
      cam_intrinsics.camera_mat * ellipsoid_extrema_pos_x_cam_frame;
  Eigen::Vector3f ellipsoid_extrema_neg_y_pixel =
      cam_intrinsics.camera_mat * ellipsoid_extrema_neg_y_cam_frame;
  Eigen::Vector3f ellipsoid_extrema_pos_y_pixel =
      cam_intrinsics.camera_mat * ellipsoid_extrema_pos_y_cam_frame;
  Eigen::Vector3f ellipsoid_extrema_neg_z_pixel =
      cam_intrinsics.camera_mat * ellipsoid_extrema_neg_z_cam_frame;
  Eigen::Vector3f ellipsoid_extrema_pos_z_pixel =
      cam_intrinsics.camera_mat * ellipsoid_extrema_pos_z_cam_frame;
  Eigen::Vector3f ellipsoid_center_pixel =
      cam_intrinsics.camera_mat * ellipsoid_center_cam_frame;

  //  LOG(INFO) << "Pixels unscaled";
  //  LOG(INFO) << ellipsoid_extrema_neg_x_pixel;
  //  LOG(INFO) << ellipsoid_extrema_pos_x_pixel;
  //  LOG(INFO) << ellipsoid_extrema_neg_y_pixel;
  //  LOG(INFO) << ellipsoid_extrema_pos_y_pixel;
  //  LOG(INFO) << ellipsoid_extrema_neg_z_pixel;
  //  LOG(INFO) << ellipsoid_extrema_pos_z_pixel;
  //  LOG(INFO) << ellipsoid_center_pixel;
  //
  //  LOG(INFO) << "Pixels scaled";
  //  LOG(INFO) << ellipsoid_extrema_neg_x_pixel /
  //  ellipsoid_extrema_neg_x_pixel.z(); LOG(INFO) <<
  //  ellipsoid_extrema_pos_x_pixel / ellipsoid_extrema_pos_x_pixel.z();
  //  LOG(INFO) << ellipsoid_extrema_neg_y_pixel /
  //  ellipsoid_extrema_neg_y_pixel.z(); LOG(INFO) <<
  //  ellipsoid_extrema_pos_y_pixel / ellipsoid_extrema_pos_y_pixel.z();
  //  LOG(INFO) << ellipsoid_extrema_neg_z_pixel /
  //  ellipsoid_extrema_neg_z_pixel.z(); LOG(INFO) <<
  //  ellipsoid_extrema_pos_z_pixel / ellipsoid_extrema_pos_z_pixel.z();
  //  LOG(INFO) << ellipsoid_center_pixel / ellipsoid_center_pixel.z();
  //
  //    LOG(INFO) << "Ellipsoid translation rel to cam " <<
  //    relative_ellipsoid_pose.translation();
  //  LOG(INFO) << "Cam transl rel ellipsoid " <<
  //  relative_cam_pose.translation(); LOG(INFO) << "Cam rot rel ellipsoid" <<
  //  relative_cam_pose.linear();

  Eigen::DiagonalMatrix<float, 4> transformed_ellipsoid_dual(
      Eigen::Vector4f(pow(gt_ellipsoid.ellipsoid_dim.x() / 2, 2),
                      pow(gt_ellipsoid.ellipsoid_dim.y() / 2, 2),
                      pow(gt_ellipsoid.ellipsoid_dim.z() / 2, 2),
                      -1));

  //  LOG(INFO) << "Ellipsoid rep: " <<
  //  transformed_ellipsoid_dual.toDenseMatrix(); LOG(INFO) << "Cam pose " <<
  //  relative_cam_pose.matrix();

  Eigen::Matrix3f g_mat =
      cam_intrinsics.camera_mat * relative_cam_pose.matrix() *
      transformed_ellipsoid_dual * (relative_cam_pose.matrix().transpose()) *
      (cam_intrinsics.camera_mat.transpose());
  //  LOG(INFO) << "G_mat \n" << g_mat;
  //  Eigen::Matrix3f g_mat = cam_intrinsics.camera_mat *
  //                          relative_ellipsoid_pose.matrix() *
  //                          transformed_ellipsoid_dual *
  //                          (relative_ellipsoid_pose.matrix().transpose()) *
  //                          (cam_intrinsics.camera_mat.transpose());

  float u_sqrt_term = sqrt(pow(g_mat(0, 2), 2) - (g_mat(0, 0) * g_mat(2, 2)));
  //  LOG(INFO) << "u sqrt term " << u_sqrt_term;
  float u_min = g_mat(0, 2) + u_sqrt_term;
  float u_max = g_mat(0, 2) - u_sqrt_term;
  float v_sqrt_term = sqrt(pow(g_mat(1, 2), 2) - (g_mat(1, 1) * g_mat(2, 2)));
  //  LOG(INFO) << "v sqrt term " << v_sqrt_term;
  float v_min = g_mat(1, 2) + v_sqrt_term;
  float v_max = g_mat(1, 2) - v_sqrt_term;

  u_min = u_min / g_mat(2, 2);
  u_max = u_max / g_mat(2, 2);
  v_min = v_min / g_mat(2, 2);
  v_max = v_max / g_mat(2, 2);

  return Eigen::Vector4f(u_min, u_max, v_min, v_max);
}

bool doesBoundingBoxSatisfyVisibilityParams(
    const Eigen::Vector4f &bounding_box,
    const EllipsoidVisibilityParams &ellipsoid_visibility_params,
    const CameraIntrinsics &camera_intrinsics) {
  std::pair<Eigen::Vector2f, Eigen::Vector2f> corners =
      vslam_util::cornerLocationsVectorToPair(bounding_box);
  float min_x = corners.first.x();
  float max_x = corners.second.x();
  float min_y = corners.first.y();
  float max_y = corners.second.y();

  bool big_enough =
      (((max_x - min_x) >
        ellipsoid_visibility_params.minimum_bounding_box_size_pixels) &&
       ((max_y - min_y) >
        ellipsoid_visibility_params.minimum_bounding_box_size_pixels));

  // TODO is this right?
  float max_pixel_x = 2 * camera_intrinsics.camera_mat(0, 2);
  float max_pixel_y = 2 * camera_intrinsics.camera_mat(1, 2);

  bool in_frame = min_x >= 0;
  in_frame = in_frame && (max_x >= 0);
  in_frame = in_frame && (min_x <= max_pixel_x);
  in_frame = in_frame && (max_x <= max_pixel_x);
  in_frame = in_frame && (min_y >= 0);
  in_frame = in_frame && (max_y >= 0);
  in_frame = in_frame && (min_y <= max_pixel_y);
  in_frame = in_frame && (max_y <= max_pixel_y);

  //  if (!in_frame) {
  //    LOG(INFO) << "Ellipsoid not in the bounds of the camera frame";
  //  }
  //  if (!big_enough) {
  //    LOG(INFO) << "Ellipsoid bounding box not big enough";
  //  }

  return in_frame && big_enough;
}

std::pair<std::vector<vslam_types::ObjectImageBoundingBoxDetection>,
          std::vector<vslam_types::ObjectImageBoundingBoxDetection>>
filterByVisibility(
    const std::vector<vslam_types::EllipsoidEstimate> &ground_truth_ellipsoids,
    const std::vector<vslam_types::RobotPose> &ground_truth_robot_poses,
    const std::unordered_map<vslam_types::CameraId,
                             vslam_types::CameraExtrinsics> &extrinsics,
    const EllipsoidVisibilityParams &ellipsoid_visibility_params,
    const std::vector<vslam_types::ObjectImageBoundingBoxDetection>
        &unfiltered_bounding_boxes,
    const std::pair<std::vector<vslam_types::ObjectImageBoundingBoxDetection>,
                    std::vector<vslam_types::ObjectImageBoundingBoxDetection>>
        &gt_and_noisy_bounding_boxes_unfiltered) {

  // TODO
  // Group detections by camera id and robot pose
  // For each camera id and robot (camera pose)
  //    Determine what the foreground order of the ellipsoids is
  //    Order the unfiltered bounding boxes by foreground order
  //    For each gt and noisy bounding box
  //        Find the pixels that make up the union of the unfiltered bounding boxes in the foreground
  //        Find the intersection of these pixels and those in the gt bounding box
  //        If this intersection is greater than the specified percentage of the total gt bounding box, exclude the gt and noisy bounding boxes

  // TODO to simulate occulsions, we'd find the min/max of the unocculated pixels (gt bounding box - noted intersection)

  return gt_and_noisy_bounding_boxes_unfiltered;
}

std::pair<std::vector<vslam_types::ObjectImageBoundingBoxDetection>,
          std::vector<vslam_types::ObjectImageBoundingBoxDetection>>
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
  std::vector<vslam_types::ObjectImageBoundingBoxDetection> gt_bounding_boxes;
  std::vector<vslam_types::ObjectImageBoundingBoxDetection>
      unfiltered_bounding_boxes;
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

        vslam_types::ObjectImageBoundingBoxDetection bounding_box_detection;
        bounding_box_detection.frame_idx = robot_pose_num;
        bounding_box_detection.ellipsoid_idx = ellipsoid_num;
        bounding_box_detection.camera_id = cam_id;
        bounding_box_detection.semantic_class = gt_ellipsoid.semantic_class;

        vslam_types::ObjectImageBoundingBoxDetection unfiltered_detection =
            bounding_box_detection;
        unfiltered_detection.pixel_corner_locations =
            vslam_util::cornerLocationsVectorToPair(
                noiseless_predicted_bounding_box);

        if (random_gen.UniformRandom() > bounding_box_detection_success_rate) {
          //          LOG(INFO) << "Simulated failed detection";
          // This detection "failed" so we don't add a bounding box for it
          continue;
        }
        //        LOG(INFO) << "Predicted noiseless bounding box "
        //                  << noiseless_predicted_bounding_box;
        if (!doesBoundingBoxSatisfyVisibilityParams(
                noiseless_predicted_bounding_box,
                ellipsoid_visibility_params,
                curr_cam_intrinsics)) {
          LOG(INFO) << "Noiseless bb estimate for ellipsoid "
                    << gt_ellipsoid.ellipsoid_idx << " observed by " << cam_id
                    << " at pose " << robot_pose.frame_idx
                    << " doesn't satisfy visibility";
          continue;
        }

        Eigen::Vector4f noisy_bounding_box =
            addNoiseToVectorDiagonalCov(noiseless_predicted_bounding_box,
                                        bounding_box_std_devs,
                                        random_gen);
        if (!doesBoundingBoxSatisfyVisibilityParams(noisy_bounding_box,
                                                    ellipsoid_visibility_params,
                                                    curr_cam_intrinsics)) {
          LOG(INFO) << "Noisy bb estimate for ellipsoid "
                    << gt_ellipsoid.ellipsoid_idx << " observed by " << cam_id
                    << " at pose " << robot_pose.frame_idx
                    << " doesn't satisfy visibility";
          continue;
        }

        vslam_types::ObjectImageBoundingBoxDetection gt_detection =
            bounding_box_detection;
        gt_detection.pixel_corner_locations =
            vslam_util::cornerLocationsVectorToPair(
                noiseless_predicted_bounding_box);

        std::pair<Eigen::Vector2f, Eigen::Vector2f> noisy_pixels =
            vslam_util::cornerLocationsVectorToPair(noisy_bounding_box);
        bounding_box_detection.pixel_corner_locations = noisy_pixels;

        gt_bounding_boxes.emplace_back(gt_detection);
        detected_bounding_boxes.emplace_back(bounding_box_detection);
      }
    }
  }

  // TODO filter out heavily obstructed bounding boxes (for now just trying to
  // simulate lack of visibility -- not
  //  trying to simulate partial visibility; i.e. if we think we've seen it, the
  //  bounding box reflects the whole object)

  return filterByVisibility(
      ground_truth_ellipsoids,
      ground_truth_robot_poses,
      extrinsics,
      ellipsoid_visibility_params,
      unfiltered_bounding_boxes,
      std::make_pair(gt_bounding_boxes, detected_bounding_boxes));
}

template <typename FeatureTrackType>
SyntheticProblemGeneratedData<FeatureTrackType>
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
  std::pair<std::vector<ObjectImageBoundingBoxDetection>,
            std::vector<ObjectImageBoundingBoxDetection>>
      gt_and_noisy_bounding_boxes =
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

  //  LOG(INFO) << "Ellipsoid pose est noise in synthetic "
  //            << ellipsoid_pose_estimate_noise;

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
  slam_problem.bounding_boxes = gt_and_noisy_bounding_boxes.second;
  slam_problem.camera_intrinsics_by_camera = intrinsics;
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
      .bounding_box_covariance = vslam_util::createDiagCovFromStdDevs(
      bounding_box_std_devs, kMinBoundingBoxStdDev);

  SyntheticProblemGeneratedData<FeatureTrackType> synthetic_output;
  synthetic_output.created_slam_problem = slam_problem;
  synthetic_output.object_slam_problem_params = object_slam_params;
  synthetic_output.initial_ellipsoid_estimates = noisy_ellipsoid_estimates;

  for (const ObjectImageBoundingBoxDetection &gt_detection :
       gt_and_noisy_bounding_boxes.first) {
    synthetic_output
        .gt_corner_locations[gt_detection.frame_idx][gt_detection.camera_id]
                            [gt_detection.ellipsoid_idx] =
        gt_detection.pixel_corner_locations;
  }
  for (const ObjectImageBoundingBoxDetection &noisy_detection :
       gt_and_noisy_bounding_boxes.second) {
    synthetic_output.observed_corner_locations[noisy_detection.frame_idx]
                                              [noisy_detection.camera_id]
                                              [noisy_detection.ellipsoid_idx] =
        noisy_detection.pixel_corner_locations;
  }

  return synthetic_output;
}

template SyntheticProblemGeneratedData<StructuredVisionFeatureTrack>
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
