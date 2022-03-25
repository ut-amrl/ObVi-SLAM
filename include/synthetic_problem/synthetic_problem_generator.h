//
// Created by amanda on 1/21/22.
//

#ifndef UT_VSLAM_SYNTHETIC_PROBLEM_GENERATOR_H
#define UT_VSLAM_SYNTHETIC_PROBLEM_GENERATOR_H

#include <object_slam_problem_params.h>
#include <shared/util/random.h>
#include <vslam_types.h>

#include <vector>

namespace synthetic_problem {

/**
 * Parameters used to determine if an ellipsoid is visible enough to have a
 * bounding box generated for it.
 */
struct EllipsoidVisibilityParams {
  /**
   * Minimum bounding box size in pixels. If the resulting bounding box
   * has either dimension less than this value, it will be excluded.
   *
   * This should be considered before adding noise and also after.
   */
  int minimum_bounding_box_size_pixels = 20;

  double min_visibility_percentage = 0.6;
};

/**
 * Data structure for data generated for the synthetic problem.
 * @tparam FeatureTrackType
 */
template <typename FeatureTrackType>
struct SyntheticProblemGeneratedData {
  /**
   * The problem created.
   */
  vslam_types::UTObjectSLAMProblem<FeatureTrackType> created_slam_problem;

  /**
   * Parameters for the SLAM problem.
   */
  vslam_solver::ObjectSlamProblemParams object_slam_problem_params;

  /**
   * Initial ellipsoid estimates.
   */
  std::vector<vslam_types::EllipsoidEstimate> initial_ellipsoid_estimates;

  /**
   * Observed corner locations (noise added).
   */
  std::unordered_map<
      uint64_t,
      std::unordered_map<
          vslam_types::CameraId,
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>>
      observed_corner_locations;

  /**
   * Ground truth corner locations.
   */
  std::unordered_map<
      uint64_t,
      std::unordered_map<
          vslam_types::CameraId,
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>>
      gt_corner_locations;
};

/**
 * Check if the bounding box satisfies the ellipsoid visibility parameters.
 *
 * @param bounding_box                  Bounding box to check. The first entry
 *                                      is the min x, second is max x, third is
 *                                      min y, and the fourth is max y.
 * @param ellipsoid_visibility_params   Ellipsoid visibility parameters
 *                                      specifying how big the bounding box
 *                                      must be.
 * @param camera_intrinsics             Camera intrinsics from which to derive
 *                                      frame size from.
 *
 * @return True if the bounding box satisfies visibility params (big enough and
 * in frame), false otherwise.
 */
bool doesBoundingBoxSatisfyVisibilityParams(
    const Eigen::Vector4f &bounding_box,
    const EllipsoidVisibilityParams &ellipsoid_visibility_params,
    const vslam_types::CameraIntrinsics &camera_intrinsics);

/**
 * Get the ground truth bounding box from the ground truth robot pose and
 * ellipsoid.
 *
 * @param gt_robot_pose     Ground truth robot pose from which ellipsoid was
 *                          viewed.
 * @param gt_ellipsoid      Ground truth ellipsoid.
 * @param cam_intrinsics    Camera intrinsics for the camera that "viewed" the
 *                          ellipsoid.
 * @param cam_extrinsics    Camera extrinsics for the camera that "viewed" the
 *                          ellipsoid (camera pose relative to robot baselink).
 *                          .
 * @return Bounding box of the ellipsoid observed at the given robot pose. Order
 * of the output is (min_x, max_x, min_y, max_y).
 */
Eigen::Vector4f getGroundTruthBoundingBoxObservation(
    const vslam_types::RobotPose &gt_robot_pose,
    const vslam_types::EllipsoidEstimate &gt_ellipsoid,
    const vslam_types::CameraIntrinsics &cam_intrinsics,
    const vslam_types::CameraExtrinsics &cam_extrinsics);

/**
 * TODO not implemented
 *
 * Filter the generated bounding box to remove those that would be heavily
 * occluded.
 *
 * @param ground_truth_ellipsoids                   Ground truth ellipsoids.
 * @param ground_truth_robot_poses                  Ground truth robot poses.
 * @param extrinsics                                Map of camera id to
 *                                                  extrinsics.
 * @param ellipsoid_visibility_params               Visibility parameters for
 *                                                  the ellipsoids.
 * @param unfiltered_bounding_boxes                 The unfiltered bounding
 *                                                  boxes. Includes those that
 *                                                  would be out of frame. Also
 *                                                  does not reflect any
 *                                                  detection failures.
 * @param gt_and_noisy_bounding_boxes_unfiltered    The ground truth and noisy
 *                                                  bounding boxes before
 *                                                  being filtered to remove the
 *                                                  heavily occluded ones.
 *
 * @return Ground ruth and noisy bounding boxes, with those that would be
 * heavily occluded removed.
 */
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
        &gt_and_noisy_bounding_boxes_unfiltered);

/**
 * Generate bounding box detections from ground truth ellipsoids and poses.
 *
 * @param ground_truth_ellipsoids               Ground truth ellipsoids.
 * @param ground_truth_robot_poses              Ground truth robot poses.
 * @param intrinsics                            Map of camera ids to camera
 *                                              intrinsics for the cameras.
 *                                              Assumes that there are the same
 *                                              keys for the intrinsics and
 *                                              extrinsics.
 * @param extrinsics                            Map of camera ids to camera
 *                                              extrinsics for the cameras.
 *                                              Assumes that there are the same
 *                                              keys for the intrinsics and
 *                                              extrinsics.
 * @param bounding_box_std_devs                 Standard deviations for each
 *                                              bounding box corner component.
 *                                              Entries are applied to (min_x,
 *                                              max_x, min_y, and max_y)
 *                                              respectively.
 * @param bounding_box_detection_success_rate   Success rate for the bounding
 *                                              box detections. Specifies the
 *                                              percentage of bounding box
 *                                              viewings that should result in
 *                                              bounding box detections.
 * @param ellipsoid_visibility_params           Parameters specifying how
 *                                              visible an ellipsoid must be to
 *                                              be included in the bounding box
 *                                              detections.
 * @param random_gen                            Random number generator.
 *
 * @return Ground truth bounding boxes and noisy bounding box detections.
 */
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
    util_random::Random &random_gen);

/**
 * Create a SLAM problem and object SLAM params based on the true robot poses
 * and ellipsoids.
 * @tparam FeatureTrackType Type of the feature track in the SLAM problem. We
 * don't actually use feature tracks in this function -- this is just to
 * parameterize the SLAM problem.
 *
 * @param ground_truth_ellipsoids                   Ground truth ellipsoid poses
 *                                                  and dimensions.
 * @param ground_truth_robot_poses                  Ground truth trajectory.
 * @param intrinsics                                Map of camera ids to camera
 *                                                  intrinsics for the cameras.
 *                                                  Assumes that there are the
 *                                                  same keys for the
 *                                                  intrinsics and extrinsics.
 * @param extrinsics                                Map of camera ids to camera
 *                                                  extrinsics for the cameras.
 *                                                  Assumes that there are the
 *                                                  same keys for the intrinsics
 *                                                  and extrinsics.
 * @param shape_mean_and_std_devs_by_semantic_class Mean and standard deviations
 *                                                  for each component of the
 *                                                  ellipsoid by the semantic
 *                                                  class.
 * @param bounding_box_std_devs                     Standard deviation for each
 *                                                  component of a generated
 *                                                  bounding box.
 * @param ellipsoid_pose_estimate_noise             Ellipsoid pose estimate
 *                                                  noise. Standard deviations
 *                                                  for each component of the
 *                                                  ellipsoid pose (location and
 *                                                  orientation).
 * @param robot_pose_std_devs                       Standard deviation for each
 *                                                  component of the robot pose.
 * @param bounding_box_detection_success_rate       Success rate for the
 *                                                  bounding box detections.
 *                                                  Specifies the percentage of
 *                                                  bounding box viewings that
 *                                                  should result in bounding
 *                                                  box detections.
 * @param ellipsoid_visibility_params               Parameters specifying how
 *                                                  visible an ellipsoid must be
 *                                                  to be included in the
 *                                                  bounding box detections.
 * @param random_gen                                Random number generator.
 *
 * @return Synthetic problem output (slam problem, problem params, generated
 * data).
 */
template <typename FeatureTrackType>
SyntheticProblemGeneratedData<FeatureTrackType>
createEllipsoidOnlySyntheticProblemFromEllipsoidsAndCameraPoses(
    const std::vector<vslam_types::EllipsoidEstimate> &ground_truth_ellipsoids,
    const std::vector<vslam_types::RobotPose> &ground_truth_robot_poses,
    const std::unordered_map<vslam_types::CameraId,
                             vslam_types::CameraIntrinsics> &intrinsics,
    const std::unordered_map<vslam_types::CameraId,
                             vslam_types::CameraExtrinsics> &extrinsics,
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

#endif  // UT_VSLAM_SYNTHETIC_PROBLEM_GENERATOR_H
