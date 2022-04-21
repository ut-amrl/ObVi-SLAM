//
// Created by amanda on 1/23/22.
//

#ifndef UT_VSLAM_ROS_VISUALIZATION_H
#define UT_VSLAM_ROS_VISUALIZATION_H

#include <cv_bridge/cv_bridge.h>
#include <ellipsoid_utils.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <vslam_math_util.h>
#include <vslam_types.h>

#include <optional>

namespace vslam_viz {

const std::string kGtPrefix = "gt_";
const std::string kInitPrefix = "init_";
const std::string kEstPrefix = "est_";

class RosVisualization {
 public:
  RosVisualization(ros::NodeHandle &node_handle,
                   const std::string &topic_prefix = "",
                   const std::string &frame_prefix = "")
      : node_handle_(node_handle),
        topic_prefix_(topic_prefix),
        frame_prefix_(frame_prefix) {
    ground_truth_bounding_box_color_.a = 1;
    ground_truth_bounding_box_color_.g = 1;

    observed_bounding_box_color_.a = 1;
    observed_bounding_box_color_.b = 1;
    observed_bounding_box_color_.g = 0.5;

    predicted_bounding_box_from_optimized_color_.a = 1;
    predicted_bounding_box_from_optimized_color_.r = 1;

    predicted_bounding_box_from_gt_color_.a = 1;
    predicted_bounding_box_from_gt_color_.r =
        0.5;  // TODO do we want these to be 1 instead?
    predicted_bounding_box_from_gt_color_.g = 0.5;

    predicted_bounding_box_from_initial_color_.a = 1;
    predicted_bounding_box_from_initial_color_.r =
        0.5;  // TODO do we want these to be 1 instead?
    predicted_bounding_box_from_initial_color_.b = 0.5;
  }

  std::string createEllipsoidFrameId(const uint64_t &ellipsoid_idx,
                                     const std::string &ellipsoid_prefix = "") {
    return frame_prefix_ + "ellipsoid_" + std::to_string(ellipsoid_idx);
  }

  std::string createRobotPoseFrameId(
      const uint64_t &robot_pose_index,
      const std::string &robot_pose_prefix = "") {
    return frame_prefix_ + robot_pose_prefix + "robot_" +
           std::to_string(robot_pose_index);
  }

  std::string createCameraPoseFrameIdRelRobot(
      const uint64_t &robot_pose_index,
      const vslam_types::CameraId &camera_id,
      const std::string &prefix = "") {
    return createRobotPoseFrameId(robot_pose_index, prefix) + "_cam" +
           std::to_string(camera_id);
  }

  std::pair<std::string, std::string>
  createImageAndCameraInfoTopicsForTrajectoryTypeRobotPoseAndCameraId(
      const uint64_t &robot_pose_idx,
      const vslam_types::CameraId &camera_id,
      const std::string &trajectory_type_prefix = "") {
    std::string trajectory_type_prefix_with_slash;
    if (!trajectory_type_prefix.empty()) {
      trajectory_type_prefix_with_slash = trajectory_type_prefix + "/";
    }
    std::string base_name =
        topic_prefix_ + "/" + trajectory_type_prefix_with_slash + "/pose_" +
        std::to_string(robot_pose_idx) + +"/cam_" + std::to_string(camera_id);
    return std::make_pair(base_name + "/image_raw", base_name + "/camera_info");
  }

  void visualizeEllipsoids(
      const std::vector<vslam_types::EllipsoidEstimate> &ellipsoid_estimates,
      const std::string &topic,
      const std_msgs::ColorRGBA &color) {
    if (publishers_by_topic_.find(topic) == publishers_by_topic_.end()) {
      ros::Publisher ellipsoid_pub =
          node_handle_.advertise<visualization_msgs::Marker>(
              topic_prefix_ + topic, kEllipsoidMarkerPubQueueSize);
      publishers_by_topic_[topic_prefix_ + topic] = ellipsoid_pub;
      ros::Duration(kSleepAfterPubCreationTime).sleep();
    }
    ros::Publisher pub = publishers_by_topic_[topic_prefix_ + topic];
    for (const vslam_types::EllipsoidEstimate &ellipsoid :
         ellipsoid_estimates) {
      //      LOG(INFO) << "Publishing ellipsoid at pose " << ellipsoid.loc << "
      //      with dim " << ellipsoid.ellipsoid_dim;
      publishEllipsoid(ellipsoid, pub, color);
    }
  }

  void visualizeCameraObservations(
      const std::vector<vslam_types::RobotPose> &initial_trajectory,
      const std::optional<std::vector<vslam_types::RobotPose>>
          &optimized_trajectory,
      const std::optional<std::vector<vslam_types::RobotPose>> &gt_trajectory,
      const std::vector<vslam_types::EllipsoidEstimate>
          &initial_ellipsoid_estimates,
      const std::optional<std::vector<vslam_types::EllipsoidEstimate>>
          &optimized_ellipsoid_estimates,
      const std::optional<std::vector<vslam_types::EllipsoidEstimate>>
          &gt_ellipsoid_estimates,
      const std::unordered_map<vslam_types::CameraId,
                               vslam_types::CameraExtrinsics> &extrinsics,
      const std::unordered_map<vslam_types::CameraId,
                               vslam_types::CameraIntrinsics> &intrinsics,
      const std::unordered_map<
          uint64_t,
          std::unordered_map<vslam_types::CameraId, sensor_msgs::ImagePtr>>
          &images,
      const std::unordered_map<
          uint64_t,
          std::unordered_map<
              vslam_types::CameraId,
              std::unordered_map<uint64_t,
                                 std::pair<Eigen::Vector2f, Eigen::Vector2f>>>>
          &observed_corner_locations,
      const std::unordered_map<
          uint64_t,
          std::unordered_map<
              vslam_types::CameraId,
              std::unordered_map<uint64_t,
                                 std::pair<Eigen::Vector2f, Eigen::Vector2f>>>>
          &gt_corner_locations,
      const bool &display_boxes_if_no_image) {
    // Publish frames for ellipsoids
    publishEllipsoidTransforms(initial_ellipsoid_estimates, kInitPrefix);
    if (optimized_ellipsoid_estimates.has_value()) {
      publishEllipsoidTransforms(optimized_ellipsoid_estimates.value(),
                                 kEstPrefix);
    }
    if (gt_ellipsoid_estimates.has_value()) {
      publishEllipsoidTransforms(gt_ellipsoid_estimates.value(), kGtPrefix);
    }

    // Publish frames for trajectories and cameras and bounding boxes for each
    // trajectory
    publishTransformsForEachCamera(initial_trajectory, extrinsics, kInitPrefix);
    publishBoundingBoxDataFromTrajectory(initial_trajectory,
                                         kInitPrefix,
                                         initial_ellipsoid_estimates,
                                         optimized_ellipsoid_estimates,
                                         gt_ellipsoid_estimates,
                                         extrinsics,
                                         intrinsics,
                                         images,
                                         observed_corner_locations,
                                         gt_corner_locations,
                                         display_boxes_if_no_image);
    if (optimized_trajectory.has_value()) {
      publishTransformsForEachCamera(
          optimized_trajectory.value(), extrinsics, kEstPrefix);
      publishBoundingBoxDataFromTrajectory(optimized_trajectory.value(),
                                           kEstPrefix,
                                           initial_ellipsoid_estimates,
                                           optimized_ellipsoid_estimates,
                                           gt_ellipsoid_estimates,
                                           extrinsics,
                                           intrinsics,
                                           images,
                                           observed_corner_locations,
                                           gt_corner_locations,
                                           display_boxes_if_no_image);
    }
    if (gt_trajectory.has_value()) {
      publishTransformsForEachCamera(
          gt_trajectory.value(), extrinsics, kGtPrefix);
      publishBoundingBoxDataFromTrajectory(gt_trajectory.value(),
                                           kGtPrefix,
                                           initial_ellipsoid_estimates,
                                           optimized_ellipsoid_estimates,
                                           gt_ellipsoid_estimates,
                                           extrinsics,
                                           intrinsics,
                                           images,
                                           observed_corner_locations,
                                           gt_corner_locations,
                                           display_boxes_if_no_image);
    }
  }

  void publishBoundingBoxDataFromTrajectory(
      const std::vector<vslam_types::RobotPose> &trajectory,
      const std::string &trajectory_type_prefix,
      const std::vector<vslam_types::EllipsoidEstimate>
          &initial_ellipsoid_estimates,
      const std::optional<std::vector<vslam_types::EllipsoidEstimate>>
          &optimized_ellipsoid_estimates,
      const std::optional<std::vector<vslam_types::EllipsoidEstimate>>
          &gt_ellipsoid_estimates,
      const std::unordered_map<vslam_types::CameraId,
                               vslam_types::CameraExtrinsics> &extrinsics,
      const std::unordered_map<vslam_types::CameraId,
                               vslam_types::CameraIntrinsics> &intrinsics,
      const std::unordered_map<
          uint64_t,
          std::unordered_map<vslam_types::CameraId, sensor_msgs::ImagePtr>>
          &images,
      const std::unordered_map<
          uint64_t,
          std::unordered_map<
              vslam_types::CameraId,
              std::unordered_map<uint64_t,
                                 std::pair<Eigen::Vector2f, Eigen::Vector2f>>>>
          &observed_corner_locations,
      const std::unordered_map<
          uint64_t,
          std::unordered_map<
              vslam_types::CameraId,
              std::unordered_map<uint64_t,
                                 std::pair<Eigen::Vector2f, Eigen::Vector2f>>>>
          &gt_corner_locations,
      const bool &display_boxes_if_no_image) {
    // For each robot pose
    for (const vslam_types::RobotPose &robot_pose : trajectory) {
      // If we need images to display things, check if there are any images for
      // the robot pose
      if (!display_boxes_if_no_image &&
          (images.find(robot_pose.frame_idx) == images.end())) {
        continue;
      }

      // If there are no observed bounding boxes for the pose, skip it
      if (observed_corner_locations.find(robot_pose.frame_idx) ==
          observed_corner_locations.end()) {
        continue;
      }

      std::unordered_map<vslam_types::CameraId, sensor_msgs::ImagePtr>
          empty_images_map;
      std::unordered_map<vslam_types::CameraId, sensor_msgs::ImagePtr>
          &images_for_pose = empty_images_map;
      if (images.find(robot_pose.frame_idx) != images.end()) {
        images_for_pose = images.at(robot_pose.frame_idx);
      }

      std::unordered_map<
          vslam_types::CameraId,
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
          observed_corner_locations_for_pose =
              observed_corner_locations.at(robot_pose.frame_idx);
      std::unordered_map<
          vslam_types::CameraId,
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
          gt_corner_locations_for_pose;
      if (gt_corner_locations.find(robot_pose.frame_idx) !=
          gt_corner_locations.end()) {
        gt_corner_locations_for_pose =
            gt_corner_locations.at(robot_pose.frame_idx);
      }

      for (const auto &cam_id_and_extrinsics : extrinsics) {
        // If there are no observed bounding boxes for the camera at the pose,
        // sip it
        if (observed_corner_locations_for_pose.find(
                cam_id_and_extrinsics.first) ==
            observed_corner_locations_for_pose.end()) {
          continue;
        }

        std::unordered_map<uint64_t,
                           std::pair<Eigen::Vector2f, Eigen::Vector2f>>
            observed_corners_for_pose_and_cam =
                observed_corner_locations_for_pose.at(
                    cam_id_and_extrinsics.first);

        bool has_image_for_cam_at_pose =
            (images_for_pose.find(cam_id_and_extrinsics.first) !=
             images_for_pose.end());
        if (!display_boxes_if_no_image && !has_image_for_cam_at_pose) {
          continue;
        }

        if (intrinsics.find(cam_id_and_extrinsics.first) == intrinsics.end()) {
          LOG(WARNING) << "No intrinsics found for camera " << cam_id_and_extrinsics.first;
          continue;
        }
        vslam_types::CameraIntrinsics cam_intrinsics =
            intrinsics.at(cam_id_and_extrinsics.first);

        std::optional<sensor_msgs::ImagePtr> image =
            has_image_for_cam_at_pose
                ? std::optional<sensor_msgs::ImagePtr>{images_for_pose.at(
                      cam_id_and_extrinsics.first)}
                : std::nullopt;

        std::pair<std::string, std::string> image_and_camera_info_topics =
            createImageAndCameraInfoTopicsForTrajectoryTypeRobotPoseAndCameraId(
                robot_pose.frame_idx,
                cam_id_and_extrinsics.first,
                trajectory_type_prefix);

        std::string frame_id =
            createCameraPoseFrameIdRelRobot(robot_pose.frame_idx,
                                            cam_id_and_extrinsics.first,
                                            trajectory_type_prefix);

        std::optional<
            std::unordered_map<uint64_t,
                               std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
            gt_corners = std::nullopt;

        if (gt_corner_locations_for_pose.find(cam_id_and_extrinsics.first) !=
            gt_corner_locations_for_pose.end()) {
          gt_corners =
              gt_corner_locations_for_pose.at(cam_id_and_extrinsics.first);
        }

        std::optional<
            std::unordered_map<uint64_t,
                               std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
            predicted_corners_from_optimized;
        std::optional<
            std::unordered_map<uint64_t,
                               std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
            predicted_corners_from_initial;
        std::optional<
            std::unordered_map<uint64_t,
                               std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
            predicted_corners_from_gt;
        std::unordered_map<uint64_t,
                           std::pair<Eigen::Vector2f, Eigen::Vector2f>>
            pred_corners_from_opt_nonopt;
        std::unordered_map<uint64_t,
                           std::pair<Eigen::Vector2f, Eigen::Vector2f>>
            pred_corners_from_gt_nonopt;
        std::unordered_map<uint64_t,
                           std::pair<Eigen::Vector2f, Eigen::Vector2f>>
            pred_corners_from_initial_nonopt;

        for (size_t ellipsoid_entry_num = 0;
             ellipsoid_entry_num < initial_ellipsoid_estimates.size();
             ellipsoid_entry_num++) {
          vslam_types::EllipsoidEstimate init_ellipsoid =
              initial_ellipsoid_estimates[ellipsoid_entry_num];

          // Only plot the predicted bounding box when there is an observed one
          if (observed_corners_for_pose_and_cam.find(
                  init_ellipsoid.ellipsoid_idx) ==
              observed_corners_for_pose_and_cam.end()) {
            continue;
          }

          std::pair<Eigen::Vector2f, Eigen::Vector2f>
              predicted_corners_from_init_for_ellipsoid =
                  vslam_util::getCornerLocationsPair(
                      init_ellipsoid,
                      robot_pose,
                      cam_id_and_extrinsics.second,
                      cam_intrinsics);
          //          pred_corners_from_initial_nonopt[init_ellipsoid.ellipsoid_idx]
          //          =
          //              predicted_corners_from_init_for_ellipsoid;
          if (optimized_ellipsoid_estimates.has_value()) {
            pred_corners_from_opt_nonopt[init_ellipsoid.ellipsoid_idx] =
                vslam_util::getCornerLocationsPair(
                    optimized_ellipsoid_estimates.value()[ellipsoid_entry_num],
                    robot_pose,
                    cam_id_and_extrinsics.second,
                    cam_intrinsics);
          }
          //          if (gt_ellipsoid_estimates.has_value()) {
          //            pred_corners_from_gt_nonopt[init_ellipsoid.ellipsoid_idx]
          //            =
          //                vslam_util::getCornerLocationsPair(
          //                    gt_ellipsoid_estimates.value()[ellipsoid_entry_num],
          //                    robot_pose,
          //                    cam_id_and_extrinsics.second,
          //                    cam_intrinsics);
          //          }
        }

        predicted_corners_from_initial = pred_corners_from_initial_nonopt;
        predicted_corners_from_optimized =
            optimized_ellipsoid_estimates.has_value()
                ? std::optional<std::unordered_map<
                      uint64_t,
                      std::pair<Eigen::Vector2f,
                                Eigen::Vector2f>>>{pred_corners_from_opt_nonopt}
                : std::nullopt;
        predicted_corners_from_gt =
            gt_ellipsoid_estimates.has_value()
                ? std::optional<std::unordered_map<
                      uint64_t,
                      std::pair<Eigen::Vector2f,
                                Eigen::Vector2f>>>{pred_corners_from_gt_nonopt}
                : std::nullopt;

        publishImageWithBoundingBoxes(
            image_and_camera_info_topics.first,
            image_and_camera_info_topics.second,
            frame_id,
            cam_intrinsics,
            image,
            observed_corner_locations_for_pose.at(cam_id_and_extrinsics.first),
            gt_corners,
            predicted_corners_from_optimized,
            predicted_corners_from_initial,
            predicted_corners_from_gt);
      }
    }
  }

  //  void visualizeTrajectoryAndEllipsoidsWithTf(
  //      const std::vector<vslam_types::RobotPose> &trajectory,
  //      const std::vector<vslam_types::EllipsoidEstimate> &ellipsoids,
  //      const std::unordered_map<vslam_types::CameraId,
  //                               vslam_types::CameraExtrinsics> &extrinsics,
  //      const std::unordered_map<vslam_types::CameraId,
  //                               vslam_types::CameraIntrinsics> &intrinsics,
  //      const std::string &prefix = "") {
  //    publishEllipsoidTransforms(ellipsoids, prefix);
  //
  //    for (const vslam_types::RobotPose &robot_pose : trajectory) {
  //      geometry_msgs::TransformStamped robot_transform;
  //      robot_transform.header.stamp = ros::Time::now();
  //      robot_transform.header.frame_id = kVizFrame;
  //      robot_transform.child_frame_id =
  //          createRobotPoseFrameId(robot_pose.frame_idx, prefix);
  //      robot_transform.transform.translation.x = robot_pose.loc.x();
  //      robot_transform.transform.translation.y = robot_pose.loc.y();
  //      robot_transform.transform.translation.z = robot_pose.loc.z();
  //      Eigen::Quaternionf robot_quat(robot_pose.angle);
  //      robot_transform.transform.rotation.x = robot_quat.x();
  //      robot_transform.transform.rotation.y = robot_quat.y();
  //      robot_transform.transform.rotation.z = robot_quat.z();
  //      robot_transform.transform.rotation.w = robot_quat.w();
  //      static_tf_broadcaster_.sendTransform(robot_transform);
  //
  //      for (const auto &cam_id_and_extrinsics : extrinsics) {
  //        geometry_msgs::TransformStamped cam_transform;
  //        cam_transform.header.stamp = ros::Time::now();
  //        cam_transform.header.frame_id = robot_transform.child_frame_id;
  //        cam_transform.child_frame_id = createCameraPoseFrameIdRelRobot(
  //            robot_pose.frame_idx, cam_id_and_extrinsics.first, prefix);
  //        cam_transform.transform.translation.x =
  //            cam_id_and_extrinsics.second.translation.x();
  //        cam_transform.transform.translation.y =
  //            cam_id_and_extrinsics.second.translation.y();
  //        cam_transform.transform.translation.z =
  //            cam_id_and_extrinsics.second.translation.z();
  //        cam_transform.transform.rotation.x =
  //            cam_id_and_extrinsics.second.rotation.x();
  //        cam_transform.transform.rotation.y =
  //            cam_id_and_extrinsics.second.rotation.y();
  //        cam_transform.transform.rotation.z =
  //            cam_id_and_extrinsics.second.rotation.z();
  //        cam_transform.transform.rotation.w =
  //            cam_id_and_extrinsics.second.rotation.w();
  //        static_tf_broadcaster_.sendTransform(cam_transform);
  //        if (intrinsics.find(cam_id_and_extrinsics.first) !=
  //        intrinsics.end()) {
  //          publishImageWithBoundingBoxes(
  //              cam_transform.child_frame_id + "/image_raw",
  //              cam_transform.child_frame_id + "/camera_info",
  //              cam_transform.child_frame_id,
  //              intrinsics.at(cam_id_and_extrinsics.first),
  //              intrinsics.at(cam_id_and_extrinsics.first).camera_mat(0, 2) *
  //              2, intrinsics.at(cam_id_and_extrinsics.first).camera_mat(1, 2)
  //              * 2, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
  //          //
  //          publishCameraIntrinsics(intrinsics.at(cam_id_and_extrinsics.first),
  //          //                                  cam_transform.child_frame_id,
  //          //                                  cam_transform.child_frame_id +
  //          //                                  "/camera_info",
  //          //                                  cam_transform.child_frame_id +
  //          //                                  "/image_raw");
  //        }
  //
  //        vslam_types::RobotPose extrinsics_pose(
  //            robot_pose.frame_idx,
  //            cam_id_and_extrinsics.second.translation,
  //            Eigen::AngleAxisf(cam_id_and_extrinsics.second.rotation));
  //        vslam_types::RobotPose cam_pose_in_world =
  //            vslam_util::combinePoses(robot_pose, extrinsics_pose);
  //        Eigen::Quaternionf cam_quat(cam_pose_in_world.angle);
  //        geometry_msgs::TransformStamped alt_cam_transform;
  //        alt_cam_transform.header.stamp = ros::Time::now();
  //        alt_cam_transform.header.frame_id = kVizFrame;
  //        alt_cam_transform.child_frame_id =
  //            robot_transform.child_frame_id + "_cam" +
  //            std::to_string(cam_id_and_extrinsics.first) + "_alt";
  //        alt_cam_transform.transform.translation.x =
  //        cam_pose_in_world.loc.x(); alt_cam_transform.transform.translation.y
  //        = cam_pose_in_world.loc.y();
  //        alt_cam_transform.transform.translation.z =
  //        cam_pose_in_world.loc.z(); alt_cam_transform.transform.rotation.x =
  //        cam_quat.x(); alt_cam_transform.transform.rotation.y = cam_quat.y();
  //        alt_cam_transform.transform.rotation.z = cam_quat.z();
  //        alt_cam_transform.transform.rotation.w = cam_quat.w();
  //
  //        static_tf_broadcaster_.sendTransform(alt_cam_transform);
  //        for (const vslam_types::EllipsoidEstimate &ellipsoid_est :
  //        ellipsoids) {
  //          vslam_types::RobotPose ellipsoid_pose_in_world(
  //              robot_pose.frame_idx,
  //              ellipsoid_est.loc,
  //              ellipsoid_est.orientation);
  //
  //          vslam_types::RobotPose ellipsoid_pose_rel_cam =
  //              vslam_util::getPose2RelativeToPose1(cam_pose_in_world,
  //                                                  ellipsoid_pose_in_world);
  //          Eigen::Quaternionf ellipsoid_quat(ellipsoid_pose_rel_cam.angle);
  //          geometry_msgs::TransformStamped rel_ellipsoid_transform;
  //          rel_ellipsoid_transform.header.stamp = ros::Time::now();
  //          rel_ellipsoid_transform.header.frame_id =
  //              alt_cam_transform.child_frame_id;
  //          rel_ellipsoid_transform.child_frame_id = createEllipsoidFrameId(
  //              ellipsoid_est.ellipsoid_idx,
  //              alt_cam_transform.child_frame_id);
  //          rel_ellipsoid_transform.transform.translation.x =
  //              ellipsoid_pose_rel_cam.loc.x();
  //          rel_ellipsoid_transform.transform.translation.y =
  //              ellipsoid_pose_rel_cam.loc.y();
  //          rel_ellipsoid_transform.transform.translation.z =
  //              ellipsoid_pose_rel_cam.loc.z();
  //          rel_ellipsoid_transform.transform.rotation.x = ellipsoid_quat.x();
  //          rel_ellipsoid_transform.transform.rotation.y = ellipsoid_quat.y();
  //          rel_ellipsoid_transform.transform.rotation.z = ellipsoid_quat.z();
  //          rel_ellipsoid_transform.transform.rotation.w = ellipsoid_quat.w();
  //
  //          static_tf_broadcaster_.sendTransform(rel_ellipsoid_transform);
  //
  //          vslam_types::RobotPose cam_pose_rel_ellispoid =
  //              vslam_util::getPose2RelativeToPose1(ellipsoid_pose_in_world,
  //                                                  cam_pose_in_world);
  //          Eigen::Quaternionf cam_pose_rel_ellipsoid_quat(
  //              cam_pose_rel_ellispoid.angle);
  //          geometry_msgs::TransformStamped rel_cam_pose_transform;
  //          rel_cam_pose_transform.header.stamp = ros::Time::now();
  //          rel_cam_pose_transform.header.frame_id =
  //              createEllipsoidFrameId(ellipsoid_est.ellipsoid_idx, prefix);
  //          rel_cam_pose_transform.child_frame_id =
  //              createEllipsoidFrameId(ellipsoid_est.ellipsoid_idx,
  //                                     alt_cam_transform.child_frame_id) +
  //              "_rel_cam";
  //          rel_cam_pose_transform.transform.translation.x =
  //              cam_pose_rel_ellispoid.loc.x();
  //          rel_cam_pose_transform.transform.translation.y =
  //              cam_pose_rel_ellispoid.loc.y();
  //          rel_cam_pose_transform.transform.translation.z =
  //              cam_pose_rel_ellispoid.loc.z();
  //          rel_cam_pose_transform.transform.rotation.x =
  //              cam_pose_rel_ellipsoid_quat.x();
  //          rel_cam_pose_transform.transform.rotation.y =
  //              cam_pose_rel_ellipsoid_quat.y();
  //          rel_cam_pose_transform.transform.rotation.z =
  //              cam_pose_rel_ellipsoid_quat.z();
  //          rel_cam_pose_transform.transform.rotation.w =
  //              cam_pose_rel_ellipsoid_quat.w();
  //
  //          static_tf_broadcaster_.sendTransform(rel_cam_pose_transform);
  //        }
  //      }
  //    }
  //  }

  void publishTransformsForEachCamera(
      const std::vector<vslam_types::RobotPose> &trajectory,
      const std::unordered_map<vslam_types::CameraId,
                               vslam_types::CameraExtrinsics> &extrinsics,
      const std::string &robot_pose_prefix = "") {
    for (const vslam_types::RobotPose &robot_pose : trajectory) {
      geometry_msgs::TransformStamped robot_transform;
      robot_transform.header.stamp = ros::Time::now();
      robot_transform.header.frame_id = kVizFrame;
      robot_transform.child_frame_id =
          createRobotPoseFrameId(robot_pose.frame_idx, robot_pose_prefix);
      robot_transform.transform.translation.x = robot_pose.loc.x();
      robot_transform.transform.translation.y = robot_pose.loc.y();
      robot_transform.transform.translation.z = robot_pose.loc.z();
      Eigen::Quaternionf robot_quat(robot_pose.angle);
      robot_transform.transform.rotation.x = robot_quat.x();
      robot_transform.transform.rotation.y = robot_quat.y();
      robot_transform.transform.rotation.z = robot_quat.z();
      robot_transform.transform.rotation.w = robot_quat.w();
      static_tf_broadcaster_.sendTransform(robot_transform);

      for (const auto &cam_id_and_extrinsics : extrinsics) {
        geometry_msgs::TransformStamped cam_transform;
        cam_transform.header.stamp = ros::Time::now();
        cam_transform.header.frame_id = robot_transform.child_frame_id;
        cam_transform.child_frame_id =
            createCameraPoseFrameIdRelRobot(robot_pose.frame_idx,
                                            cam_id_and_extrinsics.first,
                                            robot_pose_prefix);
        cam_transform.transform.translation.x =
            cam_id_and_extrinsics.second.translation.x();
        cam_transform.transform.translation.y =
            cam_id_and_extrinsics.second.translation.y();
        cam_transform.transform.translation.z =
            cam_id_and_extrinsics.second.translation.z();
        cam_transform.transform.rotation.x =
            cam_id_and_extrinsics.second.rotation.x();
        cam_transform.transform.rotation.y =
            cam_id_and_extrinsics.second.rotation.y();
        cam_transform.transform.rotation.z =
            cam_id_and_extrinsics.second.rotation.z();
        cam_transform.transform.rotation.w =
            cam_id_and_extrinsics.second.rotation.w();
        LOG(INFO) << "Publishing transform from " << cam_transform.header.frame_id << " to " << cam_transform.child_frame_id;
        static_tf_broadcaster_.sendTransform(cam_transform);
      }
    }
  }

  void publishEllipsoidTransforms(
      const std::vector<vslam_types::EllipsoidEstimate> &ellipsoids,
      const std::string &ellipoid_frame_prefix) {
    for (const vslam_types::EllipsoidEstimate &ellipsoid_est : ellipsoids) {
      geometry_msgs::TransformStamped ellipsoid_transform;
      ellipsoid_transform.header.stamp = ros::Time::now();
      ellipsoid_transform.header.frame_id = kVizFrame;
      ellipsoid_transform.child_frame_id = createEllipsoidFrameId(
          ellipsoid_est.ellipsoid_idx, ellipoid_frame_prefix);
      ellipsoid_transform.transform.translation.x = ellipsoid_est.loc.x();
      ellipsoid_transform.transform.translation.y = ellipsoid_est.loc.y();
      ellipsoid_transform.transform.translation.z = ellipsoid_est.loc.z();
      Eigen::Quaternionf ellipsoid_quat(ellipsoid_est.orientation);
      ellipsoid_transform.transform.rotation.x = ellipsoid_quat.x();
      ellipsoid_transform.transform.rotation.y = ellipsoid_quat.y();
      ellipsoid_transform.transform.rotation.z = ellipsoid_quat.z();
      ellipsoid_transform.transform.rotation.w = ellipsoid_quat.w();
      static_tf_broadcaster_.sendTransform(ellipsoid_transform);
    }
  }

  void publishImageWithBoundingBoxes(
      const std::string &image_topic_id,
      const std::string &camera_info_topic_id,
      const std::string &frame_id,
      const vslam_types::CameraIntrinsics
          &intrinsics,  // Should we just take in cam info?
      const std::optional<sensor_msgs::ImagePtr> &image,
      const std::optional<
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
          &observed_corner_locations,
      const std::optional<
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
          &ground_truth_corner_locations,
      const std::optional<
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
          &predicted_corner_locations_from_optimized,
      const std::optional<
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
          &predicted_corner_locations_from_initial,
      const std::optional<
          std::unordered_map<uint64_t,
                             std::pair<Eigen::Vector2f, Eigen::Vector2f>>>
          &predicted_corner_locations_from_gt) {
    // Get or create publishers
    ros::Publisher intrinsics_pub =
        getOrCreatePublisher<sensor_msgs::CameraInfo>(camera_info_topic_id,
                                                      kCameraInfoQueueSize);
    ros::Publisher image_pub = getOrCreatePublisher<sensor_msgs::Image>(
        image_topic_id, kCameraInfoQueueSize);

    // Create/convert image
    cv_bridge::CvImagePtr cv_ptr;
    ros::Time image_stamp;
    if (image.has_value()) {
//      image_stamp = image.value()->header.stamp;
      try {
        cv_ptr = cv_bridge::toCvCopy(image.value(),
                                     sensor_msgs::image_encodings::BGR8);
        cv_ptr->header.frame_id = frame_id;
        image_stamp = ros::Time::now();
        cv_ptr->header.stamp = image_stamp;
      } catch (cv_bridge::Exception &e) {
        LOG(ERROR) << "cv_bridge exception: " << e.what();
        exit(1);
      }
    } else {
      // Create empty image
      // TODO: Is height/width correct?
      cv::Mat cv_img(intrinsics.image_height,
                     intrinsics.image_width,
                     CV_8UC3,
                     (0, 0, 0));  // Create black image
      image_stamp = ros::Time::now();
      std_msgs::Header img_header;
      img_header.stamp = image_stamp;
      img_header.frame_id = frame_id;
      cv_ptr = boost::make_shared<cv_bridge::CvImage>(
          img_header, sensor_msgs::image_encodings::BGR8, cv_img);
    }

    // Draw provided bounding boxes
    if (observed_corner_locations.has_value()) {
      drawRectanglesOnImage(observed_corner_locations.value(),
                            observed_bounding_box_color_,
                            cv_ptr);
    }
    if (ground_truth_corner_locations.has_value()) {
      drawRectanglesOnImage(ground_truth_corner_locations.value(),
                            ground_truth_bounding_box_color_,
                            cv_ptr);
    }
    if (predicted_corner_locations_from_optimized.has_value()) {
      drawRectanglesOnImage(predicted_corner_locations_from_optimized.value(),
                            predicted_bounding_box_from_optimized_color_,
                            cv_ptr);
    }
    if (predicted_corner_locations_from_initial.has_value()) {
      drawRectanglesOnImage(predicted_corner_locations_from_initial.value(),
                            predicted_bounding_box_from_initial_color_,
                            cv_ptr);
    }
    if (predicted_corner_locations_from_gt.has_value()) {
      drawRectanglesOnImage(predicted_corner_locations_from_gt.value(),
                            predicted_bounding_box_from_gt_color_,
                            cv_ptr);
    }

    // Publish image and camera info
//    LOG(INFO) << "Publishing to " << image_pub.getTopic();
    image_pub.publish(cv_ptr->toImageMsg());
    publishCameraInfo(frame_id, image_stamp, intrinsics, intrinsics_pub);
  }

 private:
  const static uint32_t kEllipsoidMarkerPubQueueSize = 100;

  const static uint32_t kCameraInfoQueueSize = 100;

  const static constexpr double kSleepAfterPubCreationTime = 0.5;

  const static int kBoundingBoxLineThickness = 4;

  const std::string kVizFrame = "map";

  const static int kBoundingBoxMinCornerLabelXOffset = 0;
  const static int kBoundingBoxMinCornerLabelYOffset = -10;

  const static constexpr double kBoundingBoxLabelFontScale = 1.0;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  std_msgs::ColorRGBA ground_truth_bounding_box_color_;

  std_msgs::ColorRGBA observed_bounding_box_color_;

  std_msgs::ColorRGBA predicted_bounding_box_from_optimized_color_;

  std_msgs::ColorRGBA predicted_bounding_box_from_gt_color_;

  std_msgs::ColorRGBA predicted_bounding_box_from_initial_color_;

  /**
   * Node handle.
   */
  ros::NodeHandle node_handle_;

  /**
   * Prefix for topics specific to this execution.
   */
  std::string topic_prefix_;

  /**
   * Prefix for frames specific to this execution. Must not contain /.
   */
  std::string frame_prefix_;

  /**
   * Need to take care that messages published align with the message type that
   * the publisher was initialized for.
   */
  std::unordered_map<std::string, ros::Publisher> publishers_by_topic_;

  template <typename T>
  ros::Publisher getOrCreatePublisher(
      const std::string &camera_info_topic,
      const int &queue_size,
      const ros::Duration &sleep_after_create =
          ros::Duration(kSleepAfterPubCreationTime)) {
    if (publishers_by_topic_.find(camera_info_topic) ==
        publishers_by_topic_.end()) {
      ros::Publisher pub =
          node_handle_.advertise<T>(camera_info_topic, queue_size);
      publishers_by_topic_[camera_info_topic] = pub;
      sleep_after_create.sleep();
    }
    return publishers_by_topic_[camera_info_topic];
  }

  void publishMarker(visualization_msgs::Marker &marker_msg,
                     ros::Publisher &marker_pub) {
    if (marker_msg.header.frame_id.empty()) {
      marker_msg.header.frame_id = kVizFrame;
    }
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "ut_vslam";
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker_msg);
  }

  void publishEllipsoid(const vslam_types::EllipsoidEstimate &ellipsoid,
                        ros::Publisher &pub,
                        const std_msgs::ColorRGBA &color) {
    visualization_msgs::Marker marker;
    marker.scale.x = ellipsoid.ellipsoid_dim.x();
    marker.scale.y = ellipsoid.ellipsoid_dim.y();
    marker.scale.z = ellipsoid.ellipsoid_dim.z();

    marker.pose.position.x = ellipsoid.loc.x();
    marker.pose.position.y = ellipsoid.loc.y();
    marker.pose.position.z = ellipsoid.loc.z();

    Eigen::Quaternionf ellipsoid_quat(ellipsoid.orientation);
    marker.pose.orientation.x = ellipsoid_quat.x();
    marker.pose.orientation.y = ellipsoid_quat.y();
    marker.pose.orientation.z = ellipsoid_quat.z();
    marker.pose.orientation.w = ellipsoid_quat.w();

    marker.id = ellipsoid.ellipsoid_idx;

    marker.type = visualization_msgs::Marker::Type::SPHERE;

    marker.color = color;

    publishMarker(marker, pub);
  }

  cv::Scalar convertColorMsgToOpenCvColor(const std_msgs::ColorRGBA &color) {
    return CV_RGB(color.r * 255, color.g * 255, color.b * 255);
  }

  void drawRectanglesOnImage(
      const std::unordered_map<uint64_t,
                               std::pair<Eigen::Vector2f, Eigen::Vector2f>>
          &bounding_box_corners,
      const std_msgs::ColorRGBA &color,
      cv_bridge::CvImagePtr &cv_ptr) {
    for (const auto &bounding_box_corners_entry : bounding_box_corners) {
      drawRectangleOnImage(bounding_box_corners_entry.second,
                           color,
                           bounding_box_corners_entry.first,
                           cv_ptr);
    }
  }

  void drawRectangleOnImage(
      const std::pair<Eigen::Vector2f, Eigen::Vector2f> &bounding_box_corners,
      const std_msgs::ColorRGBA &color,
      const std::optional<uint64_t> &bounding_box_numeric_label,
      cv_bridge::CvImagePtr &cv_ptr) {
    cv::rectangle(cv_ptr->image,
                  cv::Point(bounding_box_corners.first.x(),
                            bounding_box_corners.first.y()),
                  cv::Point(bounding_box_corners.second.x(),
                            bounding_box_corners.second.y()),
                  convertColorMsgToOpenCvColor(color),
                  kBoundingBoxLineThickness);

    if (bounding_box_numeric_label.has_value()) {
      cv::putText(cv_ptr->image,
                  std::to_string(bounding_box_numeric_label.value()),
                  cv::Point(bounding_box_corners.first.x() +
                                kBoundingBoxMinCornerLabelXOffset,
                            bounding_box_corners.first.y() +
                                kBoundingBoxMinCornerLabelYOffset),
                  cv::FONT_HERSHEY_SIMPLEX,
                  kBoundingBoxLabelFontScale,
                  convertColorMsgToOpenCvColor(color));
    }
  }

  void publishCameraInfo(const std::string &frame_id,
                         const ros::Time &header_stamp,
                         const vslam_types::CameraIntrinsics &intrinsics,
                         ros::Publisher &camera_info_publisher) {

    LOG(INFO) << "Camera intrinsics for frame " << frame_id << ": " << intrinsics.camera_mat;
    sensor_msgs::CameraInfo cam_info;
    cam_info.header.stamp = header_stamp;
    cam_info.header.frame_id = frame_id;
    cam_info.K[0] = intrinsics.camera_mat(0, 0);
    cam_info.K[1] = intrinsics.camera_mat(0, 1);
    cam_info.K[2] = intrinsics.camera_mat(0, 2);
    cam_info.K[3] = intrinsics.camera_mat(1, 0);
    cam_info.K[4] = intrinsics.camera_mat(1, 1);
    cam_info.K[5] = intrinsics.camera_mat(1, 2);
    cam_info.K[6] = intrinsics.camera_mat(2, 0);
    cam_info.K[7] = intrinsics.camera_mat(2, 1);
    cam_info.K[8] = intrinsics.camera_mat(2, 2);

    for (const double &k_val : cam_info.K) {
      LOG(INFO) << "K " << k_val;
    }

    cam_info.width = intrinsics.image_width;
    cam_info.height = intrinsics.image_height;

    cam_info.P[0] = cam_info.K[0];
    cam_info.P[1] = cam_info.K[1];
    cam_info.P[2] = cam_info.K[2];
    cam_info.P[4] = cam_info.K[3];
    cam_info.P[5] = cam_info.K[4];
    cam_info.P[6] = cam_info.K[5];
    cam_info.P[8] = cam_info.K[6];
    cam_info.P[9] = cam_info.K[7];
    cam_info.P[10] = cam_info.K[8];

    for (const double &p_val : cam_info.P) {
      LOG(INFO) << "P " << p_val;
    }

    cam_info.distortion_model = "plumb_bob";

    camera_info_publisher.publish(cam_info);
  }

  void publishCameraIntrinsics(const vslam_types::CameraIntrinsics &intrinsics,
                               const std::string &frame_id,
                               const std::string &topic_id,
                               const std::string &image_topic_id) {
    ros::Publisher intrinsics_pub =
        getOrCreatePublisher<sensor_msgs::CameraInfo>(topic_id,
                                                      kCameraInfoQueueSize);
    ros::Publisher image_pub = getOrCreatePublisher<sensor_msgs::Image>(
        image_topic_id, kCameraInfoQueueSize);
    sensor_msgs::CameraInfo cam_info;
    cam_info.header.stamp = ros::Time::now();
    cam_info.header.frame_id = frame_id;
    cam_info.K[0] = intrinsics.camera_mat(0, 0);
    cam_info.K[1] = intrinsics.camera_mat(0, 1);
    cam_info.K[2] = intrinsics.camera_mat(0, 2);
    cam_info.K[3] = intrinsics.camera_mat(1, 0);
    cam_info.K[4] = intrinsics.camera_mat(1, 1);
    cam_info.K[5] = intrinsics.camera_mat(1, 2);
    cam_info.K[6] = intrinsics.camera_mat(2, 0);
    cam_info.K[7] = intrinsics.camera_mat(2, 1);
    cam_info.K[8] = intrinsics.camera_mat(2, 2);

    cam_info.width = intrinsics.camera_mat(0, 2) * 2;
    cam_info.height = intrinsics.camera_mat(1, 2) * 2;

    cam_info.P[0] = cam_info.K[0];
    cam_info.P[1] = cam_info.K[1];
    cam_info.P[2] = cam_info.K[2];
    cam_info.P[4] = cam_info.K[3];
    cam_info.P[5] = cam_info.K[4];
    cam_info.P[6] = cam_info.K[5];
    cam_info.P[8] = cam_info.K[6];
    cam_info.P[9] = cam_info.K[7];
    cam_info.P[10] = cam_info.K[8];

    cam_info.distortion_model = "plumb_bob";

    sensor_msgs::Image image;
    image.width = cam_info.width;
    image.height = cam_info.height;
    image.header.frame_id = cam_info.header.frame_id;
    image.header.stamp = cam_info.header.stamp;

    intrinsics_pub.publish(cam_info);
    image_pub.publish(image);
  }
};
}  // namespace vslam_viz

#endif  // UT_VSLAM_ROS_VISUALIZATION_H
