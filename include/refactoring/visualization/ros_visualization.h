//
// Created by amanda on 1/23/22.
//

#ifndef UT_VSLAM_REFACTORING_ROS_VISUALIZATION_H
#define UT_VSLAM_REFACTORING_ROS_VISUALIZATION_H

#include <cv_bridge/cv_bridge.h>
#include <refactoring/types/ellipsoid_utils.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/highgui.hpp>
#include <optional>

namespace vslam_types_refactor {

const std::string kGtPrefix = "gt_";
const std::string kInitPrefix = "init_";
const std::string kEstPrefix = "est_";


enum PlotType { GROUND_TRUTH, INITIAL, ESTIMATED };

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

    color_for_plot_type_[GROUND_TRUTH] = ground_truth_bounding_box_color_;
    color_for_plot_type_[ESTIMATED] =
        predicted_bounding_box_from_optimized_color_;
    color_for_plot_type_[INITIAL] =
        observed_bounding_box_color_;  // TODO is this the color we want?

    prefixes_for_plot_type_[GROUND_TRUTH] = kGtPrefix;
    prefixes_for_plot_type_[ESTIMATED] = kEstPrefix;
    prefixes_for_plot_type_[INITIAL] = kInitPrefix;
  }

  std::string createEllipsoidFrameId(const ObjectId &ellipsoid_idx,
                                     const std::string &ellipsoid_prefix = "") {
    return frame_prefix_ + "ellipsoid_" + std::to_string(ellipsoid_idx);
  }

  std::string createRobotPoseFrameId(
      const FrameId &robot_pose_index,
      const std::string &robot_pose_prefix = "") {
    return frame_prefix_ + robot_pose_prefix + "robot_" +
           std::to_string(robot_pose_index);
  }

  std::string createCameraPoseFrameIdRelRobot(const FrameId &robot_pose_index,
                                              const CameraId &camera_id,
                                              const std::string &prefix = "") {
    return createRobotPoseFrameId(robot_pose_index, prefix) + "_cam" +
           std::to_string(camera_id);
  }

  std::pair<std::string, std::string>
  createImageAndCameraInfoTopicsForTrajectoryTypeRobotPoseAndCameraId(
      const FrameId &robot_pose_idx,
      const CameraId &camera_id,
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

  std::pair<std::string, std::string>
  createLatestImageAndCameraInfoTopicsForTrajectoryTypeCameraId(
      const CameraId &camera_id,
      const std::string &trajectory_type_prefix = "") {
    std::string trajectory_type_prefix_with_slash;
    if (!trajectory_type_prefix.empty()) {
      trajectory_type_prefix_with_slash = trajectory_type_prefix + "/";
    }
    std::string base_name = topic_prefix_ + "/" +
                            trajectory_type_prefix_with_slash + "/latest/cam_" +
                            std::to_string(camera_id);
    return std::make_pair(base_name + "/image_raw", base_name + "/camera_info");
  }

  void visualizeEllipsoids(
      const std::unordered_map<ObjectId, EllipsoidState<double>>
          &ellipsoid_estimates,
      const PlotType &plot_type) {
    std::string topic = createTopicForPlotTypeAndBase(plot_type, kEllipsoidTopicSuffix);
    visualizeEllipsoids(
        ellipsoid_estimates,
        topic,
        color_for_plot_type_.at(plot_type));
  }

  void visualizeEllipsoids(
      const std::unordered_map<ObjectId, EllipsoidState<double>>
          &ellipsoid_estimates,
      const std::string &topic,
      const std_msgs::ColorRGBA &color) {
    ros::Publisher pub = getOrCreatePublisher<visualization_msgs::Marker>(
        topic, kEllipsoidMarkerPubQueueSize);
    for (const auto &ellipsoid : ellipsoid_estimates) {
      publishEllipsoid(ellipsoid.second, ellipsoid.first, pub, color);
    }
  }

  void visualizeTrajectory(const std::vector<Pose3D<double>> &trajectory,
                      const PlotType &plot_type) {
    ros::Publisher pub = getOrCreatePublisher<visualization_msgs::Marker>(
        createTopicForPlotTypeAndBase(plot_type, kPoseTopicSuffix),
        kRobotPoseMarkerPubQueueSize);
    std_msgs::ColorRGBA color = color_for_plot_type_.at(plot_type);
    publishTrajectory(pub, color, trajectory);
  }

  // TODO modify this to also take min frame
  void visualizeCameraObservations(
      const FrameId &max_frame,
      const std::unordered_map<FrameId, Pose3D<double>> &initial_trajectory,
      const std::optional<std::unordered_map<FrameId, Pose3D<double>>>
          &optimized_trajectory,
      const std::optional<std::unordered_map<FrameId, Pose3D<double>>>
          &gt_trajectory,
      const std::optional<std::unordered_map<ObjectId, EllipsoidState<double>>>
          &initial_ellipsoid_estimates,
      const std::optional<std::unordered_map<ObjectId, EllipsoidState<double>>>
          &optimized_ellipsoid_estimates,
      const std::optional<std::unordered_map<ObjectId, EllipsoidState<double>>>
          &gt_ellipsoid_estimates,
      const std::unordered_map<CameraId, CameraExtrinsics<double>> &extrinsics,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &intrinsics,
      const std::unordered_map<CameraId, std::pair<double, double>>
          &img_heights_and_widths,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>> &images,
      const std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<ObjectId, BbCornerPair<double>>>>
          &observed_corner_locations,
      const std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<ObjectId, BbCornerPair<double>>>>
          &gt_corner_locations,
      const bool &display_boxes_if_no_image) {
    // Publish frames for ellipsoids
    if (initial_ellipsoid_estimates.has_value()) {
      publishEllipsoidTransforms(initial_ellipsoid_estimates.value(),
                                 kInitPrefix);
    }
    if (optimized_ellipsoid_estimates.has_value()) {
      publishEllipsoidTransforms(optimized_ellipsoid_estimates.value(),
                                 kEstPrefix);
    }
    if (gt_ellipsoid_estimates.has_value()) {
      publishEllipsoidTransforms(gt_ellipsoid_estimates.value(), kGtPrefix);
    }

    // Publish frames for trajectories and cameras and bounding boxes for each
    // trajectory
    publishTransformsForEachCamera(
        max_frame, initial_trajectory, extrinsics, kInitPrefix);
    publishBoundingBoxDataFromTrajectory(max_frame,
                                         initial_trajectory,
                                         kInitPrefix,
                                         initial_ellipsoid_estimates,
                                         optimized_ellipsoid_estimates,
                                         gt_ellipsoid_estimates,
                                         extrinsics,
                                         intrinsics,
                                         img_heights_and_widths,
                                         images,
                                         observed_corner_locations,
                                         gt_corner_locations,
                                         display_boxes_if_no_image);
    if (optimized_trajectory.has_value()) {
      publishTransformsForEachCamera(
          max_frame, optimized_trajectory.value(), extrinsics, kEstPrefix);
      publishBoundingBoxDataFromTrajectory(max_frame,
                                           optimized_trajectory.value(),
                                           kEstPrefix,
                                           initial_ellipsoid_estimates,
                                           optimized_ellipsoid_estimates,
                                           gt_ellipsoid_estimates,
                                           extrinsics,
                                           intrinsics,
                                           img_heights_and_widths,
                                           images,
                                           observed_corner_locations,
                                           gt_corner_locations,
                                           display_boxes_if_no_image);
    }
    if (gt_trajectory.has_value()) {
      publishTransformsForEachCamera(
          max_frame, gt_trajectory.value(), extrinsics, kGtPrefix);
      publishBoundingBoxDataFromTrajectory(max_frame,
                                           gt_trajectory.value(),
                                           kGtPrefix,
                                           initial_ellipsoid_estimates,
                                           optimized_ellipsoid_estimates,
                                           gt_ellipsoid_estimates,
                                           extrinsics,
                                           intrinsics,
                                           img_heights_and_widths,
                                           images,
                                           observed_corner_locations,
                                           gt_corner_locations,
                                           display_boxes_if_no_image);
    }
  }

  void publishBoundingBoxDataFromTrajectory(
      const FrameId &max_frame,
      const std::unordered_map<FrameId, Pose3D<double>> &trajectory,
      const std::string &trajectory_type_prefix,
      const std::optional<std::unordered_map<ObjectId, EllipsoidState<double>>>
          &initial_ellipsoid_estimates,
      const std::optional<std::unordered_map<ObjectId, EllipsoidState<double>>>
          &optimized_ellipsoid_estimates,
      const std::optional<std::unordered_map<ObjectId, EllipsoidState<double>>>
          &gt_ellipsoid_estimates,
      const std::unordered_map<CameraId, CameraExtrinsics<double>> &extrinsics,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &intrinsics,
      const std::unordered_map<CameraId, std::pair<double, double>>
          &img_heights_and_widths,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>> &images,
      const std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<ObjectId, BbCornerPair<double>>>>
          &observed_corner_locations,
      const std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<ObjectId, BbCornerPair<double>>>>
          &gt_corner_locations,
      const bool &display_boxes_if_no_image) {
    // For each robot pose
    for (FrameId pose_idx = 0; pose_idx <= max_frame; pose_idx++) {
      Pose3D<double> robot_pose = trajectory.at(pose_idx);

      // If we need images to display things, check if there are any images for
      // the robot pose
      if (!display_boxes_if_no_image &&
          (images.find(pose_idx) == images.end())) {
        continue;
      }

      // If there are no observed bounding boxes for the pose, skip it
      if (observed_corner_locations.find(pose_idx) ==
          observed_corner_locations.end()) {
        continue;
      }

      std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
          empty_images_map;
      std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
          &images_for_pose = empty_images_map;
      if (images.find(pose_idx) != images.end()) {
        images_for_pose = images.at(pose_idx);
      }

      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId, BbCornerPair<double>>>
          observed_corner_locations_for_pose =
              observed_corner_locations.at(pose_idx);
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId, BbCornerPair<double>>>
          gt_corner_locations_for_pose;
      if (gt_corner_locations.find(pose_idx) != gt_corner_locations.end()) {
        gt_corner_locations_for_pose = gt_corner_locations.at(pose_idx);
      }

      for (const auto &cam_id_and_extrinsics : extrinsics) {
        // If there are no observed bounding boxes for the camera at the pose,
        // sip it
        if (observed_corner_locations_for_pose.find(
                cam_id_and_extrinsics.first) ==
            observed_corner_locations_for_pose.end()) {
          continue;
        }

        std::unordered_map<ObjectId, BbCornerPair<double>>
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
          LOG(WARNING) << "No intrinsics found for camera "
                       << cam_id_and_extrinsics.first;
          continue;
        }
        CameraIntrinsicsMat<double> cam_intrinsics =
            intrinsics.at(cam_id_and_extrinsics.first);
        std::pair<double, double> image_height_and_width =
            img_heights_and_widths.at(cam_id_and_extrinsics.first);

        std::optional<sensor_msgs::Image::ConstPtr> image =
            has_image_for_cam_at_pose
                ? std::optional<
                      sensor_msgs::Image::ConstPtr>{images_for_pose.at(
                      cam_id_and_extrinsics.first)}
                : std::nullopt;

        std::pair<std::string, std::string> image_and_camera_info_topics =
            createImageAndCameraInfoTopicsForTrajectoryTypeRobotPoseAndCameraId(
                pose_idx, cam_id_and_extrinsics.first, trajectory_type_prefix);

        std::string frame_id = createCameraPoseFrameIdRelRobot(
            pose_idx, cam_id_and_extrinsics.first, trajectory_type_prefix);

        std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
            gt_corners = std::nullopt;

        if (gt_corner_locations_for_pose.find(cam_id_and_extrinsics.first) !=
            gt_corner_locations_for_pose.end()) {
          gt_corners =
              gt_corner_locations_for_pose.at(cam_id_and_extrinsics.first);
        }

        std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
            predicted_corners_from_optimized;
        std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
            predicted_corners_from_initial;
        std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
            predicted_corners_from_gt;
        std::unordered_map<ObjectId, BbCornerPair<double>>
            pred_corners_from_opt_nonopt;
        std::unordered_map<ObjectId, BbCornerPair<double>>
            pred_corners_from_gt_nonopt;
        std::unordered_map<ObjectId, BbCornerPair<double>>
            pred_corners_from_initial_nonopt;

        if (initial_ellipsoid_estimates.has_value() ||
            optimized_ellipsoid_estimates.has_value()) {
          std::unordered_map<ObjectId, EllipsoidState<double>> ellipsoid_ests;
          if (initial_ellipsoid_estimates.has_value()) {
            ellipsoid_ests = initial_ellipsoid_estimates.value();
          } else {
            ellipsoid_ests = optimized_ellipsoid_estimates.value();
          }

          for (const auto &ellipsoid : ellipsoid_ests) {
            // Only plot the predicted bounding box when there is an observed
            // one
            if (observed_corners_for_pose_and_cam.find(ellipsoid.first) ==
                observed_corners_for_pose_and_cam.end()) {
              continue;
            }

            if (optimized_ellipsoid_estimates.has_value()) {
              pred_corners_from_opt_nonopt[ellipsoid.first] =
                  getCornerLocationsPair(
                      optimized_ellipsoid_estimates.value().at(ellipsoid.first),
                      robot_pose,
                      cam_id_and_extrinsics.second,
                      cam_intrinsics);
            }
          }

          predicted_corners_from_initial =
              initial_ellipsoid_estimates.has_value()
                  ? std::optional<std::unordered_map<
                        ObjectId,
                        BbCornerPair<double>>>{pred_corners_from_initial_nonopt}
                  : std::nullopt;
          predicted_corners_from_optimized =
              optimized_ellipsoid_estimates.has_value()
                  ? std::optional<std::unordered_map<
                        ObjectId,
                        BbCornerPair<double>>>{pred_corners_from_opt_nonopt}
                  : std::nullopt;
          predicted_corners_from_gt =
              gt_ellipsoid_estimates.has_value()
                  ? std::optional<std::unordered_map<
                        ObjectId,
                        BbCornerPair<double>>>{pred_corners_from_gt_nonopt}
                  : std::nullopt;

          publishImageWithBoundingBoxes(image_and_camera_info_topics.first,
                                        image_and_camera_info_topics.second,
                                        frame_id,
                                        cam_intrinsics,
                                        image_height_and_width,
                                        image,
                                        observed_corner_locations_for_pose.at(
                                            cam_id_and_extrinsics.first),
                                        gt_corners,
                                        predicted_corners_from_optimized,
                                        predicted_corners_from_initial,
                                        predicted_corners_from_gt);

          if (pose_idx == max_frame) {
            std::pair<std::string, std::string>
                last_image_and_camera_info_topics =
                    createLatestImageAndCameraInfoTopicsForTrajectoryTypeCameraId(
                        cam_id_and_extrinsics.first, trajectory_type_prefix);
            publishImageWithBoundingBoxes(
                last_image_and_camera_info_topics.first,
                last_image_and_camera_info_topics.second,
                frame_id,
                cam_intrinsics,
                image_height_and_width,
                image,
                observed_corner_locations_for_pose.at(
                    cam_id_and_extrinsics.first),
                gt_corners,
                predicted_corners_from_optimized,
                predicted_corners_from_initial,
                predicted_corners_from_gt);
          }
        }
      }
    }
  }

  void publishTransformsForEachCamera(
      const FrameId &max_frame,
      const std::unordered_map<FrameId, Pose3D<double>> &trajectory,
      const std::unordered_map<CameraId, CameraExtrinsics<double>> &extrinsics,
      const std::string &robot_pose_prefix = "") {
    for (FrameId frame = 0; frame <= max_frame; frame++) {
      Pose3D<double> robot_pose = trajectory.at(frame);
      geometry_msgs::TransformStamped robot_transform;
      robot_transform.header.stamp = ros::Time::now();
      robot_transform.header.frame_id = kVizFrame;
      robot_transform.child_frame_id =
          createRobotPoseFrameId(frame, robot_pose_prefix);
      robot_transform.transform.translation.x = robot_pose.transl_.x();
      robot_transform.transform.translation.y = robot_pose.transl_.y();
      robot_transform.transform.translation.z = robot_pose.transl_.z();
      Eigen::Quaterniond robot_quat(robot_pose.orientation_);
      robot_transform.transform.rotation.x = robot_quat.x();
      robot_transform.transform.rotation.y = robot_quat.y();
      robot_transform.transform.rotation.z = robot_quat.z();
      robot_transform.transform.rotation.w = robot_quat.w();
      static_tf_broadcaster_.sendTransform(robot_transform);

      for (const auto &cam_id_and_extrinsics : extrinsics) {
        geometry_msgs::TransformStamped cam_transform;
        cam_transform.header.stamp = ros::Time::now();
        cam_transform.header.frame_id = robot_transform.child_frame_id;
        cam_transform.child_frame_id = createCameraPoseFrameIdRelRobot(
            frame, cam_id_and_extrinsics.first, robot_pose_prefix);
        Eigen::Quaterniond cam_extrinsics_quat(
            cam_id_and_extrinsics.second.orientation_);
        cam_transform.transform.translation.x =
            cam_id_and_extrinsics.second.transl_.x();
        cam_transform.transform.translation.y =
            cam_id_and_extrinsics.second.transl_.y();
        cam_transform.transform.translation.z =
            cam_id_and_extrinsics.second.transl_.z();
        cam_transform.transform.rotation.x = cam_extrinsics_quat.x();
        cam_transform.transform.rotation.y = cam_extrinsics_quat.y();
        cam_transform.transform.rotation.z = cam_extrinsics_quat.z();
        cam_transform.transform.rotation.w = cam_extrinsics_quat.w();
        //        LOG(INFO) << "Publishing transform from "
        //                  << cam_transform.header.frame_id << " to "
        //                  << cam_transform.child_frame_id;
        static_tf_broadcaster_.sendTransform(cam_transform);
      }
    }
  }

  void publishEllipsoidTransforms(
      const std::unordered_map<ObjectId, EllipsoidState<double>> &ellipsoids,
      const std::string &ellipoid_frame_prefix) {
    for (const auto &ellipsoid_est : ellipsoids) {
      geometry_msgs::TransformStamped ellipsoid_transform;
      ellipsoid_transform.header.stamp = ros::Time::now();
      ellipsoid_transform.header.frame_id = kVizFrame;
      ellipsoid_transform.child_frame_id =
          createEllipsoidFrameId(ellipsoid_est.first, ellipoid_frame_prefix);
      ellipsoid_transform.transform.translation.x =
          ellipsoid_est.second.pose_.transl_.x();
      ellipsoid_transform.transform.translation.y =
          ellipsoid_est.second.pose_.transl_.y();
      ellipsoid_transform.transform.translation.z =
          ellipsoid_est.second.pose_.transl_.z();
      Eigen::Quaterniond ellipsoid_quat(
          ellipsoid_est.second.pose_.orientation_);
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
      const CameraIntrinsicsMat<double>
          &intrinsics,  // Should we just take in cam info?
      const std::pair<double, double> &img_height_and_width,
      const std::optional<sensor_msgs::Image::ConstPtr> &image,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &observed_corner_locations,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &ground_truth_corner_locations,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &predicted_corner_locations_from_optimized,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &predicted_corner_locations_from_initial,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
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
      cv::Mat cv_img(img_height_and_width.first,
                     img_height_and_width.second,
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
      //      LOG(INFO) << "Drawing observed corner locations on image "
      //                << image_topic_id << "Num rects "
      //                << observed_corner_locations.value().size();
      drawRectanglesOnImage(observed_corner_locations.value(),
                            observed_bounding_box_color_,
                            cv_ptr);
    }
    if (ground_truth_corner_locations.has_value()) {
      //      LOG(INFO) << "Drawing gt corner locations on image " <<
      //      image_topic_id
      //                << "Num rects "
      //                << ground_truth_corner_locations.value().size();
      drawRectanglesOnImage(ground_truth_corner_locations.value(),
                            ground_truth_bounding_box_color_,
                            cv_ptr);
    }
    if (predicted_corner_locations_from_optimized.has_value()) {
      //      LOG(INFO) << "Drawing predicted corner locations from optimized on
      //      image "
      //                << image_topic_id << "Num rects "
      //                <<
      //                predicted_corner_locations_from_optimized.value().size();
      drawRectanglesOnImage(predicted_corner_locations_from_optimized.value(),
                            predicted_bounding_box_from_optimized_color_,
                            cv_ptr);
    }
    if (predicted_corner_locations_from_initial.has_value()) {
      //      LOG(INFO) << "Drawing predicted corner locations from initial on
      //      image "
      //                << image_topic_id << "Num rects "
      //                <<
      //                predicted_corner_locations_from_initial.value().size();
      drawRectanglesOnImage(predicted_corner_locations_from_initial.value(),
                            predicted_bounding_box_from_initial_color_,
                            cv_ptr);
    }
    if (predicted_corner_locations_from_gt.has_value()) {
      //      LOG(INFO) << "Drawing predicted corner locations from gt on image
      //      "
      //                << image_topic_id << "Num rects "
      //                << predicted_corner_locations_from_gt.value().size();
      drawRectanglesOnImage(predicted_corner_locations_from_gt.value(),
                            predicted_bounding_box_from_gt_color_,
                            cv_ptr);
    }

    // Publish image and camera info
    //    LOG(INFO) << "Publishing to " << image_pub.getTopic();
    image_pub.publish(cv_ptr->toImageMsg());
    publishCameraInfo(frame_id,
                      image_stamp,
                      intrinsics,
                      img_height_and_width,
                      intrinsics_pub);
  }

 private:
  const static uint32_t kEllipsoidMarkerPubQueueSize = 100;
  const static uint32_t kRobotPoseMarkerPubQueueSize = 1000;

  const static uint32_t kCameraInfoQueueSize = 100;

  const static constexpr double kSleepAfterPubCreationTime = 0.01;

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

  std::unordered_map<PlotType, std_msgs::ColorRGBA> color_for_plot_type_;
  std::unordered_map<PlotType, std::string> prefixes_for_plot_type_;

  // TODO maybe have another topic for LTM ellipsoids?
  const std::string kEllipsoidTopicSuffix = "ellipsoids";

  const std::string kPoseTopicSuffix = "pose";

  const int32_t kTrajectoryMarkerNum = 1;
  const int32_t kMaxRobotPoseMarkerNum = std::numeric_limits<int32_t>::max();

  const double kTrajectoryScaleX = 0.02;

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
      const std::string &topic_name,
      const int &queue_size,
      const ros::Duration &sleep_after_create =
          ros::Duration(kSleepAfterPubCreationTime)) {
    if (publishers_by_topic_.find(topic_name) == publishers_by_topic_.end()) {
      ros::Publisher pub = node_handle_.advertise<T>(topic_name, queue_size);
      publishers_by_topic_[topic_name] = pub;
      sleep_after_create.sleep();
    }
    return publishers_by_topic_[topic_name];
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

  void publishEllipsoid(const EllipsoidState<double> &ellipsoid,
                        const ObjectId &obj_id,
                        ros::Publisher &pub,
                        const std_msgs::ColorRGBA &color) {
    visualization_msgs::Marker marker;
    marker.scale.x = ellipsoid.dimensions_.x();
    marker.scale.y = ellipsoid.dimensions_.y();
    marker.scale.z = ellipsoid.dimensions_.z();

    marker.pose.position.x = ellipsoid.pose_.transl_.x();
    marker.pose.position.y = ellipsoid.pose_.transl_.y();
    marker.pose.position.z = ellipsoid.pose_.transl_.z();

    Eigen::Quaterniond ellipsoid_quat(ellipsoid.pose_.orientation_);
    marker.pose.orientation.x = ellipsoid_quat.x();
    marker.pose.orientation.y = ellipsoid_quat.y();
    marker.pose.orientation.z = ellipsoid_quat.z();
    marker.pose.orientation.w = ellipsoid_quat.w();

    marker.id = obj_id;

    marker.type = visualization_msgs::Marker::Type::SPHERE;

    marker.color = color;

    publishMarker(marker, pub);
  }

  std::string createTopicForPlotTypeAndBase(const PlotType &plot_type,
                                            const std::string &base_topic) {
    return topic_prefix_ + prefixes_for_plot_type_.at(plot_type) + base_topic;
  }

  cv::Scalar convertColorMsgToOpenCvColor(const std_msgs::ColorRGBA &color) {
    return CV_RGB(color.r * 255, color.g * 255, color.b * 255);
  }

  void drawRectanglesOnImage(
      const std::unordered_map<ObjectId, BbCornerPair<double>>
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
      const BbCornerPair<double> &bounding_box_corners,
      const std_msgs::ColorRGBA &color,
      const std::optional<ObjectId> &bounding_box_numeric_label,
      cv_bridge::CvImagePtr &cv_ptr) {
    //    LOG(INFO) << bounding_box_corners.first << ", " <<
    //    bounding_box_corners.second;
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
    //    int img_x_size = (cv_ptr->image).size().width;
    //    int img_y_size = (cv_ptr->image).size().height;
    //    LOG(INFO) << "Img dims " << img_x_size << ", " << img_y_size;
    //    cv::rectangle(cv_ptr->image, cv::Point(img_x_size * .25, img_y_size *
    //    0.25), cv::Point(img_x_size * 0.75, img_y_size * 0.75),
    //                  cv::Scalar(0, 255, 0));
    //    cv::imshow("Img with rect", cv_ptr->image);
    //    cv::waitKey(0);
  }

  void publishCameraInfo(const std::string &frame_id,
                         const ros::Time &header_stamp,
                         const CameraIntrinsicsMat<double> &intrinsics,
                         const std::pair<double, double> img_height_and_width,
                         ros::Publisher &camera_info_publisher) {
    //    LOG(INFO) << "Camera intrinsics for frame " << frame_id << ": "
    //              << intrinsics;
    sensor_msgs::CameraInfo cam_info;
    cam_info.header.stamp = header_stamp;
    cam_info.header.frame_id = frame_id;
    cam_info.K[0] = intrinsics(0, 0);
    cam_info.K[1] = intrinsics(0, 1);
    cam_info.K[2] = intrinsics(0, 2);
    cam_info.K[3] = intrinsics(1, 0);
    cam_info.K[4] = intrinsics(1, 1);
    cam_info.K[5] = intrinsics(1, 2);
    cam_info.K[6] = intrinsics(2, 0);
    cam_info.K[7] = intrinsics(2, 1);
    cam_info.K[8] = intrinsics(2, 2);

    //    for (const double &k_val : cam_info.K) {
    //      LOG(INFO) << "K " << k_val;
    //    }

    cam_info.width = img_height_and_width.second;
    cam_info.height = img_height_and_width.first;

    cam_info.P[0] = cam_info.K[0];
    cam_info.P[1] = cam_info.K[1];
    cam_info.P[2] = cam_info.K[2];
    cam_info.P[4] = cam_info.K[3];
    cam_info.P[5] = cam_info.K[4];
    cam_info.P[6] = cam_info.K[5];
    cam_info.P[8] = cam_info.K[6];
    cam_info.P[9] = cam_info.K[7];
    cam_info.P[10] = cam_info.K[8];

    //    for (const double &p_val : cam_info.P) {
    //      LOG(INFO) << "P " << p_val;
    //    }

    cam_info.distortion_model = "plumb_bob";

    camera_info_publisher.publish(cam_info);
  }

  void publishCameraIntrinsics(const CameraIntrinsicsMat<double> &intrinsics,
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
    cam_info.K[0] = intrinsics(0, 0);
    cam_info.K[1] = intrinsics(0, 1);
    cam_info.K[2] = intrinsics(0, 2);
    cam_info.K[3] = intrinsics(1, 0);
    cam_info.K[4] = intrinsics(1, 1);
    cam_info.K[5] = intrinsics(1, 2);
    cam_info.K[6] = intrinsics(2, 0);
    cam_info.K[7] = intrinsics(2, 1);
    cam_info.K[8] = intrinsics(2, 2);

    cam_info.width = intrinsics(0, 2) * 2;
    cam_info.height = intrinsics(1, 2) * 2;

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

  void publishTrajectory(ros::Publisher &marker_pub,
                         const std_msgs::ColorRGBA &color,
                         const std::vector<Pose3D<double>> &trajectory_poses) {
    visualization_msgs::Marker marker_msg;
    marker_msg.id = kTrajectoryMarkerNum;
    marker_msg.color = color;

    marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
    marker_msg.scale.x = kTrajectoryScaleX;
    for (const Pose3D<double> &traj_pose : trajectory_poses) {
      geometry_msgs::Point point;
      point.x = traj_pose.transl_.x();
      point.y = traj_pose.transl_.y();
      point.z = traj_pose.transl_.z();
      marker_msg.points.emplace_back(point);
    }

    marker_msg.pose.orientation.w = 1.0;
    publishMarker(marker_msg, marker_pub);

    // Also publish box for each pose in the trajectory
    publishRobotPoses(marker_pub,
                      trajectory_poses,
                      color,
                      kTrajectoryMarkerNum + 1,
                      kMaxRobotPoseMarkerNum);
  }

  void publishRobotPoses(ros::Publisher &marker_pub,
                         const std::vector<Pose3D<double>> &robot_poses,
                         const std_msgs::ColorRGBA &color,
                         const int32_t min_id,
                         const int32_t max_id) {
    for (size_t i = 0; i < robot_poses.size(); i++) {
      publishRobotPose(marker_pub, robot_poses[i], color, min_id + i);
    }
  }

  void publishRobotPose(ros::Publisher &marker_pub,
                        const Pose3D<double> &robot_pose,
                        const std_msgs::ColorRGBA &color,
                        const int32_t id,
                        bool bigger = false) {
    visualization_msgs::Marker marker_msg;

    marker_msg.scale.x = 0.025;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.025;

    if (bigger) {
      //                marker_msg.scale.x = 0.8;
      //                marker_msg.scale.y = 0.6;

      marker_msg.scale.x = 1.2;
      marker_msg.scale.y = 0.9;
      marker_msg.scale.z = 0.3;
    }

    marker_msg.pose.position.x = robot_pose.transl_.x();
    marker_msg.pose.position.y = robot_pose.transl_.y();
    marker_msg.pose.position.z = robot_pose.transl_.z();

    Eigen::Quaterniond quat(robot_pose.orientation_);
    marker_msg.pose.orientation.w = quat.w();
    marker_msg.pose.orientation.x = quat.x();
    marker_msg.pose.orientation.y = quat.y();
    marker_msg.pose.orientation.z = quat.z();

    marker_msg.type = visualization_msgs::Marker::CUBE;

    marker_msg.color = color;
    marker_msg.id = id;

    publishMarker(marker_msg, marker_pub);
  }
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_ROS_VISUALIZATION_H