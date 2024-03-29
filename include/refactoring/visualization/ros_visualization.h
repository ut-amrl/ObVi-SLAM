//
// Created by amanda on 1/23/22.
//

#ifndef UT_VSLAM_REFACTORING_ROS_VISUALIZATION_H
#define UT_VSLAM_REFACTORING_ROS_VISUALIZATION_H

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <refactoring/image_processing/debugging_image_utils.h>
#include <refactoring/types/ellipsoid_utils.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <types/timestamped_data_to_frames_utils.h>
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

    residual_feature_color_.a = 1;
    residual_feature_color_.r = 1;
    residual_feature_color_.g = 234.0 / 255.0;
    residual_feature_color_.b = 0;

    pending_obj_color_.a = 0.7;
    pending_obj_color_.r = 137.0 / 255;
    pending_obj_color_.g = 250.0 / 255;
    pending_obj_color_.b = 224.0 / 255;

    ltm_obj_color_.a = 1.0;
    ltm_obj_color_.r = 0;
    ltm_obj_color_.g = 0;
    ltm_obj_color_.b = 0;

    color_for_plot_type_[GROUND_TRUTH] = ground_truth_bounding_box_color_;
    color_for_plot_type_[ESTIMATED] =
        predicted_bounding_box_from_optimized_color_;
    color_for_plot_type_[INITIAL] =
        observed_bounding_box_color_;  // TODO is this the color we want?

    prefixes_for_plot_type_[GROUND_TRUTH] = kGtPrefix;
    prefixes_for_plot_type_[ESTIMATED] = kEstPrefix;
    prefixes_for_plot_type_[INITIAL] = kInitPrefix;

    // TODO read this from file
    std_msgs::ColorRGBA bench_color;
    bench_color.a = 1.0;
    bench_color.r = 201.0 / 255;
    bench_color.g = 36.0 / 255;
    bench_color.b = 137.0 / 255;
    colors_for_semantic_classes_["bench"] = bench_color;

    std_msgs::ColorRGBA lamppost_color;
    lamppost_color.a = 1.0;
    lamppost_color.r = 135.0 / 255;
    lamppost_color.g = 157.0 / 255;
    lamppost_color.b = 161.0 / 255;
    colors_for_semantic_classes_["lamppost"] = lamppost_color;

    std_msgs::ColorRGBA tree_trunk_color;
    tree_trunk_color.a = 1.0;
    tree_trunk_color.r = 0.0 / 255;
    tree_trunk_color.g = 138.0 / 255;
    tree_trunk_color.b = 64.0 / 255;
    colors_for_semantic_classes_["treetrunk"] = tree_trunk_color;

    std_msgs::ColorRGBA trashcan_color;
    trashcan_color.a = 1.0;
    trashcan_color.r = 46.0 / 255;
    trashcan_color.g = 166.0 / 255;
    trashcan_color.b = 225.0 / 255;
    colors_for_semantic_classes_["trashcan"] = trashcan_color;
  }

  std::string createEllipsoidFrameId(const ObjectId &ellipsoid_idx,
                                     const std::string &ellipsoid_prefix = "") {
    return frame_prefix_ + "ellipsoid_" + std::to_string(ellipsoid_idx);
  }

  void publishBox(const std::string &topic_base,
                  const PlotType &plot_type,
                  const Position3d<double> &box_center,
                  const Eigen::Quaterniond &box_orientation,
                  const Eigen::Vector3d &box_dims,
                  const int &box_id,
                  const double &alpha = 0.2) {
    std::string topic = createTopicForPlotTypeAndBase(plot_type, topic_base);
    ros::Publisher pub =
        getOrCreatePublisher<visualization_msgs::Marker>(topic, 1000);

    std_msgs::ColorRGBA color = color_for_plot_type_.at(plot_type);
    publishBox(pub, color, box_center, box_orientation, box_dims, box_id);
  }

  void publishBox(ros::Publisher &marker_pub,
                  const std_msgs::ColorRGBA &color,
                  const Position3d<double> &box_center,
                  const Eigen::Quaterniond &box_orientation,
                  const Eigen::Vector3d &box_dims,
                  const int &box_id,
                  const double &alpha = 0.2) {
    visualization_msgs::Marker marker_msg;
    marker_msg.id = box_id;
    marker_msg.color = color;
    marker_msg.color.a = alpha;
    marker_msg.type = visualization_msgs::Marker::CUBE;
    marker_msg.scale.x = box_dims.x();
    marker_msg.scale.y = box_dims.y();
    marker_msg.scale.z = box_dims.z();

    marker_msg.pose.position.x = box_center.x();
    marker_msg.pose.position.y = box_center.y();
    marker_msg.pose.position.z = box_center.z();

    marker_msg.pose.orientation.x = box_orientation.x();
    marker_msg.pose.orientation.y = box_orientation.y();
    marker_msg.pose.orientation.z = box_orientation.z();
    marker_msg.pose.orientation.w = box_orientation.w();

    publishMarker(marker_msg, marker_pub);
  }

  void publishLines(
      const std::string &topic_base,
      const PlotType &plot_type,
      const std::vector<std::pair<Position3d<double>, Position3d<double>>>
          &lines,
      const int32_t &marker_num) {
    std::string topic = createTopicForPlotTypeAndBase(plot_type, topic_base);
    ros::Publisher pub =
        getOrCreateVisMarkerPublisherAndClearPrevious(topic, 1000);
    std_msgs::ColorRGBA color = color_for_plot_type_.at(plot_type);
    publishLines(pub, color, lines, marker_num);
  }

  void publishLines(
      ros::Publisher &marker_pub,
      const std_msgs::ColorRGBA &color,
      const std::vector<std::pair<Position3d<double>, Position3d<double>>>
          &lines,
      const int32_t marker_num) {
    visualization_msgs::Marker marker_msg;
    marker_msg.id = marker_num;
    marker_msg.color = color;
    marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    marker_msg.scale.x = kTrajectoryScaleX;
    marker_msg.pose.orientation.w = 1.0;

    for (const std::pair<Position3d<double>, Position3d<double>> &line_seg :
         lines) {
      geometry_msgs::Point point1;
      point1.x = line_seg.first.x();
      point1.y = line_seg.first.y();
      point1.z = line_seg.first.z();
      marker_msg.points.emplace_back(point1);

      geometry_msgs::Point point2;
      point2.x = line_seg.second.x();
      point2.y = line_seg.second.y();
      point2.z = line_seg.second.z();
      marker_msg.points.emplace_back(point2);
    }
    publishMarker(marker_msg, marker_pub);
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

  std::string getFrameForLatestPose(const std::string &type_prefix) {
    return frame_prefix_ + type_prefix + kFrameForLatestNode;
  }

  std::string getFrameForLatestPoseAndCamId(const std::string &type_prefix,
                                            const CameraId &cam_id) {
    return frame_prefix_ + type_prefix + kFrameForLatestNode + "_cam" +
           std::to_string(cam_id);
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

  std::pair<std::pair<std::string, std::string>, std::string>
  createLatestImageAndCameraInfoTopicsForTrajectoryTypeCameraId(
      const CameraId &camera_id,
      const std::string &data_type,
      const std::string &latest_frame_prefix,
      const std::string &trajectory_type_prefix = "") {
    std::string trajectory_type_prefix_with_slash;
    if (!trajectory_type_prefix.empty()) {
      trajectory_type_prefix_with_slash = trajectory_type_prefix + "/";
    }
    std::string base_name = topic_prefix_ + "/" +
                            trajectory_type_prefix_with_slash + "/latest/cam_" +
                            std::to_string(camera_id) + "_" + data_type;
    std::string frame_name =
        getFrameForLatestPoseAndCamId(latest_frame_prefix, camera_id);
    return std::make_pair(
        std::make_pair(base_name + "/image_raw", base_name + "/camera_info"),
        frame_name);
  }

  std::string createFeatureCloudTopicName(const PlotType &plot_type) {
    return topic_prefix_ + "/" + prefixes_for_plot_type_.at(plot_type) +
           "/feature_cloud";
  }

  void visualizeEllipsoids(
      const std::unordered_map<ObjectId,
                               std::pair<std::string, EllipsoidState<double>>>
          &ellipsoid_estimates,
      const PlotType &plot_type,
      const bool &different_colors_per_class = true) {
    std::string topic =
        createTopicForPlotTypeAndBase(plot_type, kEllipsoidTopicSuffix);
    LOG(INFO) << "Publishing ellipsoids for plot type " << topic;
    visualizeEllipsoids(ellipsoid_estimates,
                        topic,
                        color_for_plot_type_.at(plot_type),
                        different_colors_per_class);
  }

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  void visualizeEllipsoids(
      const std::unordered_map<
          ObjectId,
          std::pair<std::string, FullDOFEllipsoidState<double>>>
          &ellipsoid_estimates,
      const PlotType &plot_type,
      const bool &different_colors_per_class = true) {
    std::string topic =
        createTopicForPlotTypeAndBase(plot_type, kEllipsoidTopicSuffix);
    LOG(INFO) << "Publishing ellipsoids for plot type " << topic;
    visualizeEllipsoids(ellipsoid_estimates,
                        topic,
                        color_for_plot_type_.at(plot_type),
                        different_colors_per_class);
  }
#endif

  void visualizePendingEllipsoids(
      const std::shared_ptr<std::vector<
          std::pair<std::string, std::optional<EllipsoidState<double>>>>>
          &pending_objects,
      const std::vector<size_t> &num_obs_per_obj,
      const size_t &min_obs_threshold,
      const bool &different_colors_per_class = true) {
    std::string topic = topic_prefix_ + "pending_" + kEllipsoidTopicSuffix;
    ros::Publisher pub = getOrCreateVisMarkerPublisherAndClearPrevious(
        topic, kEllipsoidMarkerPubQueueSize);
    publishDeleteAllMarker(pub);
    std::vector<std::pair<std::string, std::optional<EllipsoidState<double>>>>
        pending_objects_vec = *pending_objects;
    for (size_t ellipsoid_id = 0; ellipsoid_id < pending_objects_vec.size();
         ellipsoid_id++) {
      std::pair<std::string, std::optional<EllipsoidState<double>>> ellipsoid =
          pending_objects_vec[ellipsoid_id];
      if (ellipsoid.second.has_value()) {
        std_msgs::ColorRGBA color_for_ellipsoid = pending_obj_color_;
        if (different_colors_per_class) {
          color_for_ellipsoid =
              brightenColor(getColorForClass(ellipsoid.first), .25);
        }
        color_for_ellipsoid.a =
            (((0.8 - 0.25) * num_obs_per_obj[ellipsoid_id]) /
             min_obs_threshold) +
            0.25;
        publishEllipsoid(ellipsoid.second.value(),
                         ellipsoid_id,
                         pub,
                         color_for_ellipsoid,
                         false);
      }
    }
  }

  void visualizeEllipsoids(
      const std::unordered_map<ObjectId,
                               std::pair<std::string, EllipsoidState<double>>>
          &ellipsoid_estimates,
      const std::string &topic,
      const std_msgs::ColorRGBA &color,
      const bool &different_colors_per_class) {
    ros::Publisher pub = getOrCreateVisMarkerPublisherAndClearPrevious(
        topic, kEllipsoidMarkerPubQueueSize);
    for (const auto &ellipsoid : ellipsoid_estimates) {
      std_msgs::ColorRGBA color_for_ellipsoid = color;
      if (different_colors_per_class) {
        color_for_ellipsoid = getColorForClass(ellipsoid.second.first);
      }
      publishEllipsoid(
          ellipsoid.second.second, ellipsoid.first, pub, color_for_ellipsoid);
    }
  }

#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  void visualizeEllipsoids(
      const std::unordered_map<
          ObjectId,
          std::pair<std::string, FullDOFEllipsoidState<double>>>
          &ellipsoid_estimates,
      const std::string &topic,
      const std_msgs::ColorRGBA &color,
      const bool &different_colors_per_class) {
    ros::Publisher pub = getOrCreateVisMarkerPublisherAndClearPrevious(
        topic, kEllipsoidMarkerPubQueueSize);
    for (const auto &ellipsoid : ellipsoid_estimates) {
      std_msgs::ColorRGBA color_for_ellipsoid = color;
      if (different_colors_per_class) {
        color_for_ellipsoid = getColorForClass(ellipsoid.second.first);
      }
      publishEllipsoid(
          ellipsoid.second.second, ellipsoid.first, pub, color_for_ellipsoid);
    }
  }
#endif

  void visualizeTrajectory(const std::vector<Pose3D<double>> &trajectory,
                           const PlotType &plot_type) {
    ros::Publisher pub = getOrCreateVisMarkerPublisherAndClearPrevious(
        createTopicForPlotTypeAndBase(plot_type, kPoseTopicSuffix),
        kRobotPoseMarkerPubQueueSize);
    std_msgs::ColorRGBA color = color_for_plot_type_.at(plot_type);
    publishTrajectory(pub, color, trajectory);
  }

  void visualizeTrajectories(
      const std::vector<std::vector<Pose3D<double>>> &trajectories,
      const PlotType &plot_type) {
    ros::Publisher pub = getOrCreateVisMarkerPublisherAndClearPrevious(
        createTopicForPlotTypeAndBase(plot_type, kPoseTopicSuffix),
        kRobotPoseMarkerPubQueueSize);
    ros::Duration(2).sleep();
    std_msgs::ColorRGBA color = color_for_plot_type_.at(plot_type);
    int32_t next_marker = kTrajectoryMarkerNum;
    float inc = 1.0 / (trajectories.size() * 4);
    float curr_inc = 0;
    for (const std::vector<Pose3D<double>> &trajectory : trajectories) {
      color.g += curr_inc;
      if (plot_type == INITIAL) {
        color.b -= curr_inc;
      } else {
        color.r -= curr_inc;
      }
      curr_inc += inc;
      ros::Duration(0.5).sleep();
      next_marker = publishTrajectory(pub, color, trajectory, next_marker);
    }
  }

  void visualizeWaypoints(
      const std::unordered_map<WaypointId,
                               std::vector<std::optional<Pose3D<double>>>>
          &waypoints) {
    int32_t next_robot_pose_id = 1;
    std::string topic_name =
        createTopicForPlotTypeAndBase(ESTIMATED, "waypoint");
    ros::Publisher waypoint_pub_ =
        getOrCreateVisMarkerPublisherAndClearPrevious(
            topic_name, kRobotPoseMarkerPubQueueSize);
    LOG(INFO) << "Publishing waypoints to " << topic_name;
    ros::Duration(4).sleep();

    std::vector<std_msgs::ColorRGBA> colors;
    size_t waypoint_idx = 0;
    size_t waypoints_size = waypoints.size();
    for (const auto &data_for_wp : waypoints) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.g = 1.0;

      size_t half_poses = waypoints_size / 2;
      if (waypoint_idx % 2) {
        color.b = ((double)(waypoint_idx / 2)) / half_poses;
        color.g = 1.0;
      } else {
        color.b = 1.0;
        color.g = ((double)((waypoints_size - waypoint_idx) / 2)) / half_poses;
      }
      waypoint_idx++;
      LOG(INFO) << "Publishing wp " << data_for_wp.first << " num points "
                << data_for_wp.second.size();
      std::vector<Pose3D<double>> non_opt_wp_poses;
      for (const std::optional<Pose3D<double>> &wp_pose : data_for_wp.second) {
        if (wp_pose.has_value()) {
          non_opt_wp_poses.emplace_back(wp_pose.value());
        }
      }
      publishRobotPoses(waypoint_pub_,
                        non_opt_wp_poses,
                        color,
                        next_robot_pose_id,
                        std::numeric_limits<int32_t>::max(),
                        true);
      next_robot_pose_id += data_for_wp.second.size();
    }
  }

  void publishTfsForLatestPose(
      const Pose3D<double> &latest_pose,
      const PlotType &plot_type,
      const std::unordered_map<CameraId, CameraExtrinsics<double>>
          &extrinsics) {
    std::string bl_frame_id =
        getFrameForLatestPose(prefixes_for_plot_type_.at(plot_type));
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = kVizFrame;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = bl_frame_id;
    transform.transform = convertPose3DtoTransformMsg(latest_pose);
    tf_broadcaster_.sendTransform(transform);

    for (const auto &extrinsics_entry : extrinsics) {
      geometry_msgs::TransformStamped transform_cam;
      transform_cam.header.frame_id = kVizFrame;
      transform_cam.header.stamp = ros::Time::now();
      transform_cam.child_frame_id = getFrameForLatestPoseAndCamId(
          prefixes_for_plot_type_.at(plot_type), extrinsics_entry.first);
      transform_cam.transform = convertPose3DtoTransformMsg(
          combinePoses(latest_pose, extrinsics_entry.second));
      tf_broadcaster_.sendTransform(transform_cam);
    }
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
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>
          &observed_corner_locations,
      const std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<ObjectId, BbCornerPair<double>>>>
          &gt_corner_locations,
      const bool &display_boxes_if_no_image,
      const double &near_edge_threshold = -1) {
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
    //    publishTransformsForEachCamera(
    //        max_frame, initial_trajectory, extrinsics, kInitPrefix);
    publishBoundingBoxDataFromTrajectory(
        max_frame,
        initial_trajectory,
        prefixes_for_plot_type_.at(PlotType::INITIAL),
        initial_ellipsoid_estimates,
        optimized_ellipsoid_estimates,
        gt_ellipsoid_estimates,
        extrinsics,
        intrinsics,
        img_heights_and_widths,
        images,
        observed_corner_locations,
        gt_corner_locations,
        display_boxes_if_no_image,
        near_edge_threshold);
    if (optimized_trajectory.has_value()) {
      //      publishTransformsForEachCamera(
      //          max_frame, optimized_trajectory.value(), extrinsics,
      //          kEstPrefix);
      publishBoundingBoxDataFromTrajectory(
          max_frame,
          optimized_trajectory.value(),
          prefixes_for_plot_type_.at(PlotType::ESTIMATED),
          initial_ellipsoid_estimates,
          optimized_ellipsoid_estimates,
          gt_ellipsoid_estimates,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images,
          observed_corner_locations,
          gt_corner_locations,
          display_boxes_if_no_image,
          near_edge_threshold);
    }
    if (gt_trajectory.has_value()) {
      //      publishTransformsForEachCamera(
      //          max_frame, gt_trajectory.value(), extrinsics, kGtPrefix);
      publishBoundingBoxDataFromTrajectory(
          max_frame,
          gt_trajectory.value(),
          prefixes_for_plot_type_.at(PlotType::GROUND_TRUTH),
          initial_ellipsoid_estimates,
          optimized_ellipsoid_estimates,
          gt_ellipsoid_estimates,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images,
          observed_corner_locations,
          gt_corner_locations,
          display_boxes_if_no_image,
          near_edge_threshold);
    }
  }

  void publishDetectedBoundingBoxesWithUncertainty(
      const FrameId &max_frame,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::vector<std::pair<BbCornerPair<double>,
                                                   std::optional<double>>>>>
          &observed_corner_locations_with_uncertainty,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>> &images,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &intrinsics,
      const std::unordered_map<CameraId, std::pair<double, double>>
          &img_heights_and_widths,
      const PlotType &plot_type) {
    for (const auto &frame_and_bbs :
         observed_corner_locations_with_uncertainty) {
      FrameId pose_idx = frame_and_bbs.first;
      if (pose_idx != max_frame) {
        //      if (pose_idx > max_frame) {
        continue;
      }

      // If we need images to display things, check if there are any images for
      // the robot pose (there should be if there were detected bounding boxes
      if (images.find(pose_idx) == images.end()) {
        continue;
      }

      std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
          images_for_pose = images.at(pose_idx);
      std::unordered_map<
          CameraId,
          std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>>
          observed_corner_locations_for_pose = frame_and_bbs.second;

      for (const auto &cam_id_and_intrinsics : intrinsics) {
        // If there are no observed bounding boxes for the camera at the pose,
        // sip it
        //        if (observed_corner_locations_for_pose.find(
        //                cam_id_and_intrinsics.first) ==
        //            observed_corner_locations_for_pose.end()) {
        //          continue;
        //        }

        std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>
            observed_corners_for_pose_and_cam;

        if (observed_corner_locations_for_pose.find(
                cam_id_and_intrinsics.first) !=
            observed_corner_locations_for_pose.end()) {
          observed_corners_for_pose_and_cam =
              observed_corner_locations_for_pose.at(
                  cam_id_and_intrinsics.first);
        }
        bool has_image_for_cam_at_pose =
            (images_for_pose.find(cam_id_and_intrinsics.first) !=
             images_for_pose.end());
        if (!has_image_for_cam_at_pose) {
          continue;
        }

        CameraIntrinsicsMat<double> cam_intrinsics =
            cam_id_and_intrinsics.second;
        std::pair<double, double> image_height_and_width =
            img_heights_and_widths.at(cam_id_and_intrinsics.first);

        sensor_msgs::Image::ConstPtr image =
            images_for_pose.at(cam_id_and_intrinsics.first);

        std::pair<std::string, std::string> image_and_camera_info_topics =
            createImageAndCameraInfoTopicsForTrajectoryTypeRobotPoseAndCameraId(
                pose_idx, cam_id_and_intrinsics.first, kAllBoundingBoxesPrefix);

        std::string frame_id = createCameraPoseFrameIdRelRobot(
            pose_idx,
            cam_id_and_intrinsics.first,
            prefixes_for_plot_type_.at(plot_type));

        publishImageWithDetectedBoundingBoxes(
            image_and_camera_info_topics.first,
            image_and_camera_info_topics.second,
            frame_id,
            cam_intrinsics,
            image_height_and_width,
            image,
            observed_corners_for_pose_and_cam,
            std::to_string(pose_idx));

        if (pose_idx == max_frame) {
          std::pair<std::pair<std::string, std::string>, std::string>
              last_image_and_camera_info_topics =
                  createLatestImageAndCameraInfoTopicsForTrajectoryTypeCameraId(
                      cam_id_and_intrinsics.first,
                      kBoundingBoxesImageLabel,
                      prefixes_for_plot_type_.at(plot_type),
                      kAllBoundingBoxesPrefix);
          publishImageWithDetectedBoundingBoxes(
              last_image_and_camera_info_topics.first.first,
              last_image_and_camera_info_topics.first.second,
              last_image_and_camera_info_topics.second,
              cam_intrinsics,
              image_height_and_width,
              image,
              observed_corners_for_pose_and_cam,
              std::to_string(pose_idx));
        }
      }
    }
  }

  void publishBoundingBoxDataFromTrajectory(
      const FrameId &max_frame,
      const std::unordered_map<FrameId, Pose3D<double>> &trajectory,
      const PlotType &plot_type,
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
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>
          &observed_corner_locations_with_opt_confidence,
      const std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<ObjectId, BbCornerPair<double>>>>
          &gt_corner_locations,
      const bool &display_boxes_if_no_image,
      const double &near_edge_threshold) {
    publishBoundingBoxDataFromTrajectory(
        max_frame,
        trajectory,
        prefixes_for_plot_type_.at(plot_type),
        initial_ellipsoid_estimates,
        optimized_ellipsoid_estimates,
        gt_ellipsoid_estimates,
        extrinsics,
        intrinsics,
        img_heights_and_widths,
        images,
        observed_corner_locations_with_opt_confidence,
        gt_corner_locations,
        display_boxes_if_no_image,
        near_edge_threshold);
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
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>
          &observed_corner_locations_with_opt_confidence,
      const std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<ObjectId, BbCornerPair<double>>>>
          &gt_corner_locations,
      const bool &display_boxes_if_no_image,
      const double &near_edge_threshold) {
    // For each robot pose
    //    for (FrameId pose_idx = 0; pose_idx <= max_frame; pose_idx++) {
    for (FrameId pose_idx = max_frame; pose_idx <= max_frame; pose_idx++) {
      Pose3D<double> robot_pose = trajectory.at(pose_idx);
      // If we need images to display things, check if there are any images for
      // the robot pose
      if (!display_boxes_if_no_image &&
          (images.find(pose_idx) == images.end())) {
        continue;
      }

      // If there are no observed bounding boxes for the pose, skip it
      //      if (observed_corner_locations_with_opt_confidence.find(pose_idx)
      //      ==
      //          observed_corner_locations_with_opt_confidence.end()) {
      //        continue;
      //      }

      std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
          empty_images_map;
      std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
          &images_for_pose = empty_images_map;
      if (images.find(pose_idx) != images.end()) {
        images_for_pose = images.at(pose_idx);
      }

      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId,
                                            std::pair<BbCornerPair<double>,
                                                      std::optional<double>>>>
          observed_corner_locations_for_pose;
      if (observed_corner_locations_with_opt_confidence.find(pose_idx) !=
          observed_corner_locations_with_opt_confidence.end()) {
        observed_corner_locations_for_pose =
            observed_corner_locations_with_opt_confidence.at(pose_idx);
      }

      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId, BbCornerPair<double>>>
          gt_corner_locations_for_pose;
      if (gt_corner_locations.find(pose_idx) != gt_corner_locations.end()) {
        gt_corner_locations_for_pose = gt_corner_locations.at(pose_idx);
      }

      for (const auto &cam_id_and_extrinsics : extrinsics) {
        //        // If there are no observed bounding boxes for the camera at
        //        the pose,
        //        // sip it
        //        if (observed_corner_locations_for_pose.find(
        //                cam_id_and_extrinsics.first) ==
        //            observed_corner_locations_for_pose.end()) {
        //          continue;
        //        }

        std::unordered_map<
            ObjectId,
            std::pair<BbCornerPair<double>, std::optional<double>>>
            observed_corners_for_pose_and_cam;
        if (observed_corner_locations_for_pose.find(
                cam_id_and_extrinsics.first) !=
            observed_corner_locations_for_pose.end()) {
          observed_corners_for_pose_and_cam =
              observed_corner_locations_for_pose.at(
                  cam_id_and_extrinsics.first);
        }

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

          //          publishImageWithBoundingBoxes(image_and_camera_info_topics.first,
          //                                        image_and_camera_info_topics.second,
          //                                        frame_id,
          //                                        cam_intrinsics,
          //                                        image_height_and_width,
          //                                        image,
          //                                        observed_corner_locations_for_pose.at(
          //                                            cam_id_and_extrinsics.first),
          //                                        gt_corners,
          //                                        predicted_corners_from_optimized,
          //                                        predicted_corners_from_initial,
          //                                        predicted_corners_from_gt,
          //                                        near_edge_threshold,
          //                                        std::to_string(pose_idx));

          if (pose_idx == max_frame) {
            std::pair<std::pair<std::string, std::string>, std::string>
                last_image_and_camera_info_topics =
                    createLatestImageAndCameraInfoTopicsForTrajectoryTypeCameraId(
                        cam_id_and_extrinsics.first,
                        kBoundingBoxesImageLabel,
                        trajectory_type_prefix,
                        trajectory_type_prefix);
            publishImageWithBoundingBoxes(
                last_image_and_camera_info_topics.first.first,
                last_image_and_camera_info_topics.first.second,
                last_image_and_camera_info_topics.second,
                cam_intrinsics,
                image_height_and_width,
                image,
                observed_corners_for_pose_and_cam,
                gt_corners,
                predicted_corners_from_optimized,
                predicted_corners_from_initial,
                predicted_corners_from_gt,
                near_edge_threshold,
                std::to_string(pose_idx));
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
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
      Eigen::Quaterniond ellipsoid_quat(Eigen::AngleAxisd(
          ellipsoid_est.second.pose_.yaw_, Eigen::Vector3d::UnitZ()));
#else
      Eigen::Quaterniond ellipsoid_quat(
          ellipsoid_est.second.pose_.orientation_);
#endif
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
      const std::optional<std::unordered_map<
          ObjectId,
          std::pair<BbCornerPair<double>, std::optional<double>>>>
          &observed_corner_locations,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &ground_truth_corner_locations,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &predicted_corner_locations_from_optimized,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &predicted_corner_locations_from_initial,
      const std::optional<std::unordered_map<ObjectId, BbCornerPair<double>>>
          &predicted_corner_locations_from_gt,
      const double &near_edge_threshold,
      const std::string &img_disp_text = "") {
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
      std::unordered_map<ObjectId, BbCornerPair<double>> observed_locations;
      std::unordered_map<ObjectId, double> confidences;
      for (const auto &observed_loc : observed_corner_locations.value()) {
        observed_locations[observed_loc.first] = observed_loc.second.first;
        if (observed_loc.second.second.has_value()) {
          confidences[observed_loc.first] = observed_loc.second.second.value();
        }
      }
      drawRectanglesOnImage(observed_locations,
                            observed_bounding_box_color_,
                            cv_ptr,
                            confidences);
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
    displayLinesFromEdges(near_edge_threshold, img_height_and_width, cv_ptr);
    optionallyDisplayLeftCornerTextOnImage(
        img_disp_text, img_height_and_width, cv_ptr);

    // Publish image and camera info

    image_pub.publish(cv_ptr->toImageMsg());
    publishCameraInfo(frame_id,
                      image_stamp,
                      intrinsics,
                      img_height_and_width,
                      intrinsics_pub);
  }

  void publishImageWithDetectedBoundingBoxes(
      const std::string &image_topic_id,
      const std::string &camera_info_topic_id,
      const std::string &frame_id,
      const CameraIntrinsicsMat<double>
          &intrinsics,  // Should we just take in cam info?
      const std::pair<double, double> &img_height_and_width,
      const sensor_msgs::Image::ConstPtr &image,
      const std::vector<std::pair<BbCornerPair<double>, std::optional<double>>>
          &observed_corner_locations,
      const std::string &img_disp_text) {
    // Get or create publishers
    ros::Publisher intrinsics_pub =
        getOrCreatePublisher<sensor_msgs::CameraInfo>(camera_info_topic_id,
                                                      kCameraInfoQueueSize);
    ros::Publisher image_pub = getOrCreatePublisher<sensor_msgs::Image>(
        image_topic_id, kCameraInfoQueueSize);

    // Create/convert image
    cv_bridge::CvImagePtr cv_ptr;
    ros::Time image_stamp;
    try {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv_ptr->header.frame_id = frame_id;
      image_stamp = ros::Time::now();
      cv_ptr->header.stamp = image_stamp;
    } catch (cv_bridge::Exception &e) {
      LOG(ERROR) << "cv_bridge exception: " << e.what();
      exit(1);
    }

    for (const auto &bounding_box_corners_entry : observed_corner_locations) {
      drawRectangleOnImage(bounding_box_corners_entry.first,
                           observed_bounding_box_color_,
                           std::nullopt,
                           bounding_box_corners_entry.second,
                           cv_ptr);
    }
    optionallyDisplayLeftCornerTextOnImage(
        img_disp_text, img_height_and_width, cv_ptr);

    // Publish image and camera info
    image_pub.publish(cv_ptr->toImageMsg());
    publishCameraInfo(frame_id,
                      image_stamp,
                      intrinsics,
                      img_height_and_width,
                      intrinsics_pub);
  }

  void displayLinesFromEdges(
      const double &near_edge_threshold,
      const std::pair<double, double> &img_height_and_width,
      cv_bridge::CvImagePtr &cv_ptr) {
    if (near_edge_threshold < 0) {
      return;
    }

    std_msgs::ColorRGBA line_color;
    line_color.a = line_color.r = line_color.g = line_color.b = 1;

    PixelCoord<double> px1(0, near_edge_threshold);
    PixelCoord<double> px2(img_height_and_width.second, near_edge_threshold);
    drawLineOnImage(px1, px2, line_color, cv_ptr);

    PixelCoord<double> px1b(0,
                            img_height_and_width.first - near_edge_threshold);
    PixelCoord<double> px2b(img_height_and_width.second,
                            img_height_and_width.first - near_edge_threshold);
    drawLineOnImage(px1b, px2b, line_color, cv_ptr);

    PixelCoord<int> px1c(near_edge_threshold, 0);
    PixelCoord<int> px2c(near_edge_threshold, img_height_and_width.first);
    drawLineOnImage(px1c, px2c, line_color, cv_ptr);

    PixelCoord<int> px1d(img_height_and_width.second - near_edge_threshold, 0);
    PixelCoord<int> px2d(img_height_and_width.second - near_edge_threshold,
                         img_height_and_width.first);
    drawLineOnImage(px1d, px2d, line_color, cv_ptr);
  }

  void visualizeFeatureEstimates(
      const std::unordered_map<FeatureId, Position3d<double>> &features,
      const PlotType &display_type) {
    // TODO make own queue size
    ros::Publisher point_cloud_pub =
        getOrCreatePublisher<sensor_msgs::PointCloud2>(
            createFeatureCloudTopicName(display_type), kCameraInfoQueueSize);
    std_msgs::ColorRGBA color = color_for_plot_type_.at(display_type);
    uint8_t r = color.r * 255;
    uint8_t g = color.g * 255;
    uint8_t b = color.b * 255;
    pcl::PointCloud<pcl::PointXYZRGB> feature_cloud;
    for (const auto &point : features) {
      pcl::PointXYZRGB pcl_point(r, g, b);
      pcl_point.x = point.second.x();
      pcl_point.y = point.second.y();
      pcl_point.z = point.second.z();
      feature_cloud.push_back(pcl_point);
    }
    feature_cloud.header.frame_id = kVizFrame;
    sensor_msgs::PointCloud2 ros_feature_cloud;
    pcl::toROSMsg(feature_cloud, ros_feature_cloud);
    point_cloud_pub.publish(ros_feature_cloud);
  }

  void publishLatestImageWithReprojectionResiduals(
      const std::pair<std::string, std::string>
          &img_and_camera_info_topic_names,
      const std::string &camera_frame_id,
      const CameraIntrinsicsMat<double> &intrinsics,
      const std::unordered_map<FeatureId, PixelCoord<double>> &observed_pixels,
      const std::unordered_map<FeatureId, PixelCoord<double>> &projected_pixels,
      const std::optional<sensor_msgs::Image::ConstPtr> &image,
      const std::pair<double, double> &img_height_and_width,
      const bool &display_pixels_if_no_image,
      const std::string &img_disp_text = "") {
    if (!display_pixels_if_no_image && !image.has_value()) {
      return;
    }

    ros::Publisher image_pub = getOrCreatePublisher<sensor_msgs::Image>(
        img_and_camera_info_topic_names.first, kCameraInfoQueueSize);

    ros::Publisher intrinsics_pub =
        getOrCreatePublisher<sensor_msgs::CameraInfo>(
            img_and_camera_info_topic_names.second, kCameraInfoQueueSize);

    // Create/convert image
    cv_bridge::CvImagePtr cv_ptr;
    ros::Time image_stamp;
    if (image.has_value()) {
      //      image_stamp = image.value()->header.stamp;
      try {
        cv_ptr = cv_bridge::toCvCopy(image.value(),
                                     sensor_msgs::image_encodings::BGR8);
        cv_ptr->header.frame_id = camera_frame_id;
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
      img_header.frame_id = camera_frame_id;
      cv_ptr = boost::make_shared<cv_bridge::CvImage>(
          img_header, sensor_msgs::image_encodings::BGR8, cv_img);
    }

    for (const auto &obs_pix : observed_pixels) {
      FeatureId feat_id = obs_pix.first;
      PixelCoord<double> observed_feat = obs_pix.second;
      drawTinyCircleOnImage(observed_feat,
                            brightenColor(color_for_plot_type_[INITIAL], 0.4),
                            cv_ptr);

      if (projected_pixels.find(feat_id) == projected_pixels.end()) {
        continue;
      }

      PixelCoord<double> projected_feat = projected_pixels.at(feat_id);
      drawTinyCircleOnImage(projected_feat,
                            brightenColor(color_for_plot_type_[ESTIMATED], 0.4),
                            cv_ptr);
      drawLineOnImage(
          observed_feat, projected_feat, residual_feature_color_, cv_ptr);
    }

    optionallyDisplayLeftCornerTextOnImage(
        img_disp_text, img_height_and_width, cv_ptr);

    LOG(INFO) << "Publishing image for frame " << camera_frame_id
              << " to topic " << image_pub.getTopic();
    image_pub.publish(cv_ptr->toImageMsg());

    publishCameraInfo(camera_frame_id,
                      image_stamp,
                      intrinsics,
                      img_height_and_width,
                      intrinsics_pub);
  }

  void publishLatestImageWithReprojectionResiduals(
      const FrameId &node_id,
      const CameraId &camera_id,
      const CameraIntrinsicsMat<double> &camera_intrinsics,
      const PlotType &trajectory_type,
      const std::unordered_map<FeatureId, PixelCoord<double>> &observed_pixels,
      const std::unordered_map<FeatureId, PixelCoord<double>> &projected_pixels,
      const std::optional<sensor_msgs::Image::ConstPtr> &image,
      const std::pair<double, double> &img_height_and_width,
      const bool &display_pixels_if_no_image) {
    std::string trajectory_type_prefix =
        prefixes_for_plot_type_.at(trajectory_type);
    std::pair<std::pair<std::string, std::string>, std::string>
        image_and_camera_info_topics =
            createLatestImageAndCameraInfoTopicsForTrajectoryTypeCameraId(
                camera_id,
                kLowLevelFeatsImageLabel,
                trajectory_type_prefix,
                trajectory_type_prefix);

    publishLatestImageWithReprojectionResiduals(
        image_and_camera_info_topics.first,
        image_and_camera_info_topics.second,
        camera_intrinsics,
        observed_pixels,
        projected_pixels,
        image,
        img_height_and_width,
        display_pixels_if_no_image,
        std::to_string(node_id));
  }

  void visualizeFrustum(const Pose3D<double> &robot_pose,
                        const CameraIntrinsicsMat<double> &intrinsics,
                        const CameraExtrinsics<double> &cam_extrinsics,
                        const std::pair<double, double> img_height_and_width,
                        const PlotType &plot_type) {
    Position3d<double> frustum_center =
        getWorldFramePos(PixelCoord<double>(intrinsics(0, 2), intrinsics(1, 2)),
                         intrinsics,
                         cam_extrinsics,
                         robot_pose,
                         (double)0);
    double frustrum_depth = 1.5;
    std::vector<Position3d<double>> frustum_points;
    frustum_points.emplace_back(getWorldFramePos(PixelCoord<double>(0, 0),
                                                 intrinsics,
                                                 cam_extrinsics,
                                                 robot_pose,
                                                 frustrum_depth));
    frustum_points.emplace_back(
        getWorldFramePos(PixelCoord<double>(0, img_height_and_width.first),
                         intrinsics,
                         cam_extrinsics,
                         robot_pose,
                         frustrum_depth));
    frustum_points.emplace_back(
        getWorldFramePos(PixelCoord<double>(img_height_and_width.second,
                                            img_height_and_width.first),
                         intrinsics,
                         cam_extrinsics,
                         robot_pose,
                         frustrum_depth));
    frustum_points.emplace_back(
        getWorldFramePos(PixelCoord<double>(img_height_and_width.second, 0),
                         intrinsics,
                         cam_extrinsics,
                         robot_pose,
                         frustrum_depth));
    std::vector<std::pair<Position3d<double>, Position3d<double>>> lines;
    for (size_t frustum_idx = 0; frustum_idx < 4; frustum_idx++) {
      Position3d<double> frustum_point = frustum_points[frustum_idx];
      lines.emplace_back(std::make_pair(frustum_point, frustum_center));
      lines.emplace_back(
          std::make_pair(frustum_point, frustum_points[(frustum_idx + 1) % 4]));
    }

    ros::Publisher marker_pub = getOrCreateVisMarkerPublisherAndClearPrevious(
        createTopicForPlotTypeAndBase(plot_type, kFrustumTopicSuffix),
        kRobotPoseMarkerPubQueueSize);
    publishLines(marker_pub,
                 color_for_plot_type_.at(plot_type),
                 lines,
                 next_frustum_marker_num_++);
  }

  void publishLongTermMap(
      const std::unordered_map<
          ObjectId,
          std::pair<
              std::string,
              std::pair<EllipsoidState<double>,
                        Covariance<double, kEllipsoidParamterizationSize>>>>
          &initial_ests_and_cov) {
    std::unordered_map<ObjectId, std::pair<std::string, EllipsoidState<double>>>
        ltm_ellipsoids;
    for (const auto &obj_info : initial_ests_and_cov) {
      ltm_ellipsoids[obj_info.first] =
          std::make_pair(obj_info.second.first, obj_info.second.second.first);
    }
    std::string topic =
        createTopicForPlotTypeAndBase(INITIAL, kEllipsoidTopicSuffix);
    LOG(INFO) << "Publishing ellipsoids for plot type " << topic;
    visualizeEllipsoids(ltm_ellipsoids, topic, ltm_obj_color_, false);

    //    int next_marker = ltm_ellipsoids.size();
    //    for (const auto &obj_info : initial_ests_and_cov) {
    //      next_marker = visualizeCovForObj(obj_info.second.second,
    //      next_marker);
    //    }
  }

 private:
  const static uint32_t kEllipsoidMarkerPubQueueSize = 1000;
  const static uint32_t kRobotPoseMarkerPubQueueSize = 7000;

  const static uint32_t kCameraInfoQueueSize = 100;

  const static constexpr double kSleepAfterPubCreationTime = 0.2;

  const static int kBoundingBoxLineThickness = 4;

  const std::string kVizFrame = "map";

  const static int kBoundingBoxMinCornerLabelXOffset = 0;
  const static int kBoundingBoxMinCornerLabelYOffset = -10;
  const static int kBoundingBoxMinCornerBelowLabelYOffset = -50;

  const static int kFrameNumLabelXOffset = 10;
  const static int kFrameNumLabelYOffsetFromBottom = 10;

  const static constexpr double kBoundingBoxLabelFontScale = 1.5;
  const static int kBoundingBoxLabelTextThickness = 2;
  const static int kPixelMarkerCircleRad = 3;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std_msgs::ColorRGBA ground_truth_bounding_box_color_;

  std_msgs::ColorRGBA observed_bounding_box_color_;

  std_msgs::ColorRGBA predicted_bounding_box_from_optimized_color_;

  std_msgs::ColorRGBA predicted_bounding_box_from_gt_color_;

  std_msgs::ColorRGBA predicted_bounding_box_from_initial_color_;

  std_msgs::ColorRGBA residual_feature_color_;

  std_msgs::ColorRGBA pending_obj_color_;

  std_msgs::ColorRGBA ltm_obj_color_;

  std::unordered_map<PlotType, std_msgs::ColorRGBA> color_for_plot_type_;
  std::unordered_map<PlotType, std::string> prefixes_for_plot_type_;

  // TODO maybe have another topic for LTM ellipsoids?
  const std::string kEllipsoidTopicSuffix = "ellipsoids";

  const std::string kPoseTopicSuffix = "pose";

  static const int32_t kTrajectoryMarkerNum = 1;
  const int32_t kMaxRobotPoseMarkerNum = std::numeric_limits<int32_t>::max();

  const double kTrajectoryScaleX = 0.02;

  const std::string kAllBoundingBoxesPrefix = "all_observed_bbs";
  const std::string kBoundingBoxesImageLabel = "bb";
  const std::string kLowLevelFeatsImageLabel = "feats";
  const std::string kFrameForLatestNode = "slam_base_link";
  const std::string kFrustumTopicSuffix = "frustums";

  const double kEllipsoidTextBuffer = 0.5;
  const int kEllipsoidLabelIdIncrement = 3000;

  std::unordered_map<std::string, std_msgs::ColorRGBA>
      colors_for_semantic_classes_;

  util_random::Random rand_gen_;

  int32_t next_frustum_marker_num_ = 0;

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

  ros::Publisher getOrCreateVisMarkerPublisherAndClearPrevious(
      const std::string &topic_name,
      const int &queue_size,
      const ros::Duration &sleep_after_create =
          ros::Duration(kSleepAfterPubCreationTime)) {
    if (publishers_by_topic_.find(topic_name) == publishers_by_topic_.end()) {
      ros::Publisher pub = node_handle_.advertise<visualization_msgs::Marker>(
          topic_name, queue_size);
      publishers_by_topic_[topic_name] = pub;
      //      sleep_after_create.sleep();
      publishDeleteAllMarker(pub);
    }
    return publishers_by_topic_[topic_name];
  }

  template <typename T>
  ros::Publisher getOrCreatePublisher(
      const std::string &topic_name,
      const int &queue_size,
      const ros::Duration &sleep_after_create =
          ros::Duration(kSleepAfterPubCreationTime)) {
    if (publishers_by_topic_.find(topic_name) == publishers_by_topic_.end()) {
      ros::Publisher pub = node_handle_.advertise<T>(topic_name, queue_size);
      publishers_by_topic_[topic_name] = pub;
      //      sleep_after_create.sleep();
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

  void publishDeleteAllMarker(ros::Publisher &marker_pub) {
    visualization_msgs::Marker marker_msg;
    if (marker_msg.header.frame_id.empty()) {
      marker_msg.header.frame_id = kVizFrame;
    }
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "ut_vslam";
    marker_msg.action = visualization_msgs::Marker::DELETEALL;
    //    ros::Duration(2).sleep();
    marker_pub.publish(marker_msg);
    ros::Duration(0.05).sleep();
  }

  int visualizeCovForObj(
      const std::pair<EllipsoidState<double>,
                      Covariance<double, kEllipsoidParamterizationSize>>
          &ellipsoid_cov_info,
      const int &next_marker) {
    Covariance<double, 2> x_y_cov =
        ellipsoid_cov_info.second.topLeftCorner<2, 2>();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(x_y_cov);
    Eigen::Vector2d unscaled_axis_lengths =
        eigen_solver.eigenvalues().array().sqrt().matrix();
    Eigen::Vector2d capped_axis_lengths = unscaled_axis_lengths;
    capped_axis_lengths.x() = std::min(4.0, capped_axis_lengths.x());
    capped_axis_lengths.y() = std::min(4.0, capped_axis_lengths.y());
    Eigen::Matrix2d eigVecs = eigen_solver.eigenvectors();
    double eigvec21 = eigVecs(1, 0);
    double eigvec11 = eigVecs(0, 0);
    double yaw = atan2(eigvec21, eigvec11);
    Position3d<double> center =
        ellipsoid_cov_info.first.pose_.transl_ -
        Eigen::Vector3d(0, 0, ellipsoid_cov_info.first.dimensions_.z() / 2);
    for (int i = 3; i >= 1; i--) {
      std_msgs::ColorRGBA color;
      color.a = 1;
      color.r = color.b = color.g = (50.0 + i * 60) / 255.0;

      visualizeCovCircleForObj(
          center, capped_axis_lengths, yaw, i, color, next_marker - 1 + i);
    }
    return next_marker + 3;
  }

  void visualizeCovCircleForObj(const Position3d<double> &center,
                                const Eigen::Vector2d &unscaled_axis_lengths,
                                const double &ellipsoid_yaw,
                                const int &uncertainty_mult,
                                const std_msgs::ColorRGBA &color,
                                const int &marker_num) {
    visualization_msgs::Marker marker;
    marker.scale.x = unscaled_axis_lengths.x() * uncertainty_mult;
    marker.scale.y = unscaled_axis_lengths.y() * uncertainty_mult;
    marker.scale.z = 0.005 - uncertainty_mult * 0.001;

    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();

    Eigen::Quaterniond ellipsoid_quat(
        Eigen::AngleAxisd(ellipsoid_yaw, Eigen::Vector3d::UnitZ()));

    marker.pose.orientation.x = ellipsoid_quat.x();
    marker.pose.orientation.y = ellipsoid_quat.y();
    marker.pose.orientation.z = ellipsoid_quat.z();
    marker.pose.orientation.w = ellipsoid_quat.w();

    marker.id = marker_num;

    marker.type = visualization_msgs::Marker::Type::CYLINDER;

    marker.color = color;
    ros::Publisher pub = getOrCreateVisMarkerPublisherAndClearPrevious(
        createTopicForPlotTypeAndBase(INITIAL, kEllipsoidTopicSuffix),
        kEllipsoidMarkerPubQueueSize);
    publishMarker(marker, pub);
  }

  void publishEllipsoid(const FullDOFEllipsoidState<double> &ellipsoid,
                        const ObjectId &obj_id,
                        ros::Publisher &pub,
                        const std_msgs::ColorRGBA &color,
                        const bool &visualize_id = true) {
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

    if (visualize_id) {
      visualization_msgs::Marker text_marker;

      text_marker.color = color;
      text_marker.id = obj_id + kEllipsoidLabelIdIncrement;
      text_marker.text = std::to_string(obj_id);
      text_marker.type = visualization_msgs::Marker::Type::TEXT_VIEW_FACING;
      text_marker.scale.z = 1;
      text_marker.pose.orientation.w = 1.0;
      text_marker.pose.position.x = ellipsoid.pose_.transl_.x();
      text_marker.pose.position.y = ellipsoid.pose_.transl_.y();
      // Consider taking max of all 3 dimensions instead of just height
      text_marker.pose.position.z = ellipsoid.pose_.transl_.z() +
                                    (ellipsoid.dimensions_.z() / 2) +
                                    kEllipsoidTextBuffer;
      publishMarker(text_marker, pub);
    }
  }
#ifdef CONSTRAIN_ELLIPSOID_ORIENTATION
  void publishEllipsoid(const EllipsoidState<double> &ellipsoid,
                        const ObjectId &obj_id,
                        ros::Publisher &pub,
                        const std_msgs::ColorRGBA &color,
                        const bool &visualize_id = true) {
    Pose3D<double> obj_pose(
        ellipsoid.pose_.transl_,
        Eigen::AngleAxisd(ellipsoid.pose_.yaw_, Eigen::Vector3d::UnitZ()));
    FullDOFEllipsoidState<double> full_dof_ellipsoid(obj_pose,
                                                     ellipsoid.dimensions_);
    publishEllipsoid(full_dof_ellipsoid, obj_id, pub, color, visualize_id);
  }
#endif

  std::string createTopicForPlotTypeAndBase(const PlotType &plot_type,
                                            const std::string &base_topic) {
    return topic_prefix_ + prefixes_for_plot_type_.at(plot_type) + base_topic;
  }

  void drawLineOnImage(const PixelCoord<int> &px1,
                       const PixelCoord<int> &px2,
                       const std_msgs::ColorRGBA &color,
                       cv_bridge::CvImagePtr &cv_ptr) {
    // TODO verify
    int thickness = 1;
    cv::Point p1(px1.x(), px1.y());
    cv::Point p2(px2.x(), px2.y());
    cv::line(
        cv_ptr->image, p1, p2, convertColorMsgToOpenCvColor(color), thickness);
  }

  void drawLineOnImage(const PixelCoord<double> &px1,
                       const PixelCoord<double> &px2,
                       const std_msgs::ColorRGBA &color,
                       cv_bridge::CvImagePtr &cv_ptr) {
    drawLineOnImage(
        (PixelCoord<int>)px1.cast<int>(), px2.cast<int>(), color, cv_ptr);
  }

  void drawRectanglesOnImage(
      const std::unordered_map<ObjectId, BbCornerPair<double>>
          &bounding_box_corners,
      const std_msgs::ColorRGBA &color,
      cv_bridge::CvImagePtr &cv_ptr,
      const std::unordered_map<ObjectId, double> &confidences = {}) {
    for (const auto &bounding_box_corners_entry : bounding_box_corners) {
      std::optional<double> confidence = std::nullopt;
      if (confidences.find(bounding_box_corners_entry.first) !=
          confidences.end()) {
        confidence = confidences.at(bounding_box_corners_entry.first);
      }
      drawRectangleOnImage(bounding_box_corners_entry.second,
                           color,
                           bounding_box_corners_entry.first,
                           confidence,
                           cv_ptr);
    }
  }

  void drawRectangleOnImage(
      const BbCornerPair<double> &bounding_box_corners,
      const std_msgs::ColorRGBA &color,
      const std::optional<ObjectId> &bounding_box_numeric_label,
      const std::optional<double> &confidence,
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
      cv::putText(
          cv_ptr->image,
          std::to_string(bounding_box_numeric_label.value()),
          cv::Point(std::max((double)0,
                             bounding_box_corners.first.x() +
                                 kBoundingBoxMinCornerLabelXOffset),
                    std::max((double)0,
                             bounding_box_corners.second.y() -
                                 kBoundingBoxMinCornerBelowLabelYOffset)),
          cv::FONT_HERSHEY_SIMPLEX,
          kBoundingBoxLabelFontScale,
          convertColorMsgToOpenCvColor(color),
          kBoundingBoxLabelTextThickness);
    }
    if (confidence.has_value()) {
      std::stringstream confidence_stream;
      confidence_stream << std::setprecision(3) << std::fixed
                        << confidence.value();
      cv::putText(cv_ptr->image,
                  confidence_stream.str(),
                  cv::Point(std::max((double)0,
                                     bounding_box_corners.first.x() +
                                         kBoundingBoxMinCornerLabelXOffset),
                            std::max((double)0,
                                     bounding_box_corners.second.y() +
                                         kBoundingBoxMinCornerLabelYOffset)),
                  cv::FONT_HERSHEY_SIMPLEX,
                  kBoundingBoxLabelFontScale,
                  convertColorMsgToOpenCvColor(color),
                  kBoundingBoxLabelTextThickness);
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

  int32_t publishTrajectory(
      ros::Publisher &marker_pub,
      const std_msgs::ColorRGBA &color,
      const std::vector<Pose3D<double>> &trajectory_poses,
      const int32_t &first_marker_id = kTrajectoryMarkerNum) {
    visualization_msgs::Marker marker_msg;
    marker_msg.id = first_marker_id;
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
                      first_marker_id + 1,
                      kMaxRobotPoseMarkerNum);
    return first_marker_id + 1 +
           trajectory_poses.size();  // Assumes 1 marker per pose
  }

  void publishRobotPoses(ros::Publisher &marker_pub,
                         const std::vector<Pose3D<double>> &robot_poses,
                         const std_msgs::ColorRGBA &color,
                         const int32_t min_id,
                         const int32_t max_id,
                         const bool &larger = false) {
    for (size_t i = 0; i < robot_poses.size(); i++) {
      publishRobotPose(marker_pub, robot_poses[i], color, min_id + i, larger);
    }
  }

  geometry_msgs::Transform convertPose3DtoTransformMsg(
      const Pose3D<double> &pose) {
    geometry_msgs::Transform transform;
    transform.translation.x = pose.transl_.x();
    transform.translation.y = pose.transl_.y();
    transform.translation.z = pose.transl_.z();
    Eigen::Quaterniond pose_orientation(pose.orientation_);
    transform.rotation.x = pose_orientation.x();
    transform.rotation.y = pose_orientation.y();
    transform.rotation.z = pose_orientation.z();
    transform.rotation.w = pose_orientation.w();
    return transform;
  }

  void publishRobotPose(ros::Publisher &marker_pub,
                        const Pose3D<double> &robot_pose,
                        const std_msgs::ColorRGBA &color,
                        const int32_t id,
                        bool bigger = false) {
    visualization_msgs::Marker marker_msg;

    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;

    if (bigger) {
      //                marker_msg.scale.x = 0.8;
      //                marker_msg.scale.y = 0.6;

      marker_msg.scale.x = 0.4;
      marker_msg.scale.y = 0.3;
      marker_msg.scale.z = 0.2;
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

  std_msgs::ColorRGBA getColorForClass(const std::string &semantic_class) {
    if (colors_for_semantic_classes_.find(semantic_class) ==
        colors_for_semantic_classes_.end()) {
      std_msgs::ColorRGBA color_for_class = generateRandomColor(rand_gen_);
      colors_for_semantic_classes_[semantic_class] = color_for_class;
    }
    return colors_for_semantic_classes_.at(semantic_class);
  }
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_REFACTORING_ROS_VISUALIZATION_H
