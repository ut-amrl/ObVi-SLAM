//
// Created by amanda on 12/10/22.
//

#ifndef UT_VSLAM_BOUNDING_BOX_RETRIEVER_H
#define UT_VSLAM_BOUNDING_BOX_RETRIEVER_H

#include <amrl_msgs/ObjectDetectionSrv.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <ros/ros.h>

namespace vslam_types_refactor {

template <typename InputProblemData>
bool retrievePrecomputedBoundingBoxes(
    const FrameId &frame_to_query_for,
    const InputProblemData &input_problem_data,
    std::unordered_map<CameraId, std::vector<RawBoundingBox>>
        &bounding_boxes_for_frame) {
  // Add bounding box observations
  std::unordered_map<FrameId,
                     std::unordered_map<CameraId, std::vector<RawBoundingBox>>>
      bb_obs = input_problem_data.getBoundingBoxes();
  if (bb_obs.find(frame_to_query_for) != bb_obs.end()) {
    bounding_boxes_for_frame = bb_obs.at(frame_to_query_for);
  }
  return true;
}

class YoloBoundingBoxQuerier {
 public:
  YoloBoundingBoxQuerier(
      ros::NodeHandle &node_handle,
      const std::string &query_service_name = "/yolov5_detect_objs")
      : node_handle_(node_handle), service_name_(query_service_name) {
    if (!regenerateClient()) {
      exit(1);
    }
  }

  bool retrieveBoundingBoxesFromCamIdsAndImages(
      const FrameId &frame_to_query_for,
      const std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
          &cam_ids_and_images,
      std::unordered_map<CameraId, std::vector<RawBoundingBox>>
          &bounding_boxes_for_frame) {
    return retrieveBoundingBoxHelper(
        frame_to_query_for, cam_ids_and_images, bounding_boxes_for_frame);
  }

  template <typename InputProblemData>
  bool retrieveBoundingBoxes(
      const FrameId &frame_to_query_for,
      const InputProblemData &input_problem_data,
      std::unordered_map<CameraId, std::vector<RawBoundingBox>>
          &bounding_boxes_for_frame) {
    std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
        images_by_cam_for_frame =
            input_problem_data.getImagesByCameraForFrame(frame_to_query_for);
    return retrieveBoundingBoxHelper(
        frame_to_query_for, images_by_cam_for_frame, bounding_boxes_for_frame);
  }

  // template <typename InputProblemData>
  // bool retrieveBoundingBoxes(
  //     const FrameId &frame_to_query_for,
  //     const InputProblemData &input_problem_data,
  //     std::unordered_map<CameraId, std::vector<RawBoundingBox>>
  //         &bounding_boxes_for_frame) {
  //   std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
  //       images_by_cam_for_frame =
  //           input_problem_data.getImagesByCameraForFrame(frame_to_query_for);

  //   for (const auto &cam_id_and_img : images_by_cam_for_frame) {
  //     amrl_msgs::ObjectDetectionSrv obj_det_srv_call;
  //     sensor_msgs::Image::ConstPtr curr_img = cam_id_and_img.second;
  //     obj_det_srv_call.request.query_image = *curr_img;
  //     if (!bounding_box_client_.isValid()) {
  //       if (!regenerateClient()) {
  //         LOG(ERROR) << "Tried to regenerate bounding box query client, but "
  //                       "failed. Exiting. ";
  //         exit(1);
  //       }
  //     }

  //     if (bounding_box_client_.call(obj_det_srv_call)) {
  //       std::vector<RawBoundingBox> bounding_boxes_for_camera;
  //       for (size_t bb_idx = 0;
  //            bb_idx < obj_det_srv_call.response.bounding_boxes.bboxes.size();
  //            bb_idx++) {
  //         amrl_msgs::BBox2DMsg bb =
  //             obj_det_srv_call.response.bounding_boxes.bboxes[bb_idx];
  //         RawBoundingBox bounding_box;
  //         bool valid_corners = true;
  //         for (size_t corner_idx = 0; corner_idx < 4; corner_idx++) {
  //           if (bb.xyxy[corner_idx] < 0) {
  //             LOG(WARNING) << "Bounding box corner pixel has coordinate less
  //             "
  //                             "than 0. Discarding";
  //             valid_corners = false;
  //             break;
  //           }
  //           double corner_max_dimension =
  //               (corner_idx % 2) ? curr_img->height : curr_img->width;
  //           if (bb.xyxy[corner_idx] >= corner_max_dimension) {
  //             LOG(WARNING) << "Bounding box corner pixel has coordinate "
  //                          << bb.xyxy[corner_idx]
  //                          << " higher than max dimension "
  //                          << corner_max_dimension << ". Discarding";
  //             valid_corners = false;
  //             break;
  //           }
  //         }
  //         if (!valid_corners) {
  //           continue;
  //         }
  //         bounding_box.semantic_class_ = bb.label;
  //         bounding_box.detection_confidence_ = bb.conf;
  //         bounding_box.pixel_corner_locations_ =
  //             std::make_pair(PixelCoord<double>(bb.xyxy[0], bb.xyxy[1]),
  //                            PixelCoord<double>(bb.xyxy[2], bb.xyxy[3]));
  //         bounding_boxes_for_camera.emplace_back(bounding_box);
  //       }
  //       bounding_boxes_for_frame[cam_id_and_img.first] =
  //           bounding_boxes_for_camera;
  //     } else {
  //       // TODO would it be better to have this silently fail in case calls
  //       for
  //       // other cameras succeed? Seems unlikely that this would be a super
  //       // transient thing
  //       LOG(WARNING) << "Couldn't get bounding boxes for frame id "
  //                    << frame_to_query_for << " and camera "
  //                    << cam_id_and_img.first;
  //       return false;
  //     }
  //   }
  //   return true;
  // }

 private:
  constexpr const static double kWaitForServiceMaxDuration = 10;

  ros::NodeHandle node_handle_;
  std::string service_name_;
  ros::ServiceClient bounding_box_client_;

  bool regenerateClient() {
    if (!ros::service::waitForService(
            service_name_, ros::Duration(kWaitForServiceMaxDuration))) {
      LOG(WARNING) << "Bounding box query service " << service_name_
                   << " never became available.";
      return false;
    }

    bounding_box_client_ =
        node_handle_.serviceClient<amrl_msgs::ObjectDetectionSrv>(service_name_,
                                                                  true);
    if (!bounding_box_client_.isValid()) {
      LOG(WARNING) << "Regenerated client, but still wasn't valid";
      return false;
    }
    return true;
  }

  bool retrieveBoundingBoxHelper(
      const FrameId &frame_to_query_for,
      const std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>
          &cam_ids_and_images,
      std::unordered_map<CameraId, std::vector<RawBoundingBox>>
          &bounding_boxes_for_frame) {
    for (const auto &cam_id_and_img : cam_ids_and_images) {
      amrl_msgs::ObjectDetectionSrv obj_det_srv_call;
      sensor_msgs::Image::ConstPtr curr_img = cam_id_and_img.second;
      obj_det_srv_call.request.query_image = *curr_img;
      if (!bounding_box_client_.isValid()) {
        if (!regenerateClient()) {
          LOG(ERROR) << "Tried to regenerate bounding box query client, but "
                        "failed. Exiting. ";
          exit(1);
        }
      }
      if (bounding_box_client_.call(obj_det_srv_call)) {
        std::vector<RawBoundingBox> bounding_boxes_for_camera;
        for (size_t bb_idx = 0;
             bb_idx < obj_det_srv_call.response.bounding_boxes.bboxes.size();
             bb_idx++) {
          amrl_msgs::BBox2DMsg bb =
              obj_det_srv_call.response.bounding_boxes.bboxes[bb_idx];
          RawBoundingBox bounding_box;
          bool valid_corners = true;
          for (size_t corner_idx = 0; corner_idx < 4; corner_idx++) {
            if (bb.xyxy[corner_idx] < 0) {
              LOG(WARNING) << "Bounding box corner pixel has coordinate less "
                              "than 0. Discarding";
              valid_corners = false;
              break;
            }
            double corner_max_dimension =
                (corner_idx % 2) ? curr_img->height : curr_img->width;
            if (bb.xyxy[corner_idx] >= corner_max_dimension) {
              LOG(WARNING) << "Bounding box corner pixel has coordinate "
                           << bb.xyxy[corner_idx]
                           << " higher than max dimension "
                           << corner_max_dimension << ". Discarding";
              valid_corners = false;
              break;
            }
          }
          if (!valid_corners) {
            continue;
          }
          bounding_box.semantic_class_ = bb.label;
          bounding_box.detection_confidence_ = bb.conf;
          bounding_box.pixel_corner_locations_ =
              std::make_pair(PixelCoord<double>(bb.xyxy[0], bb.xyxy[1]),
                             PixelCoord<double>(bb.xyxy[2], bb.xyxy[3]));
          bounding_boxes_for_camera.emplace_back(bounding_box);
        }
        bounding_boxes_for_frame[cam_id_and_img.first] =
            bounding_boxes_for_camera;
      } else {
        // TODO would it be better to have this silently fail in case calls for
        // other cameras succeed? Seems unlikely that this would be a super
        // transient thing
        LOG(WARNING) << "Couldn't get bounding boxes for frame id "
                     << frame_to_query_for << " and camera "
                     << cam_id_and_img.first;
        return false;
      }
    }
    return true;
  }
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_BOUNDING_BOX_RETRIEVER_H