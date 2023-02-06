//
// Created by amanda on 1/16/23.
//

#ifndef UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H
#define UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H

#include <file_io/file_access_utils.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

struct SaveToFileBbFrontEndVisualizerConfig {
  FrameId feature_validity_window_ = INT_MAX;
  double bounding_box_inflation_size_ = 0;
};

struct SaveToFileVisualizerConfig {
  SaveToFileBbFrontEndVisualizerConfig bb_assoc_visualizer_config_;
};

class SaveToFileVisualizer {
 public:
  SaveToFileVisualizer(const std::string &output_directory,
                       const SaveToFileVisualizerConfig &config)
      : output_directory_(
            file_io::ensureDirectoryPathEndsWithSlash(output_directory)),
        config_(config) {}

  void boundingBoxFrontEndVisualization(
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>> &images,
      const std::shared_ptr<std::vector<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::pair<BbCornerPair<double>, double>>>>>
          &bounding_boxes_for_pending_object,
      const std::unordered_map<CameraId, CameraExtrinsics<double>> &extrinsics,
      const std::unordered_map<CameraId, CameraIntrinsicsMat<double>>
          &intrinsics,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::vector<std::pair<BbCornerPair<double>,
                                                   std::optional<double>>>>>>
          &all_observed_corner_locations_with_uncertainty,
      const std::shared_ptr<std::unordered_map<
          FrameId,
          std::unordered_map<
              CameraId,
              std::unordered_map<
                  ObjectId,
                  std::pair<BbCornerPair<double>, std::optional<double>>>>>>
          &observed_corner_locations,
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::unordered_map<FeatureId, PixelCoord<double>>>>
          &observed_features,
      const std::unordered_map<CameraId, std::pair<double, double>>
          &img_heights_and_widths,
      const FrameId &min_frame_optimized,
      const FrameId &max_frame_optimized) {
    if (output_directory_.empty()) {
      return;
    }
    // Want to visualize
    //   -- Detected bounding boxes
    //   -- Inflated detected bounding boxes
    //   -- Bounding boxes from past frames (pending + associated)
    //   -- Features in images (randomize color)

    // For each pose in the window, for each camera in window,
    // write image with pending bounding boxes,
    // expanded pending bounded boxes, features,
    // associated bounding boxes
    // For pending bounding boxes, also add number of observations associated?
    // Frame num and camera num

    // Write these to mosaic or different photos per file (w/ directory for the
    // frame)?

    for (FrameId frame_id = min_frame_optimized;
         frame_id <= max_frame_optimized;
         frame_id++) {
      for (const auto &cam_id_and_features : observed_features.at(frame_id)) {
        CameraId cam_id = cam_id_and_features.first;

        cv_bridge::CvImagePtr cv_ptr;
        ros::Time image_stamp;
        if ((images.find(frame_id) != images.end()) &&
            (images.at(frame_id).find(cam_id) != images.at(frame_id).end())) {
          try {
            cv_ptr = cv_bridge::toCvCopy(images.at(frame_id).at(cam_id),
                                         sensor_msgs::image_encodings::BGR8);
            image_stamp = ros::Time::now();
            cv_ptr->header.stamp = image_stamp;
          } catch (cv_bridge::Exception &e) {
            LOG(ERROR) << "cv_bridge exception: " << e.what();
            exit(1);
          }
        } else {
          // Create empty image
          // TODO: Is height/width correct?
          if (img_heights_and_widths.find(cam_id) ==
              img_heights_and_widths.end()) {
            LOG(WARNING)
                << "Image, nor image height and width existed for camera id "
                << cam_id << "; not visualizing";
            continue;
          }
          cv::Mat cv_img(img_heights_and_widths.at(cam_id).first,
                         img_heights_and_widths.at(cam_id).second,
                         CV_8UC3,
                         (0, 0, 0));  // Create black image
          image_stamp = ros::Time::now();
          std_msgs::Header img_header;
          img_header.stamp = image_stamp;
          cv_ptr = boost::make_shared<cv_bridge::CvImage>(
              img_header, sensor_msgs::image_encodings::BGR8, cv_img);
        }

        for (ObjectId pending_obj_id = 0;
             pending_obj_id < bounding_boxes_for_pending_object->size();
             pending_obj_id++) {
          std::unordered_map<
              FrameId,
              std::unordered_map<CameraId,
                                 std::pair<BbCornerPair<double>, double>>>
              bounding_boxes_for_object =
                  bounding_boxes_for_pending_object->at(pending_obj_id);
          if (bounding_boxes_for_object.find(frame_id) ==
              bounding_boxes_for_object.end()) {
            continue;
          }
          if (bounding_boxes_for_object.at(frame_id).find(cam_id) ==
              bounding_boxes_for_object.at(frame_id).end()) {
            continue;
          }
          BbCornerPair<double> detected_bounding_box =
              bounding_boxes_for_object.at(frame_id).at(cam_id).first;
          BbCornerPair<double> inflated_bounding_box = inflateBoundingBox(
              detected_bounding_box,
              config_.bb_assoc_visualizer_config_.bounding_box_inflation_size_);

          // TODO draw both detected and inflated bbs,
          // Add text for pending obj number
          // Add text for number of observations for that pending obj
        }

        bool display_objs = true;
        if (observed_corner_locations->find(frame_id) ==
            observed_corner_locations->end()) {
          display_objs = false;
        }
        if (observed_corner_locations->at(frame_id).find(cam_id) ==
            observed_corner_locations->at(frame_id).end()) {
          display_objs = false;
        }

        if (display_objs) {
          for (const auto &obj_and_bb :
               observed_corner_locations->at(frame_id).at(cam_id)) {
            BbCornerPair<double> detected_bounding_box =
                obj_and_bb.second.first;
            BbCornerPair<double> inflated_bounding_box =
                inflateBoundingBox(detected_bounding_box,
                                   config_.bb_assoc_visualizer_config_
                                       .bounding_box_inflation_size_);
            // TODO draw both detected and inflated bbs,
            // Add text for obj number
          }
        }

        for (const auto &feat : cam_id_and_features.second) {
          if (colors_for_features_.find(feat.first) ==
              colors_for_features_.end()) {
            colors_for_features_[feat.first] = generateRandomColor(rand_gen_);
          }
          drawTinyCircleOnImage(
              feat.second, colors_for_features_.at(feat.first), cv_ptr);
        }
      }
    }
  }

 private:
  std::string output_directory_;

  SaveToFileVisualizerConfig config_;

  util_random::Random rand_gen_;

  std::unordered_map<FeatureId, std_msgs::ColorRGBA> colors_for_features_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H
