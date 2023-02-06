//
// Created by amanda on 1/16/23.
//

#ifndef UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H
#define UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H

#include <base_lib/basic_utils.h>
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
        config_(config) {
    associated_bb_color_.a = 1;
    associated_bb_color_.r = 1;

    inflated_associated_bb_color_ = brightenColor(associated_bb_color_, .3);

    pending_bb_color_.a = 1;
    pending_bb_color_.r = 186.0 / 255;
    pending_bb_color_.b = 247.0 / 255;

    pending_inflated_bb_color_ = brightenColor(pending_bb_color_, .3);
  }

  void boundingBoxFrontEndVisualization(
      const std::unordered_map<
          FrameId,
          std::unordered_map<CameraId, sensor_msgs::Image::ConstPtr>> &images,
      const std::shared_ptr<std::vector<std::unordered_map<
          FrameId,
          std::unordered_map<CameraId,
                             std::pair<BbCornerPair<double>, double>>>>>
          &bounding_boxes_for_pending_object,
      const std::vector<size_t> &num_obs_per_pending_obj,
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

    std::string mosaic_file_name =
        output_directory_ + "bounding_boxes_" +
        util::padIntNumberToConstantWidth(max_frame_optimized, 4) +
        kPngExtension;

    // For each pose in the window, for each camera in window,
    // write image with pending bounding boxes,
    // expanded pending bounded boxes, features,
    // associated bounding boxes
    // For pending bounding boxes, also add number of observations associated?
    // Frame num and camera num
    std::vector<cv::Mat> indiv_visualizations;

    for (FrameId frame_id = min_frame_optimized;
         frame_id <= max_frame_optimized;
         frame_id++) {
      for (const auto &cam_id_and_features : observed_features.at(frame_id)) {
        CameraId cam_id = cam_id_and_features.first;

        cv_bridge::CvImagePtr cv_ptr;
        ros::Time image_stamp;
        std::pair<double, double> img_height_and_width;
        if ((images.find(frame_id) != images.end()) &&
            (images.at(frame_id).find(cam_id) != images.at(frame_id).end())) {
          sensor_msgs::Image::ConstPtr image = images.at(frame_id).at(cam_id);
          img_height_and_width = std::make_pair(image->height, image->width);
          try {
            cv_ptr =
                cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
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
          img_height_and_width = img_heights_and_widths.at(cam_id);
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

        optionallyDisplayLeftCornerTextOnImage(
            std::to_string(frame_id), img_height_and_width, cv_ptr);

        optionallyDisplayRightCornerTextOnImage(
            std::to_string(cam_id), img_height_and_width, cv_ptr);

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

          displayBoundingBoxOnImage(inflated_bounding_box,
                                    img_height_and_width,
                                    pending_inflated_bb_color_,
                                    cv_ptr,
                                    std::nullopt,
                                    std::nullopt,
                                    std::nullopt,
                                    std::nullopt);
          displayBoundingBoxOnImage(
              detected_bounding_box,
              img_height_and_width,
              pending_bb_color_,
              cv_ptr,
              std::nullopt,
              std::to_string(num_obs_per_pending_obj.at(pending_obj_id)),
              std::to_string(pending_obj_id),
              std::nullopt);
        }

        bool display_objs = true;
        if (observed_corner_locations->find(frame_id) ==
            observed_corner_locations->end()) {
          display_objs = false;
        } else if (observed_corner_locations->at(frame_id).find(cam_id) ==
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

            displayBoundingBoxOnImage(inflated_bounding_box,
                                      img_height_and_width,
                                      inflated_associated_bb_color_,
                                      cv_ptr,
                                      std::nullopt,
                                      std::nullopt,
                                      std::nullopt,
                                      std::nullopt);
            displayBoundingBoxOnImage(detected_bounding_box,
                                      img_height_and_width,
                                      associated_bb_color_,
                                      cv_ptr,
                                      std::nullopt,
                                      std::nullopt,
                                      std::to_string(obj_and_bb.first),
                                      std::nullopt);
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
        indiv_visualizations.emplace_back(cv_ptr->image);
      }
    }
    cv::Mat mosaic = generateMosaic(indiv_visualizations, 4);
    cv::imwrite(mosaic_file_name, mosaic);
  }

 private:
  std::string output_directory_;

  SaveToFileVisualizerConfig config_;

  util_random::Random rand_gen_;

  std::unordered_map<FeatureId, std_msgs::ColorRGBA> colors_for_features_;

  std_msgs::ColorRGBA pending_bb_color_;
  std_msgs::ColorRGBA pending_inflated_bb_color_;
  std_msgs::ColorRGBA associated_bb_color_;
  std_msgs::ColorRGBA inflated_associated_bb_color_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H
