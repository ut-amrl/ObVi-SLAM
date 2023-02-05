//
// Created by amanda on 1/16/23.
//

#ifndef UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H
#define UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H

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
      : output_directory_(output_directory), config_(config) {}

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
      const FrameId &min_frame_optimized,
      const FrameId &max_frame_optimized) const {
    if (output_directory_.empty()) {
      return;
    }
    // Want to visualize
    //   -- Detected bounding boxes
    //   -- Inflated detected bounding boxes
    //   -- Bounding boxes from past frames (pending + associated)
    //   -- Features in images (randomize color)
  }

 private:
  std::string output_directory_;

  SaveToFileVisualizerConfig config_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_SAVE_TO_FILE_VISUALIZATION_H
