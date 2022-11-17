//
// Created by amanda on 10/10/22.
//

#ifndef UT_VSLAM_ORB_OUTPUT_LOW_LEVEL_FEATURE_READER_H
#define UT_VSLAM_ORB_OUTPUT_LOW_LEVEL_FEATURE_READER_H

#include <refactoring/visual_feature_processing/low_level_feature_reader.h>

namespace vslam_types_refactor {

struct FeatureObservationsForFrame {
  FrameId frame_id_;
  std::unordered_map<FeatureId,
                     std::unordered_map<CameraId, PixelCoord<double>>>
      features_;
};

struct FeatureFileContents {
  std::unordered_map<FeatureId, Position3d<double>>
      feature_initial_position_estimates_;
};

class OrbOutputLowLevelFeatureReader
    : public AbstractLowLevelFeatureReader<StructuredVisionFeatureTrack> {
 public:
  OrbOutputLowLevelFeatureReader(
      const std::string &orb_data_directory_name,
      const std::vector<CameraId> &camera_precedence_order)
      : AbstractLowLevelFeatureReader<StructuredVisionFeatureTrack>(),
        orb_data_directory_name_(orb_data_directory_name),
        camera_precedence_order_(camera_precedence_order),
        loaded_(false) {}

  /**
   * Read the low level features. Expects data as formatted by Taijing's branch
   * of ORB-SLAM.
   *
   * @param file_name[in]       Should be directory name that points to all of
   *                            the files for the dataset.
   * @param feature_tracks[out] Feature tracks that are read in from files.
   */
  virtual bool getLowLevelFeatures(
      std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
          &feature_tracks) override;

 protected:
  bool readSingleFileFrameContentsFromDirectory(
      const std::string &directory_name,
      std::unordered_map<FrameId, FeatureObservationsForFrame>
          &single_frame_feature_observations);

  bool readFeatureFileContentsFromDirectory(
      const std::string &directory_name,
      FeatureFileContents &feature_file_contents);

  bool loadData();

 private:
  const std::string kFeaturesFileLocation = "features/features.txt";

  std::string orb_data_directory_name_;
  std::vector<CameraId> camera_precedence_order_;

  bool loaded_;

  std::unordered_map<FeatureId, StructuredVisionFeatureTrack> feature_tracks_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_ORB_OUTPUT_LOW_LEVEL_FEATURE_READER_H
