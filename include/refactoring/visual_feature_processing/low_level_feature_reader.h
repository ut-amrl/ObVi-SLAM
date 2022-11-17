//
// Created by amanda on 10/10/22.
//

#ifndef UT_VSLAM_LOW_LEVEL_FEATURE_READER_H
#define UT_VSLAM_LOW_LEVEL_FEATURE_READER_H

#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

#include <fstream>
#include <sstream>
#include <string>

namespace vslam_types_refactor {
template <typename FeatureTrackType>
class AbstractLowLevelFeatureReader {
  virtual bool getLowLevelFeatures(
      std::unordered_map<FeatureId, FeatureTrackType> &feature_tracks) = 0;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LOW_LEVEL_FEATURE_READER_H
