//
// Created by amanda on 6/22/22.
//

#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {

bool operator==(const RawBoundingBox &bb_1, const RawBoundingBox &bb_2) {
  return (bb_1.semantic_class_ == bb_2.semantic_class_) &&
         (bb_1.pixel_corner_locations_ == bb_2.pixel_corner_locations_) &&
         (bb_1.detection_confidence_ == bb_2.detection_confidence_);
}
}  // namespace vslam_types_refactor