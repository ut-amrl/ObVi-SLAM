//
// Created by amanda on 6/19/23.
//

#ifndef UT_VSLAM_POSE_GRAPH_UTILS_H
#define UT_VSLAM_POSE_GRAPH_UTILS_H

#include <refactoring/bounding_box_frontend/bounding_box_front_end.h>
#include <refactoring/optimization/object_pose_graph.h>

#include <unordered_map>

namespace vslam_types_refactor {

template <typename ObjectAssociationInfo,
          typename PendingObjectInfo,
          typename RawBoundingBoxContextInfo,
          typename RefinedBoundingBoxContextInfo,
          typename SingleBbContextInfo,
          typename FrontEndObjMapData,
          typename PoseGraphType>
bool mergeObjects(
    const std::unordered_map<ObjectId, std::unordered_set<ObjectId>>
        &objects_to_merge,
    const std::shared_ptr<PoseGraphType> &pose_graph,
    const std::shared_ptr<
        AbstractBoundingBoxFrontEnd<ReprojectionErrorFactor,
                                    ObjectAssociationInfo,
                                    PendingObjectInfo,
                                    RawBoundingBoxContextInfo,
                                    RefinedBoundingBoxContextInfo,
                                    SingleBbContextInfo,
                                    FrontEndObjMapData>> &obj_front_end) {
  LOG(INFO) << "Merging the following objects";
  for (const auto &merge_group : objects_to_merge) {
    std::string merging_in_str;
    for (const ObjectId &merging_in : merge_group.second) {
      merging_in_str += std::to_string(merging_in);
      merging_in_str += ", ";
    }
    LOG(INFO) << "Merging objects " << merging_in_str << " into object "
              << merge_group.first;
  }
  return obj_front_end->mergeObjects(objects_to_merge) &&
         pose_graph->mergeObjects(objects_to_merge);
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_POSE_GRAPH_UTILS_H
