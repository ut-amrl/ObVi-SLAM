//
// Created by amanda on 8/12/23.
//

#ifndef UT_VSLAM_OBJECT_EVALUATION_UTILS_H
#define UT_VSLAM_OBJECT_EVALUATION_UTILS_H

#include <refactoring/output_problem_data.h>

namespace vslam_types_refactor {

using FullDOFEllipsoidResults =
    std::unordered_map<ObjectId,
                       std::pair<std::string, FullDOFEllipsoidState<double>>>;

void associateObjects(const FullDOFEllipsoidResults &estimated_objects,
                      const FullDOFEllipsoidResults &gt_objects,
                      const bool &one_to_one,
                      std::unordered_map<ObjectId, std::optional<ObjectId>>
                          &gt_objects_for_est_objs);

void findTransformationForAssociatedObjects(
    const FullDOFEllipsoidResults &estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    Pose3D<double> &est_obj_transformation);

void associateObjectsAndFindTransformation(
    const FullDOFEllipsoidResults &estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const bool &one_to_one,
    std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    Pose3D<double> &est_obj_transformation);

void alignEstObjects(const FullDOFEllipsoidResults &estimated_objects,
                     const Pose3D<double> &est_obj_transformation,
                     FullDOFEllipsoidResults &aligned_objects);

void getPositionDistancesList(
    const FullDOFEllipsoidResults &aligned_estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    const Pose3D<double> &est_obj_transformation,
    std::unordered_map<ObjectId, std::optional<double>> &dist_from_gt_by_obj);

void getIoUsForObjects(
    const FullDOFEllipsoidResults &aligned_estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    const Pose3D<double> &est_obj_transformation,
    std::unordered_map<ObjectId, std::optional<double>> &iou_per_obj);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OBJECT_EVALUATION_UTILS_H
