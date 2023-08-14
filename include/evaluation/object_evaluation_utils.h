//
// Created by amanda on 8/12/23.
//

#ifndef UT_VSLAM_OBJECT_EVALUATION_UTILS_H
#define UT_VSLAM_OBJECT_EVALUATION_UTILS_H

#include <refactoring/output_problem_data.h>

namespace vslam_types_refactor {

void associateObjects(const EllipsoidResults &estimated_objects,
                      const EllipsoidResults &gt_objects,
                      std::unordered_map<ObjectId, std::optional<ObjectId>>
                          &gt_objects_for_est_objs);

void findTransformationForAssociatedObjects(
    const EllipsoidResults &estimated_objects,
    const EllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    EllipsoidPose<double> &est_obj_transformation);

void associateObjectsAndFindTransformation(
    const EllipsoidResults &estimated_objects,
    const EllipsoidResults &gt_objects,
    std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    Pose3D<double> &est_obj_transformation);

void alignEstObjects(const EllipsoidResults &estimated_objects,
                     const Pose3D<double> &est_obj_transformation,
                     EllipsoidResults &aligned_objects);

void getPositionDistancesList(
    const EllipsoidResults &aligned_estimated_objects,
    const EllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    const Pose3D<double> &est_obj_transformation,
    std::unordered_map<ObjectId, std::optional<double>> &dist_from_gt_by_obj);

void getIoUsForObjects(
    const EllipsoidResults &aligned_estimated_objects,
    const EllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    const Pose3D<double> &est_obj_transformation,
    std::unordered_map<ObjectId, std::optional<double>> &iou_per_obj);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OBJECT_EVALUATION_UTILS_H
