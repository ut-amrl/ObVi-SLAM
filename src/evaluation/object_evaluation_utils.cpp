//
// Created by amanda on 8/12/23.
//

#include <evaluation/object_evaluation_utils.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

void associateObjects(const EllipsoidResults &estimated_objects,
                      const EllipsoidResults &gt_objects,
                      std::unordered_map<ObjectId, std::optional<ObjectId>>
                      &gt_objects_for_est_objs) {

}

void findTransformationForAssociatedObjects(const EllipsoidResults &estimated_objects,
                                            const EllipsoidResults &gt_objects,
                                            const std::unordered_map<ObjectId, std::optional<ObjectId>>
                                            &gt_objects_for_est_objs,
                                            EllipsoidPose<double> &est_obj_transformation) {
  Position3d<double> mean_pos_est = Eigen::Vector3d::Zero();
  Position3d<double> mean_pos_gt = Eigen::Vector3d::Zero();


  size_t total_poses = 0;
  for (const auto & gt_obj_for_est : gt_objects_for_est_objs) {
    if (!gt_obj_for_est.second.has_value()) {
      continue;
    }
    total_poses++;
    mean_pos_gt += gt_objects.ellipsoids_.at(gt_obj_for_est.second.value()).second.pose_.transl_;
    mean_pos_est += estimated_objects.ellipsoids_.at(gt_obj_for_est.first).second.pose_.transl_;
  }
  mean_pos_gt = mean_pos_gt / total_poses;
  mean_pos_est = mean_pos_est / total_poses;



}

void associateObjectsAndFindTransformation(const EllipsoidResults &estimated_objects,
                      const EllipsoidResults &gt_objects,
                      std::unordered_map<ObjectId, std::optional<ObjectId>>
                          &gt_objects_for_est_objs,
                      EllipsoidPose<double> &est_obj_transformation) {}

void alignEstObjects(const EllipsoidResults &estimated_objects,
                     const EllipsoidPose<double> &est_obj_transformation,
                     EllipsoidResults &aligned_objects) {
  for (const auto &ellipsoid_entry : estimated_objects.ellipsoids_) {
    aligned_objects.ellipsoids_[ellipsoid_entry.first] = std::make_pair(
        ellipsoid_entry.second.first,
        EllipsoidState(combinePoses(est_obj_transformation,
                                    ellipsoid_entry.second.second.pose_),
                       ellipsoid_entry.second.second.dimensions_));
  }
}

void getPositionDistancesList(
    const EllipsoidResults &aligned_estimated_objects,
    const EllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    const EllipsoidPose<double> &est_obj_transformation,
    std::unordered_map<ObjectId, std::optional<double>> &dist_from_gt_by_obj) {
  for (const auto &gt_obj_for_est_obj : gt_objects_for_est_objs) {
    ObjectId est_obj_id = gt_obj_for_est_obj.first;
    std::optional<double> dist_from_gt;
    if (gt_obj_for_est_obj.second.has_value()) {
      Pose3DYawOnly<double> gt_obj_pose =
          gt_objects.ellipsoids_.at(gt_obj_for_est_obj.second.value())
              .second.pose_;
      Pose3DYawOnly<double> aligned_est_obj_pose =
          aligned_estimated_objects.ellipsoids_.at(est_obj_id).second.pose_;
      dist_from_gt =
          (gt_obj_pose.transl_ - aligned_est_obj_pose.transl_).norm();
    }
    dist_from_gt_by_obj[est_obj_id] = dist_from_gt;
  }
}

void getIoUsForObjects(
    const EllipsoidResults &aligned_estimated_objects,
    const EllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    const EllipsoidPose<double> &est_obj_transformation,
    std::unordered_map<ObjectId, std::optional<double>> &iou_per_obj) {}
}  // namespace vslam_types_refactor