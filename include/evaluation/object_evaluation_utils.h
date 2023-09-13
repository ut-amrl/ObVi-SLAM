//
// Created by amanda on 8/12/23.
//

#ifndef UT_VSLAM_OBJECT_EVALUATION_UTILS_H
#define UT_VSLAM_OBJECT_EVALUATION_UTILS_H

#include <refactoring/output_problem_data.h>
//#include <refactoring/visualization/ros_visualization.h>

namespace vslam_types_refactor {

struct SingleTrajectoryObjectMetrics {
  std::unordered_map<ObjectId, std::optional<ObjectId>> opt_gt_obj_for_est_obj_;
  std::unordered_map<ObjectId, double> iou_for_gt_obj_;
  std::unordered_map<ObjectId, std::optional<double>> pos_diff_for_est_obj_;

  int missed_gt_objs_;
  double objects_per_gt_obj_;

  double average_pos_deviation_;
  double avg_iou_;
  double median_pos_deviation_;
  double median_iou_;
};

struct FullSequenceObjectMetrics {
  std::vector<SingleTrajectoryObjectMetrics> indiv_trajectory_object_metrics_;
};

using FullDOFEllipsoidResults =
    std::unordered_map<ObjectId,
                       std::pair<std::string, FullDOFEllipsoidState<double>>>;

void associateObjects(
    const FullDOFEllipsoidResults &estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const bool &one_to_one,
    std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    const double &max_assoc_dist = std::numeric_limits<double>::infinity());

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
    Pose3D<double> &est_obj_transformation
//    , std::shared_ptr<RosVisualization> &vis_manager
    );

void adjustObjectFrame(const FullDOFEllipsoidResults &estimated_objects,
                       const Pose3D<double> &est_obj_transformation,
                       FullDOFEllipsoidResults &aligned_objects);

void getPositionDistancesList(
    const FullDOFEllipsoidResults &aligned_estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    std::unordered_map<ObjectId, std::optional<double>> &dist_from_gt_by_obj);

std::pair<Eigen::Vector3d, Eigen::Vector3d>
getAxisAlignedBoundingBoxForEllipsoid(
    const FullDOFEllipsoidState<double> &ellipsoid);

bool pointInEllipsoid(const FullDOFEllipsoidState<double> &ellipsoid,
                      const Position3d<double> &point);

bool boundingBoxesOverlap(
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> &bb_1,
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> &bb_2);

double getIoUForObjectSet(
    const FullDOFEllipsoidState<double> &ellipsoid1,
    const std::vector<FullDOFEllipsoidState<double>> &covering_ellipsoids
//    ,const ObjectId &e1_id,
//    const std::vector<ObjectId> &e2_ids,
//    std::shared_ptr<RosVisualization> &vis_manager
    );

void getIoUsForObjects(
    const FullDOFEllipsoidResults &aligned_estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    std::unordered_map<ObjectId, double> &iou_per_gt_obj
//    , std::shared_ptr<RosVisualization> &vis_manager
    );

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OBJECT_EVALUATION_UTILS_H
