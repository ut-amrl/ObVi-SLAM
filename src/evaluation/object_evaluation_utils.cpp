//
// Created by amanda on 8/12/23.
//

#include <evaluation/object_evaluation_utils.h>
#include <refactoring/types/ellipsoid_utils.h>
#include <refactoring/types/vslam_types_math_util.h>
#include <util/random.h>

#include <iostream>

namespace vslam_types_refactor {

namespace {
const static int kObjectAlignmentMaxIters = 20;
const static double kRotationChangeForConvergence = 0.01;
const static double kTranslationChangeForConvergence = 0.05;
const static double kIoUSamplingPointsPerMeter = 20;
const static double kMaxDistanceForInliers = 5.0;
const static int kMaxPointsPerDirection = 1000;
}  // namespace

void associateObjects(const FullDOFEllipsoidResults &estimated_objects,
                      const FullDOFEllipsoidResults &gt_objects,
                      const bool &one_to_one,
                      std::unordered_map<ObjectId, std::optional<ObjectId>>
                          &gt_objects_for_est_objs,
                      const double &max_assoc_dist) {
  std::unordered_map<std::string, std::unordered_set<ObjectId>>
      est_obj_by_class;
  std::unordered_map<std::string, std::unordered_set<ObjectId>> gt_obj_by_class;

  for (const auto &est_obj : estimated_objects) {
    if (est_obj_by_class.find(est_obj.second.first) == est_obj_by_class.end()) {
      est_obj_by_class[est_obj.second.first] = {};
    }
    est_obj_by_class[est_obj.second.first].insert(est_obj.first);
    gt_objects_for_est_objs[est_obj.first] = std::nullopt;
  }

  for (const auto &gt_obj : gt_objects) {
    if (gt_obj_by_class.find(gt_obj.second.first) == gt_obj_by_class.end()) {
      gt_obj_by_class[gt_obj.second.first] = {};
    }
    gt_obj_by_class[gt_obj.second.first].insert(gt_obj.first);
  }

  if (one_to_one) {
    std::vector<std::pair<double, std::pair<ObjectId, ObjectId>>> obj_distances;
    for (const auto &est_obj : estimated_objects) {
      Position3d<double> est_obj_pos =
          estimated_objects.at(est_obj.first).second.pose_.transl_;
      std::unordered_set<ObjectId> gt_objs_for_class =
          gt_obj_by_class.at(est_obj.second.first);
      for (const ObjectId &gt_obj : gt_objs_for_class) {
        Position3d<double> gt_pos = gt_objects.at(gt_obj).second.pose_.transl_;
        double norm = (gt_pos - est_obj_pos).norm();
        obj_distances.emplace_back(
            std::make_pair(norm, std::make_pair(est_obj.first, gt_obj)));
      }
    }

    std::sort(
        obj_distances.begin(),
        obj_distances.end(),
        util::sort_pair_by_first<double, std::pair<ObjectId, ObjectId>>());

    std::unordered_set<ObjectId> assigned_gt_objs;

    for (const std::pair<double, std::pair<ObjectId, ObjectId>> &norm_for_objs :
         obj_distances) {
      if (norm_for_objs.first > max_assoc_dist) {
        break;
      }
      ObjectId est_obj = norm_for_objs.second.first;
      ObjectId gt_obj = norm_for_objs.second.second;

      if (assigned_gt_objs.find(gt_obj) != assigned_gt_objs.end()) {
        continue;
      }
      if ((gt_objects_for_est_objs.find(est_obj) !=
           gt_objects_for_est_objs.end()) &&
          (gt_objects_for_est_objs.at(est_obj).has_value())) {
        continue;
      }
      gt_objects_for_est_objs[est_obj] = gt_obj;
      assigned_gt_objs.insert(gt_obj);
    }
  } else {
    for (const auto &est_obj : estimated_objects) {
      Position3d<double> est_obj_pos =
          estimated_objects.at(est_obj.first).second.pose_.transl_;
      std::unordered_set<ObjectId> gt_objs_for_class =
          gt_obj_by_class.at(est_obj.second.first);
      for (const ObjectId &gt_obj : gt_objs_for_class) {
        Position3d<double> gt_pos = gt_objects.at(gt_obj).second.pose_.transl_;
        if ((gt_pos - est_obj_pos).norm() > max_assoc_dist) {
          continue;
        }
        bool closest_position = true;
        std::optional<ObjectId> curr_obj =
            gt_objects_for_est_objs.at(est_obj.first);
        if (curr_obj.has_value()) {
          Position3d<double> curr_closest_pos =
              gt_objects.at(curr_obj.value()).second.pose_.transl_;
          if ((est_obj_pos - curr_closest_pos).norm() <
              (est_obj_pos - gt_pos).norm()) {
            closest_position = false;
          }
        }
        if (closest_position) {
          gt_objects_for_est_objs[est_obj.first] = gt_obj;
        }
      }
    }
  }
}

void findTransformationForAssociatedObjects(
    const FullDOFEllipsoidResults &estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    Pose3D<double> &est_obj_transformation) {
  Position3d<double> mean_pos_est = Eigen::Vector3d::Zero();
  Position3d<double> mean_pos_gt = Eigen::Vector3d::Zero();
  Covariance<double, 3> object_position_cov = Eigen::Matrix3d::Zero();

  size_t total_poses = 0;
  for (const auto &gt_obj_for_est : gt_objects_for_est_objs) {
    if (!gt_obj_for_est.second.has_value()) {
      continue;
    }
    total_poses++;
    mean_pos_gt +=
        gt_objects.at(gt_obj_for_est.second.value()).second.pose_.transl_;
    mean_pos_est +=
        estimated_objects.at(gt_obj_for_est.first).second.pose_.transl_;
  }
  mean_pos_gt = mean_pos_gt / total_poses;
  mean_pos_est = mean_pos_est / total_poses;

  total_poses = 0;
  for (const auto &gt_obj_for_est : gt_objects_for_est_objs) {
    if (!gt_obj_for_est.second.has_value()) {
      continue;
    }

    total_poses++;
    Position3d<double> gt_deviation =
        gt_objects.at(gt_obj_for_est.second.value()).second.pose_.transl_ -
        mean_pos_gt;
    Position3d<double> est_deviation =
        estimated_objects.at(gt_obj_for_est.first).second.pose_.transl_ -
        mean_pos_est;

    object_position_cov += (gt_deviation * (est_deviation.transpose()));
  }
  object_position_cov = object_position_cov / total_poses;

  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd;
  svd.compute(object_position_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d uMat = svd.matrixU();
  Eigen::Matrix3d vMat = svd.matrixV();

  Eigen::Matrix3d wMat = Eigen::Matrix3d::Zero();
  wMat(0, 0) = 1;
  wMat(1, 1) = 1;
  if (uMat.determinant() * vMat.determinant() < 0) {
    wMat(2, 2) = -1;
  } else {
    wMat(2, 2) = 1;
  }

  Eigen::Matrix3d rotMat = uMat * wMat * vMat.transpose();

  Position3d<double> transl = mean_pos_gt - (rotMat * mean_pos_est);

  est_obj_transformation = Pose3D<double>(
      transl, Eigen::AngleAxis<double>(Eigen::Quaterniond(rotMat)));
}

void associateObjectsAndFindTransformation(
    const FullDOFEllipsoidResults &estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const bool &one_to_one,
    std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    Pose3D<double> &est_obj_transformation
    //    , std::shared_ptr<RosVisualization> &vis_manager
) {
  Pose3D<double> curr_transform;
  bool converged = false;
  int num_iter = 0;
  //  if (vis_manager != nullptr) {
  //    vis_manager->visualizeEllipsoids(gt_objects, GROUND_TRUTH, false);
  //    vis_manager->visualizeEllipsoids(estimated_objects, INITIAL, false);
  //    vis_manager->visualizeEllipsoids(estimated_objects, ESTIMATED, false);
  //    vis_manager->publishLines("obj_assoc_lines", ESTIMATED, {}, 1);
  //    vis_manager->publishLines("obj_assoc_lines", INITIAL, {}, 1);
  //    vis_manager->publishLines("obj_assoc_lines", GROUND_TRUTH, {}, 1);
  //  }
  //  getchar();
  //  sleep(2);
  while (!converged) {
    std::unordered_map<ObjectId, std::optional<ObjectId>>
        tmp_gt_objects_for_est_objs;
    FullDOFEllipsoidResults tmp_aligned_objects;
    adjustObjectFrame(estimated_objects, curr_transform, tmp_aligned_objects);

    //    if (vis_manager != nullptr) {
    //      std::vector<std::pair<Position3d<double>, Position3d<double>>>
    //          est_to_init;
    //      std::vector<std::pair<Position3d<double>, Position3d<double>>>
    //      est_to_gt;
    //
    //      vis_manager->visualizeEllipsoids(tmp_aligned_objects, ESTIMATED,
    //      false);
    //
    //      for (const auto &gt_obj_for_est : tmp_gt_objects_for_est_objs) {
    //        ObjectId est_obj_id = gt_obj_for_est.first;
    //        Position3d<double> orig_est_position =
    //            estimated_objects.at(est_obj_id).second.pose_.transl_;
    //        Position3d<double> adjusted_est_position =
    //            tmp_aligned_objects.at(est_obj_id).second.pose_.transl_;
    //
    //        est_to_init.emplace_back(
    //            std::make_pair(orig_est_position, adjusted_est_position));
    //
    //        if (!gt_obj_for_est.second.has_value()) {
    //          continue;
    //        }
    //        ObjectId gt_obj_id = gt_obj_for_est.second.value();
    //
    //        Position3d<double> gt_obj_position =
    //            gt_objects.at(gt_obj_id).second.pose_.transl_;
    //        est_to_gt.emplace_back(
    //            std::make_pair(adjusted_est_position, gt_obj_position));
    //      }
    //      vis_manager->publishLines("obj_assoc_lines", INITIAL, est_to_init,
    //      1); vis_manager->publishLines("obj_assoc_lines", ESTIMATED,
    //      est_to_gt, 1);
    //      //      sleep(2);
    //      //      getchar();
    //    }

    associateObjects(tmp_aligned_objects,
                     gt_objects,
                     true,
                     tmp_gt_objects_for_est_objs,
                     kMaxDistanceForInliers);

    //    if (vis_manager != nullptr) {
    //      std::vector<std::pair<Position3d<double>, Position3d<double>>>
    //      est_to_gt; std::vector<std::pair<Position3d<double>,
    //      Position3d<double>>> init_to_gt;
    //
    //      std::vector<double> association_distances;
    //
    //      for (const auto &gt_obj_for_est : tmp_gt_objects_for_est_objs) {
    //        if (!gt_obj_for_est.second.has_value()) {
    //          continue;
    //        }
    //        ObjectId est_obj_id = gt_obj_for_est.first;
    //        ObjectId gt_obj_id = gt_obj_for_est.second.value();
    //
    //        Position3d<double> gt_obj_position =
    //            gt_objects.at(gt_obj_id).second.pose_.transl_;
    //        Position3d<double> orig_est_position =
    //            estimated_objects.at(est_obj_id).second.pose_.transl_;
    //        Position3d<double> adjusted_est_position =
    //            tmp_aligned_objects.at(est_obj_id).second.pose_.transl_;
    //
    //        association_distances.emplace_back(
    //            (gt_obj_position - adjusted_est_position).norm());
    //        init_to_gt.emplace_back(
    //            std::make_pair(orig_est_position, gt_obj_position));
    //        est_to_gt.emplace_back(
    //            std::make_pair(adjusted_est_position, gt_obj_position));
    //      }
    ////      vis_manager->publishLines("obj_assoc_lines", GROUND_TRUTH,
    /// init_to_gt, 1); /      vis_manager->publishLines("obj_assoc_lines",
    /// ESTIMATED, est_to_gt, 1);
    //      //      sleep(2);
    //      //      getchar();
    //    }

    Pose3D<double> new_transform;
    findTransformationForAssociatedObjects(estimated_objects,
                                           gt_objects,
                                           tmp_gt_objects_for_est_objs,
                                           new_transform);

    num_iter++;
    if (num_iter >= kObjectAlignmentMaxIters) {
      converged = true;
    } else {
      Pose3D<double> pose_diff =
          getPose2RelativeToPose1(curr_transform, new_transform);
      if (((pose_diff.transl_.norm()) < kTranslationChangeForConvergence) &&
          (pose_diff.orientation_.angle() < kRotationChangeForConvergence)) {
        converged = true;
      }
    }

    curr_transform = new_transform;
  }

  est_obj_transformation = curr_transform;

  FullDOFEllipsoidResults tmp_aligned_objects;
  adjustObjectFrame(
      estimated_objects, est_obj_transformation, tmp_aligned_objects);

  associateObjects(
      tmp_aligned_objects, gt_objects, one_to_one, gt_objects_for_est_objs);

  //  if (vis_manager != nullptr) {
  //    LOG(INFO) << "Final assignments";
  //    std::vector<std::pair<Position3d<double>, Position3d<double>>>
  //    est_to_init; std::vector<std::pair<Position3d<double>,
  //    Position3d<double>>> est_to_gt;
  //    std::vector<std::pair<Position3d<double>, Position3d<double>>>
  //    init_to_gt;
  //
  //    vis_manager->visualizeEllipsoids(tmp_aligned_objects, ESTIMATED, false);
  //
  //    for (const auto &gt_obj_for_est : gt_objects_for_est_objs) {
  //      ObjectId est_obj_id = gt_obj_for_est.first;
  //      Position3d<double> orig_est_position =
  //          estimated_objects.at(est_obj_id).second.pose_.transl_;
  //      Position3d<double> adjusted_est_position =
  //          tmp_aligned_objects.at(est_obj_id).second.pose_.transl_;
  //
  //      est_to_init.emplace_back(
  //          std::make_pair(orig_est_position, adjusted_est_position));
  //
  //      if (!gt_obj_for_est.second.has_value()) {
  //        continue;
  //      }
  //      ObjectId gt_obj_id = gt_obj_for_est.second.value();
  //
  //      Position3d<double> gt_obj_position =
  //          gt_objects.at(gt_obj_id).second.pose_.transl_;
  //
  //      init_to_gt.emplace_back(
  //          std::make_pair(orig_est_position, gt_obj_position));
  //      est_to_gt.emplace_back(
  //          std::make_pair(adjusted_est_position, gt_obj_position));
  //    }
  //
  //    vis_manager->publishLines("obj_assoc_lines", GROUND_TRUTH, init_to_gt,
  //    1); vis_manager->publishLines("obj_assoc_lines", INITIAL, est_to_init,
  //    1); vis_manager->publishLines("obj_assoc_lines", ESTIMATED, est_to_gt,
  //    1);
  //    //    sleep(2);
  //    //    getchar();
  //  }
}

void adjustObjectFrame(const FullDOFEllipsoidResults &estimated_objects,
                       const Pose3D<double> &est_obj_transformation,
                       FullDOFEllipsoidResults &aligned_objects) {
  for (const auto &ellipsoid_entry : estimated_objects) {
    aligned_objects[ellipsoid_entry.first] = std::make_pair(
        ellipsoid_entry.second.first,
        FullDOFEllipsoidState(combinePoses(est_obj_transformation,
                                           ellipsoid_entry.second.second.pose_),
                              ellipsoid_entry.second.second.dimensions_));
  }
}

void getPositionDistancesList(
    const FullDOFEllipsoidResults &aligned_estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    std::unordered_map<ObjectId, std::optional<double>> &dist_from_gt_by_obj) {
  for (const auto &gt_obj_for_est_obj : gt_objects_for_est_objs) {
    ObjectId est_obj_id = gt_obj_for_est_obj.first;
    std::optional<double> dist_from_gt;
    if (gt_obj_for_est_obj.second.has_value()) {
      Pose3D<double> gt_obj_pose =
          gt_objects.at(gt_obj_for_est_obj.second.value()).second.pose_;
      Pose3D<double> aligned_est_obj_pose =
          aligned_estimated_objects.at(est_obj_id).second.pose_;
      dist_from_gt =
          (gt_obj_pose.transl_ - aligned_est_obj_pose.transl_).norm();
    }
    dist_from_gt_by_obj[est_obj_id] = dist_from_gt;
  }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
getAxisAlignedBoundingBoxForEllipsoid(
    const FullDOFEllipsoidState<double> &ellipsoid) {
  Eigen::Matrix4d dual_rep =
      createDualRepresentationForFullDOFEllipsoid(ellipsoid);

  // tangent planes to ellipsoid satisfy P^T Q P = 0 where P is the 4d vector
  // defining the plane and Q is the dual representation of the ellipsoid
  // The tangent planes along the min and max x dimensions are given by
  // [1 0 0 -x]^T. Similarly, the min/max y and z are given by planes satisfying
  // [0 1 0 -y] and [0 0 1 -z] respectively.
  // Using these planes and the dual equation, we can solve for the min max for
  // each of these variables, giving the following values

  double r_11 = dual_rep(0, 0);
  double r_14 = dual_rep(0, 3);
  double r_22 = dual_rep(1, 1);
  double r_24 = dual_rep(1, 3);
  double r_33 = dual_rep(2, 2);
  double r_34 = dual_rep(2, 3);
  double r_41 = dual_rep(3, 0);
  double r_42 = dual_rep(3, 1);
  double r_43 = dual_rep(3, 2);
  double r_44 = dual_rep(3, 3);

  double x_common = sqrt(pow(r_14 + r_41, 2) - (4 * r_44 * r_11));
  double x_1 = ((r_14 + r_41) + x_common) / (2 * r_44);
  double x_2 = ((r_14 + r_41) - x_common) / (2 * r_44);

  double y_common = sqrt(pow(r_24 + r_42, 2) - (4 * r_44 * r_22));
  double y_1 = ((r_24 + r_42) + y_common) / (2 * r_44);
  double y_2 = ((r_24 + r_42) - y_common) / (2 * r_44);

  double z_common = sqrt(pow(r_34 + r_43, 2) - (4 * r_44 * r_33));
  double z_1 = ((r_34 + r_43) + z_common) / (2 * r_44);
  double z_2 = ((r_34 + r_43) - z_common) / (2 * r_44);

  double x_min = std::min(x_1, x_2);
  double x_max = std::max(x_1, x_2);
  double y_min = std::min(y_1, y_2);
  double y_max = std::max(y_1, y_2);
  double z_min = std::min(z_1, z_2);
  double z_max = std::max(z_1, z_2);

  return std::make_pair(Eigen::Vector3d(x_min, y_min, z_min),
                        Eigen::Vector3d(x_max, y_max, z_max));
}

bool pointInEllipsoid(const FullDOFEllipsoidState<double> &ellipsoid,
                      const Position3d<double> &point) {
  Position3d<double> p_rel_el =
      getPositionRelativeToPose(ellipsoid.pose_, point);

  double dx_half = ellipsoid.dimensions_.x() / 2.0;
  double dy_half = ellipsoid.dimensions_.y() / 2.0;
  double dz_half = ellipsoid.dimensions_.z() / 2.0;

  return (pow(p_rel_el.x() / dx_half, 2) + pow(p_rel_el.y() / dy_half, 2) +
          pow(p_rel_el.z() / dz_half, 2)) <= 1;
}

bool boundingBoxesOverlap(
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> &bb_1,
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> &bb_2) {
  double e1_x_min = bb_1.first.x();
  double e1_x_max = bb_1.second.x();

  double e1_y_min = bb_1.first.y();
  double e1_y_max = bb_1.second.y();

  double e1_z_min = bb_1.first.z();
  double e1_z_max = bb_1.second.z();

  double e2_x_min = bb_2.first.x();
  double e2_x_max = bb_2.second.x();

  double e2_y_min = bb_2.first.y();
  double e2_y_max = bb_2.second.y();

  double e2_z_min = bb_2.first.z();
  double e2_z_max = bb_2.second.z();

  // Check if x intervals overlap ---- if no, then ellipsoids don't overlap,
  // if yes, then check y and z
  if (e1_x_min > e2_x_max) {  // e1 x range is fully greater than e2 x range
    // -- no overlap
    return false;
  }
  if (e2_x_min > e1_x_max) {  // e2 x range is fully greater than e1 x range
    // -- no overlap
    return false;
  }

  // Check if y intervals overlap ---- if no, then ellipsoids don't overlap,
  // if yes, then check z
  if (e1_y_min > e2_y_max) {  // e1 y range is fully greater than e2 y range
    // -- no overlap
    return false;
  }
  if (e2_y_min > e1_y_max) {  // e2 y range is fully greater than e1 y range
    // -- no overlap
    return false;
  }

  // Check if z intervals overlap ---- if no, then ellipsoids don't overlap,
  // if yes, ellipsoids may overlap
  if (e1_z_min > e2_z_max) {  // e1 z range is fully greater than e2 z range
    // -- no overlap
    return false;
  }
  if (e2_z_min > e1_z_max) {  // e2 z range is fully greater than e1 z range
    // -- no overlap
    return false;
  }
  return true;
}

double getIoUForObjectSet(
    const FullDOFEllipsoidState<double> &ellipsoid1,
    const std::vector<FullDOFEllipsoidState<double>> &covering_ellipsoids
    //    , const ObjectId &e1_id,
    //    const std::vector<ObjectId> &e2_ids,
    //    std::shared_ptr<RosVisualization> &vis_manager
) {
  //
  // Determine if ellipsoids overlap at all -- if not, return 0
  // Could attempt to do this more exactly, but for now, we're just checking if
  // their axis aligned bounding boxes overlap

  // Determine axis aligned bounding box for each ellipsoid
  std::pair<Eigen::Vector3d, Eigen::Vector3d> e1_bb =
      getAxisAlignedBoundingBoxForEllipsoid(ellipsoid1);

  double e1_x_min = e1_bb.first.x();
  double e1_x_max = e1_bb.second.x();

  double e1_y_min = e1_bb.first.y();
  double e1_y_max = e1_bb.second.y();

  double e1_z_min = e1_bb.first.z();
  double e1_z_max = e1_bb.second.z();

  //  vis_manager->publishBox(
  //      "bounding_box",
  //      GROUND_TRUTH,
  //      Position3d<double>((e1_x_max + e1_x_min) / 2,
  //                         (e1_y_max + e1_y_min) / 2,
  //                         (e1_z_max + e1_z_min) / 2),
  //      Eigen::Quaterniond(1, 0, 0, 0),
  //      Eigen::Vector3d(
  //          (e1_x_max - e1_x_min), (e1_y_max - e1_y_min), (e1_z_max -
  //          e1_z_min)),
  //      e1_id);

  double union_x_min = e1_x_min;
  double union_x_max = e1_x_max;
  double union_y_min = e1_y_min;
  double union_y_max = e1_y_max;
  double union_z_min = e1_z_min;
  double union_z_max = e1_z_max;

  //  std::vector<FullDOFEllipsoidState<double>>
  //  potentially_overlapping_ellipsoids;
  std::unordered_set<size_t> potentially_overlapping_ellipsoids;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> e2_bounding_boxes;
  //  for (const FullDOFEllipsoidState<double> &candidate_covering_ellipsoid :
  //       covering_ellipsoids) {
  for (size_t e2_idx = 0; e2_idx < covering_ellipsoids.size(); e2_idx++) {
    const FullDOFEllipsoidState<double> candidate_covering_ellipsoid =
        covering_ellipsoids.at(e2_idx);

    //  for (size_t covering_el_idx = 0; covering_el_idx <
    //  covering_ellipsoids.size();
    //       covering_el_idx++) {
    //    const FullDOFEllipsoidState<double> candidate_covering_ellipsoid =
    //        covering_ellipsoids.at(covering_el_idx);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> e2_bb =
        getAxisAlignedBoundingBoxForEllipsoid(candidate_covering_ellipsoid);
    e2_bounding_boxes.emplace_back(e2_bb);

    double e2_x_min = e2_bb.first.x();
    double e2_x_max = e2_bb.second.x();

    double e2_y_min = e2_bb.first.y();
    double e2_y_max = e2_bb.second.y();

    double e2_z_min = e2_bb.first.z();
    double e2_z_max = e2_bb.second.z();

    //    vis_manager->publishBox("bounding_box",
    //                            ESTIMATED,
    //                            Position3d<double>((e2_x_max + e2_x_min) / 2,
    //                                               (e2_y_max + e2_y_min) / 2,
    //                                               (e2_z_max + e2_z_min) / 2),
    //                            Eigen::Quaterniond(1, 0, 0, 0),
    //                            Eigen::Vector3d((e2_x_max - e2_x_min),
    //                                            (e2_y_max - e2_y_min),
    //                                            (e2_z_max - e2_z_min)),
    //                            e2_ids.at(covering_el_idx));
    if (!boundingBoxesOverlap(e1_bb, e2_bb)) {
      continue;
    }
    potentially_overlapping_ellipsoids.insert(e2_idx);
    //        candidate_covering_ellipsoid);

    // Take min/max for each coord to get union axis-aligned bounding box
    union_x_min = std::min(union_x_min, e2_x_min);
    union_x_max = std::max(union_x_max, e2_x_max);
    union_y_min = std::min(union_y_min, e2_y_min);
    union_y_max = std::max(union_y_max, e2_y_max);
    union_z_min = std::min(union_z_min, e2_z_min);
    union_z_max = std::max(union_z_max, e2_z_max);
  }

  if (potentially_overlapping_ellipsoids.empty()) {
    return 0;
  }

  // Want to generate grid of points evenly spaced over the union aligned
  // bounding box
  // We'll use these for approximating IoU with sampling
  double x_range = union_x_max - union_x_min;
  double y_range = union_y_max - union_y_min;
  double z_range = union_z_max - union_z_min;

  //  vis_manager->publishBox("bounding_box",
  //                          INITIAL,
  //                          Position3d<double>((union_x_min + union_x_max) /
  //                          2,
  //                                             (union_y_min + union_y_max) /
  //                                             2, (union_z_min + union_z_max)
  //                                             / 2),
  //                          Eigen::Quaterniond(1, 0, 0, 0),
  //                          Eigen::Vector3d(x_range, y_range, z_range),
  //                          e1_id);

  int num_x_points = std::min(kMaxPointsPerDirection,
                              (int)ceil(x_range * kIoUSamplingPointsPerMeter));
  int num_y_points = std::min(kMaxPointsPerDirection,
                              (int)ceil(y_range * kIoUSamplingPointsPerMeter));
  int num_z_points = std::min(kMaxPointsPerDirection,
                              (int)ceil(z_range * kIoUSamplingPointsPerMeter));
  LOG(INFO) << "Num x points " << num_x_points;
  LOG(INFO) << "Num y points " << num_y_points;
  LOG(INFO) << "Num z points " << num_z_points;
  double x_point_interval = x_range / (num_x_points - 1);
  double y_point_interval = y_range / (num_y_points - 1);
  double z_point_interval = z_range / (num_z_points - 1);

  util_random::Random rand_gen;

  double curr_x = rand_gen.UniformRandom(union_x_min, union_x_max);
  double curr_y = rand_gen.UniformRandom(union_y_min, union_y_max);
  double curr_z = rand_gen.UniformRandom(union_z_min, union_z_max);

  size_t points_in_either = 0;
  size_t points_in_both = 0;

  for (size_t x_idx = 0; x_idx <= num_x_points; x_idx++) {
    for (size_t y_idx = 0; y_idx <= num_y_points; y_idx++) {
      for (size_t z_idx = 0; z_idx <= num_z_points; z_idx++) {
        Eigen::Vector3d query_point(curr_x, curr_y, curr_z);

        // For each sample, check if point is in e1, covering set or both (add
        // to intersection and union counters as appropriate)
        bool in_e1 = pointInEllipsoid(ellipsoid1, query_point);

        bool in_covering_set = false;
        //        for (const FullDOFEllipsoidState<double>
        //        &candidate_covering_ellipsoid :
        //             potentially_overlapping_ellipsoids) {
        for (const size_t &candidate_covering_idx :
             potentially_overlapping_ellipsoids) {
          const FullDOFEllipsoidState<double> candidate_covering_ellipsoid =
              covering_ellipsoids.at(candidate_covering_idx);
          if (pointInEllipsoid(candidate_covering_ellipsoid, query_point)) {
            in_covering_set = true;
            break;
          }
        }

        if (in_e1 || in_covering_set) {
          points_in_either++;
        }
        if (in_e1 && in_covering_set) {
          points_in_both++;
        }
        curr_z += z_point_interval;
        if (curr_z > union_z_max) {
          curr_z -= z_range;
        }
      }
      curr_y += y_point_interval;
      if (curr_y > union_y_max) {
        curr_y -= y_range;
      }
    }
    curr_x += x_point_interval;
    if (curr_x > union_x_max) {
      curr_x -= x_range;
    }
  }

  std::unordered_set<size_t> considered_ellipsoids =
      potentially_overlapping_ellipsoids;

  double min_inc;
  double mid_inc;
  double max_inc;

  if (x_point_interval > y_point_interval) {
    if (y_point_interval > z_point_interval) {
      min_inc = z_point_interval;
      mid_inc = y_point_interval;
      max_inc = x_point_interval;
    } else {
      if (z_point_interval > x_point_interval) {
        max_inc = z_point_interval;
        min_inc = y_point_interval;
        mid_inc = x_point_interval;
      } else {
        // x is greater than z
        // z is greater than y
        max_inc = x_point_interval;
        mid_inc = z_point_interval;
        min_inc = y_point_interval;
      }
    }
  } else {
    // y is greater than x
    if (x_point_interval > z_point_interval) {
      min_inc = z_point_interval;
      max_inc = y_point_interval;
      mid_inc = x_point_interval;
    } else {
      if (z_point_interval > y_point_interval) {
        max_inc = z_point_interval;
        min_inc = x_point_interval;
        mid_inc = y_point_interval;
      } else {
        // y greater than x
        // x is less than z
        // z is less than y
        max_inc = y_point_interval;
        min_inc = x_point_interval;
        mid_inc = z_point_interval;
      }
    }
  }

  LOG(INFO) << "Number of other objects to consider: "
            << (covering_ellipsoids.size() -
                potentially_overlapping_ellipsoids.size());
  for (size_t ellipsoid_idx = 0; ellipsoid_idx < covering_ellipsoids.size();
       ellipsoid_idx++) {
    if (considered_ellipsoids.find(ellipsoid_idx) !=
        considered_ellipsoids.end()) {
      continue;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> bb_for_ellipsoid =
        e2_bounding_boxes.at(ellipsoid_idx);
    std::unordered_set<size_t> already_counted_overlapping;
    for (const size_t &candidate_already_counted : considered_ellipsoids) {
      std::pair<Eigen::Vector3d, Eigen::Vector3d> bb_for_ellipsoid_2 =
          e2_bounding_boxes.at(candidate_already_counted);
      if (boundingBoxesOverlap(bb_for_ellipsoid, bb_for_ellipsoid_2)) {
        already_counted_overlapping.insert(candidate_already_counted);
      }
    }

    double x_range_el =
        bb_for_ellipsoid.second.x() - bb_for_ellipsoid.first.x();
    double y_range_el =
        bb_for_ellipsoid.second.y() - bb_for_ellipsoid.first.y();
    double z_range_el =
        bb_for_ellipsoid.second.z() - bb_for_ellipsoid.first.z();

    double x_inc;
    double y_inc;
    double z_inc;

    if (x_range_el > y_range_el) {
      if (y_range_el > z_range_el) {
        z_inc = min_inc;
        y_inc = mid_inc;
        x_inc = max_inc;
      } else {
        if (z_range_el > x_range_el) {
          z_inc = max_inc;
          y_inc = min_inc;
          x_inc = mid_inc;
        } else {
          // x is greater than z
          // z is greater than y
          x_inc = max_inc;
          z_inc = mid_inc;
          y_inc = min_inc;
        }
      }
    } else {
      // y is greater than x
      if (x_range_el > z_range_el) {
        z_inc = min_inc;
        y_inc = max_inc;
        x_inc = mid_inc;
      } else {
        if (z_range_el > y_inc) {
          z_inc = max_inc;
          x_inc = min_inc;
          y_inc = mid_inc;
        } else {
          // y greater than x
          // x is less than z
          // z is less than y
          y_inc = max_inc;
          x_inc = min_inc;
          z_inc = mid_inc;
        }
      }
    }

    FullDOFEllipsoidState<double> el_to_check =
        covering_ellipsoids.at(ellipsoid_idx);

    size_t num_x_points_el = x_range / x_inc;
    size_t num_y_points_el = y_range / y_inc;
    size_t num_z_points_el = z_range / z_inc;

    LOG(INFO) << "Num points el x " << num_x_points_el;
    LOG(INFO) << "Num points el y " << num_y_points_el;
    LOG(INFO) << "Num points el z " << num_z_points_el;

    size_t inc_for_bb = 1;
    while (num_x_points_el > kMaxPointsPerDirection) {
      inc_for_bb *= 2;
      num_x_points_el /= 2;
      x_inc *= 2;
    }
    while (num_y_points_el > kMaxPointsPerDirection) {
      inc_for_bb *= 2;
      num_y_points_el /= 2;
      y_inc *= 2;
    }
    while (num_z_points_el > kMaxPointsPerDirection) {
      inc_for_bb *= 2;
      num_z_points_el /= 2;
      z_inc *= 2;
    }

    LOG(INFO) << "Num points el x after adjusting " << num_x_points_el;
    LOG(INFO) << "Num points el y after adjusting " << num_y_points_el;
    LOG(INFO) << "Num points el z after adjusting " << num_z_points_el;


//    for (double curr_x_el = bb_for_ellipsoid.first.x();
//         curr_x_el < bb_for_ellipsoid.second.x();
//         curr_x_el += x_inc) {
    for (size_t x_idx = 0; x_idx <= num_x_points_el; x_idx++) {
      double curr_x_el = bb_for_ellipsoid.first.x() + (x_inc * x_idx);
      LOG_IF(INFO, (x_idx % 100) == 0) << "x idx: " << x_idx;
      for (size_t y_idx = 0; y_idx <= num_y_points_el; y_idx++) {
//      for (double curr_y_el = bb_for_ellipsoid.first.y();
//           curr_y_el < bb_for_ellipsoid.second.y();
//           curr_y_el += y_inc) {
        double curr_y_el = bb_for_ellipsoid.first.y() + (y_inc * y_idx);
        LOG_IF(INFO, (((x_idx % 100) == 0) && ((y_idx % 100) == 0))) << "y idx: " << y_idx;
//        for (double curr_z_el = bb_for_ellipsoid.first.z();
//             curr_z_el < bb_for_ellipsoid.second.z();
//             curr_z_el += z_inc) {
        for (size_t z_idx = 0; z_idx <= num_z_points_el; z_idx++) {
//          LOG_IF(INFO, (z_idx % 100) == 0) << "z idx: " << z_idx;
          double curr_z_el = bb_for_ellipsoid.first.z() + (z_inc * z_idx);
          Position3d<double> point(curr_x_el, curr_y_el, curr_z_el);
          if (pointInEllipsoid(el_to_check, point)) {
            bool already_counted = false;
            for (const size_t &overlapping_and_counted_el :
                 already_counted_overlapping) {
              if (pointInEllipsoid(
                      covering_ellipsoids.at(overlapping_and_counted_el),
                      point)) {
                already_counted = true;
                break;
              }
            }
            if (!already_counted) {
              points_in_either += inc_for_bb;
            }
          }
        }
      }
    }

    considered_ellipsoids.insert(ellipsoid_idx);
  }

  double iou = ((double)points_in_both) / (points_in_either);
  return iou;
}

void getIoUsForObjects(
    const FullDOFEllipsoidResults &aligned_estimated_objects,
    const FullDOFEllipsoidResults &gt_objects,
    const std::unordered_map<ObjectId, std::optional<ObjectId>>
        &gt_objects_for_est_objs,
    std::unordered_map<ObjectId, double> &iou_per_gt_obj
    //    , std::shared_ptr<RosVisualization> &vis_manager
) {
  std::unordered_map<ObjectId, std::unordered_set<ObjectId>>
      est_objs_assoc_with_gt;
  for (const auto &pairing_by_est : gt_objects_for_est_objs) {
    if (pairing_by_est.second.has_value()) {
      ObjectId gt_obj_id = pairing_by_est.second.value();
      if (est_objs_assoc_with_gt.find(gt_obj_id) ==
          est_objs_assoc_with_gt.end()) {
        est_objs_assoc_with_gt[gt_obj_id] = {};
      }
      est_objs_assoc_with_gt[gt_obj_id].insert(pairing_by_est.first);
    }
  }
  LOG(INFO) << "Num objects " << gt_objects.size();
  for (const auto &gt_obj : gt_objects) {
    ObjectId gt_obj_id = gt_obj.first;
    LOG(INFO) << "Gt obj " << gt_obj_id;
    FullDOFEllipsoidState<double> gt_obj_geometry = gt_obj.second.second;
    double iou = 0;
    if (est_objs_assoc_with_gt.find(gt_obj_id) !=
        est_objs_assoc_with_gt.end()) {
      std::unordered_set<ObjectId> associated_objects =
          est_objs_assoc_with_gt.at(gt_obj_id);
      std::vector<FullDOFEllipsoidState<double>> assoc_obj_geometries;
      std::vector<ObjectId> associated_object_ids;
      for (const ObjectId &obj_id : associated_objects) {
        assoc_obj_geometries.emplace_back(
            aligned_estimated_objects.at(obj_id).second);
        associated_object_ids.emplace_back(obj_id);
      }
      iou = getIoUForObjectSet(
          gt_obj_geometry, assoc_obj_geometries
          //                               , gt_obj_id,
          //                               associated_object_ids,
          //                               vis_manager
      );
    }
    iou_per_gt_obj[gt_obj_id] = iou;
  }
}
}  // namespace vslam_types_refactor