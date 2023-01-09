//
// Created by amanda on 1/7/23.
//

#ifndef UT_VSLAM_BOUNDING_BOX_FRONT_END_CREATION_UTILS_H
#define UT_VSLAM_BOUNDING_BOX_FRONT_END_CREATION_UTILS_H

#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/bounding_box_frontend/roshan_bounding_box_front_end.h>
#include <refactoring/optimization/object_pose_graph.h>

namespace vslam_types_refactor {

struct BoundingBoxCovGenParams {
  Covariance<double, 4> bounding_box_cov_ =
      createDiagCovFromStdDevs(Eigen::Vector4d(30, 30, 30, 30));
  double near_edge_threshold_ = 2500; // TODO tune
  double image_boundary_variance_ = pow(200.0, 2.0);  // TODO tune
};

struct GeometricSimilarityScorerParams {
  double max_merge_distance_ = 1.5;
};

template <typename ImageContext>
std::function<Covariance<double, 4>(const RawBoundingBox &,
                                    const FrameId &,
                                    const CameraId &,
                                    const ImageContext &)>
getBoundingBoxCovarianceGenerator(
    const std::unordered_map<CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const BoundingBoxCovGenParams &cov_gen_params) {
  std::function<Covariance<double, 4>(const RawBoundingBox &,
                                      const FrameId &,
                                      const CameraId &,
                                      const ImageContext &)>
      covariance_generator = [&](const RawBoundingBox &bb,
                                 const FrameId &,
                                 const CameraId &camera_id,
                                 const ImageContext &) {
        Covariance<double, 4> initial_covariance =
            cov_gen_params.bounding_box_cov_;
        // TODO make sure getting covariance order right
        if (bb.pixel_corner_locations_.first.x() <
            cov_gen_params.near_edge_threshold_) {
          initial_covariance(0, 0) = cov_gen_params.image_boundary_variance_;
        }
        if (bb.pixel_corner_locations_.first.y() <
            cov_gen_params.near_edge_threshold_) {
          initial_covariance(2, 2) = cov_gen_params.image_boundary_variance_;
        }
        if (img_heights_and_widths.find(camera_id) !=
            img_heights_and_widths.end()) {
          std::pair<double, double> img_height_and_width =
              img_heights_and_widths.at(camera_id);
          if (bb.pixel_corner_locations_.second.x() >
              (img_height_and_width.second -
               cov_gen_params.near_edge_threshold_)) {
            initial_covariance(1, 1) = cov_gen_params.image_boundary_variance_;
          }
          if (bb.pixel_corner_locations_.second.y() >
              (img_height_and_width.first -
               cov_gen_params.near_edge_threshold_)) {
            initial_covariance(3, 3) = cov_gen_params.image_boundary_variance_;
          }
        }

        return initial_covariance;
      };
  return covariance_generator;
}

RoshanBbFrontEndCreator<ReprojectionErrorFactor> generateRoshanBbCreator(
    const std::shared_ptr<std::unordered_map<
        FrameId,
        std::unordered_map<
            CameraId,
            std::unordered_map<
                ObjectId,
                std::pair<BbCornerPair<double>, std::optional<double>>>>>>
        &associated_observed_corner_locations,
    const std::shared_ptr<std::unordered_map<
        FrameId,
        std::unordered_map<CameraId,
                           std::vector<std::pair<BbCornerPair<double>,
                                                 std::optional<double>>>>>>
        &all_observed_corner_locations_with_uncertainty,
    const std::unordered_map<CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const BoundingBoxCovGenParams &bb_cov_gen_params,
    const std::unordered_map<ObjectId, RoshanAggregateBbInfo>
        &long_term_map_front_end_data) {
  RoshanBbAssociationParams roshan_associator_params;  // TODO tune these
  roshan_associator_params.saturation_histogram_bins_ = 50;
  roshan_associator_params.hue_histogram_bins_ = 60;
  //  roshan_associator_params.max_distance_for_associated_ellipsoids_ = 2.0; //
  //  inside
  roshan_associator_params.max_distance_for_associated_ellipsoids_ = 3.5;
  //  roshan_associator_params.min_observations_ = 40;
  //  roshan_associator_params.min_observations_ = 10;
  roshan_associator_params.min_observations_ = 40;
  roshan_associator_params.discard_candidate_after_num_frames_ = 40;
  roshan_associator_params.min_bb_confidence_ = 0.3;
  roshan_associator_params.required_min_conf_for_initialization = 0.5;

  return RoshanBbFrontEndCreator<ReprojectionErrorFactor>(
      roshan_associator_params,
      associated_observed_corner_locations,
      all_observed_corner_locations_with_uncertainty,
      getBoundingBoxCovarianceGenerator<RoshanImageSummaryInfo>(
          img_heights_and_widths, bb_cov_gen_params),
      long_term_map_front_end_data);
}

FeatureBasedBoundingBoxFrontEndCreator<ReprojectionErrorFactor>
generateFeatureBasedBbCreator(
    const std::shared_ptr<std::unordered_map<
        FrameId,
        std::unordered_map<
            CameraId,
            std::unordered_map<
                ObjectId,
                std::pair<BbCornerPair<double>, std::optional<double>>>>>>
        &associated_observed_corner_locations,
    const std::shared_ptr<std::unordered_map<
        FrameId,
        std::unordered_map<CameraId,
                           std::vector<std::pair<BbCornerPair<double>,
                                                 std::optional<double>>>>>>
        &all_observed_corner_locations_with_uncertainty,
    const std::unordered_map<CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const BoundingBoxCovGenParams &bb_cov_gen_params,
    const GeometricSimilarityScorerParams &similarity_scorer_params) {
  // Configurable params ------------------------------------------------------
  FeatureBasedBbAssociationParams association_params;  // TODO populate
  association_params.discard_candidate_after_num_frames_ = 40;
  association_params.feature_validity_window_ = 20;

  std::function<bool(
      const EllipsoidState<double> &, const EllipsoidState<double> &, double &)>
      geometric_similarity_scorer =
          [&](const EllipsoidState<double> &ellipsoid_1,
              const EllipsoidState<double> &ellipsoid_2,
              double &similarity_score) {
            // TODO should dist use full distance or distance projected onto x-y
            // plane?
            //            double dist = (ellipsoid_1.pose_.transl_ -
            //            ellipsoid_2.pose_.transl_).norm();
            Eigen::Vector2d projected_center_1 =
                ellipsoid_1.pose_.transl_.topRows(2);
            Eigen::Vector2d projected_center_2 =
                ellipsoid_2.pose_.transl_.topRows(2);
            double dist = (projected_center_1 - projected_center_2).norm();
            if (dist > similarity_scorer_params.max_merge_distance_) {
              return false;
            }
            similarity_score = -1 * dist;
            return true;
          };
  return FeatureBasedBoundingBoxFrontEndCreator<ReprojectionErrorFactor>(
      association_params,
      getBoundingBoxCovarianceGenerator<FeatureBasedContextInfo>(
          img_heights_and_widths, bb_cov_gen_params),
      geometric_similarity_scorer,
      all_observed_corner_locations_with_uncertainty,
      associated_observed_corner_locations);
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_BOUNDING_BOX_FRONT_END_CREATION_UTILS_H
