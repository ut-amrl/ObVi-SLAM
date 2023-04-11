// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <file_io/cv_file_storage/object_and_reprojection_feature_pose_graph_file_storage_io.h>
#include <gtest/gtest.h>

#include <filesystem>

using namespace vslam_types_refactor;
namespace fs = std::filesystem;

TEST(ObjectAndReprojectionFeaturePoseGraphState,
     ReadWriteObjectAndReprojectionFeaturePoseGraphState) {
  ObjOnlyPoseGraphState obj_only_pose_graph_state;  // TODO

  Covariance<double, 3> chair_cov;
  chair_cov << 1.0, 2.1, 3.2, 4.3, 5.4, 6.5, 7.6, 8.7, 9.8;
  Covariance<double, 3> trashcan_cov;
  trashcan_cov << 1.9, 2.0, 3.1, 4.2, 5.3, 6.4, 7.5, 8.6, 9.7;
  obj_only_pose_graph_state.mean_and_cov_by_semantic_class_ = {
      {"chair", std::make_pair(ObjectDim<double>(1.2, 94.3, 92.3), chair_cov)},
      {"trashcan",
       std::make_pair(ObjectDim<double>(3.2, -03.2, 18.3), trashcan_cov)}};

  obj_only_pose_graph_state.min_object_id_ = 93;
  obj_only_pose_graph_state.max_object_id_ = 19038;

  RawEllipsoid<double> ellipsoid_1_state;
  ellipsoid_1_state << 84.3, 913.3, 8.4, 19.3, 9.4, 58.2, 3.1;
  RawEllipsoid<double> ellipsoid_2_state;
  ellipsoid_2_state << 9.4, -184.4, 4.2, 18.3, -10.3, 4.2, 0.3;
  obj_only_pose_graph_state.ellipsoid_estimates_ = {{14, ellipsoid_1_state},
                                                    {94, ellipsoid_2_state}};
  obj_only_pose_graph_state.semantic_class_for_object_ = {{324, "abc"},
                                                          {183, "def"}};
  obj_only_pose_graph_state.last_observed_frame_by_object_ = {{493, 139},
                                                              {129, 492}};
  obj_only_pose_graph_state.first_observed_frame_by_object_ = {{1848, 10},
                                                               {19348, 193}};
  obj_only_pose_graph_state.min_object_observation_factor_ = 13;
  obj_only_pose_graph_state.max_object_observation_factor_ = 93;
  obj_only_pose_graph_state.min_obj_specific_factor_ = 31;
  obj_only_pose_graph_state.max_obj_specific_factor_ = 193;

  obj_only_pose_graph_state.long_term_map_object_ids_.insert(13);
  obj_only_pose_graph_state.long_term_map_object_ids_.insert(493);
  obj_only_pose_graph_state.long_term_map_object_ids_.insert(472);
  obj_only_pose_graph_state.long_term_map_object_ids_.insert(846);

  Covariance<double, 4> bb_cov_1;
  bb_cov_1 << 1, 2, 3, 4, 11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34;
  Covariance<double, 4> bb_cov_2;
  bb_cov_2 << 0.1, 0.2, 0.3, 0.4, 1.1, 1.2, 1.3, 1.4, 2.1, 2.2, 2.3, 2.4, 3.1,
      3.2, 3.3, 3.4;
  obj_only_pose_graph_state.object_observation_factors_ = {
      {32,
       ObjectObservationFactor(
           94, 23, 43, BbCorners<double>(1.2, 2.3, 3.4, 1.4), bb_cov_1, 13.4)},
      {94,
       ObjectObservationFactor(92,
                               91,
                               42,
                               BbCorners<double>(94.2, 42.4, 0.1, 92.1),
                               bb_cov_2,
                               94.1)}};

  Covariance<double, 3> shape_cov_1;
  shape_cov_1 << 3.2, 45.2, 0.1, 34.1, 3.1, 0.4, 9.3, 2.5, 13.4;
  Covariance<double, 3> shape_cov_2;
  shape_cov_2 << 0.32, 4.52, 0.01, 3.41, 0.31, 0.04, 0.93, 0.25, 1.34;
  obj_only_pose_graph_state.shape_dim_prior_factors_ = {
      {90,
       ShapeDimPriorFactor(42, ObjectDim<double>(4.2, 0.3, 13.3), shape_cov_1)},
      {13,
       ShapeDimPriorFactor(
           135, ObjectDim<double>(9.4, 13.4, 9.3), shape_cov_2)}};

  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obs_factor_frame1;
  obs_factor_frame1.insert(std::make_pair(kReprojectionErrorFactorTypeId, 23));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obs_factor_frame2;
  obs_factor_frame2.insert(std::make_pair(kShapeDimPriorFactorTypeId, 40));
  obs_factor_frame2.insert(std::make_pair(kPairwiseErrorFactorTypeId, 13));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obs_factor_frame3;
  obs_factor_frame3.insert(std::make_pair(kLongTermMapFactorTypeId, 99));
  obs_factor_frame3.insert(std::make_pair(kObjectObservationFactorTypeId, 138));
  obs_factor_frame3.insert(std::make_pair(kPairwiseRobotPoseFactorTypeId, 924));
  obj_only_pose_graph_state.observation_factors_by_frame_ = {
      {42, obs_factor_frame1}, {91, obs_factor_frame2}, {194, obs_factor_frame3}
  };

  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obs_factor_obj1;
  obs_factor_obj1.insert(std::make_pair(kReprojectionErrorFactorTypeId, 3));
  obs_factor_obj1.insert(std::make_pair(kShapeDimPriorFactorTypeId, 45));
  obs_factor_obj1.insert(std::make_pair(kObjectObservationFactorTypeId, 914));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obs_factor_obj2;
  obs_factor_obj2.insert(std::make_pair(kLongTermMapFactorTypeId, 342));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obs_factor_obj3;
  obs_factor_obj3.insert(std::make_pair(kPairwiseErrorFactorTypeId, 94842));
  obs_factor_obj3.insert(std::make_pair(kPairwiseRobotPoseFactorTypeId, 1345));
  obj_only_pose_graph_state.observation_factors_by_object_ = {
      {84, obs_factor_obj1}, {76, obs_factor_obj2}, {95, obs_factor_obj3}
  };

  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obj_only_factors_by_obj1;
  obj_only_factors_by_obj1.insert(std::make_pair(kPairwiseRobotPoseFactorTypeId, 84));
  obj_only_factors_by_obj1.insert(std::make_pair(kShapeDimPriorFactorTypeId, 4567));
  obj_only_factors_by_obj1.insert(std::make_pair(kReprojectionErrorFactorTypeId, 678));
  obj_only_factors_by_obj1.insert(std::make_pair(kLongTermMapFactorTypeId, 34));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> obj_only_factors_by_obj2;
  obj_only_factors_by_obj2.insert(std::make_pair(kPairwiseRobotPoseFactorTypeId, 7892));
  obj_only_pose_graph_state.object_only_factors_by_object_ = {
      {24, obj_only_factors_by_obj1},
      {62, obj_only_factors_by_obj2}
  };


  LowLevelFeaturePoseGraphState<ReprojectionErrorFactor> low_level_pg_state;
  low_level_pg_state.camera_extrinsics_by_camera_ = {
      {1,
       CameraExtrinsics<double>(
           Position3d<double>(-0.3, 4.2, 2.3),
           Orientation3D<double>(4.3, Eigen::Vector3d(-.3, 12.3, -9)))},
      {2,
       CameraExtrinsics<double>(
           Position3d<double>(-1.3, 7.2, -2.3),
           Orientation3D<double>(413, Eigen::Vector3d(-.13, 142.3, -9.1)))}};

  CameraIntrinsicsMat<double> intrinsics1;
  intrinsics1 << 3.2, 89.3, 0.2, 1.4, 3.4, 9.3, 0.5, 0.2, 1.3;

  CameraIntrinsicsMat<double> intrinsics2;
  intrinsics2 << 13.2, 19.3, 1.2, 2.4, 6.4, 8.3, 1.5, 9.2, 1.5;
  low_level_pg_state.camera_intrinsics_by_camera_ = {{1, intrinsics1},
                                                     {2, intrinsics2}};
  low_level_pg_state.visual_factor_type_ = kReprojectionErrorFactorTypeId;
  low_level_pg_state.min_frame_id_ = 0;
  low_level_pg_state.max_frame_id_ = 500;
  low_level_pg_state.max_feature_factor_id_ = 9825256;
  low_level_pg_state.max_pose_factor_id_ = 135;
  RawPose3d<double> pose1;
  pose1 << 1.2, 2.3, 3.4, 4.5, 5.6, 6.7;
  RawPose3d<double> pose2;
  pose2 << 1.3, 2.4, 3.5, 4.6, 5.7, 6.8;
  RawPose3d<double> pose3;
  pose3 << 1.4, 2.5, 3.6, 4.7, 5.8, 6.9;
  low_level_pg_state.robot_poses_ = {{1, pose1}, {2, pose2}, {5, pose3}};

  std::unordered_map<FrameId,
                     util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>>
      pose_factors_by_frame;
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
      frame1_pose_factors;
  frame1_pose_factors.insert(
      std::make_pair(kReprojectionErrorFactorTypeId, 12));
  frame1_pose_factors.insert(std::make_pair(kPairwiseErrorFactorTypeId, 72));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
      frame2_pose_factors;
  frame2_pose_factors.insert(
      std::make_pair(kPairwiseRobotPoseFactorTypeId, 973));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
      frame3_pose_factors;
  frame3_pose_factors.insert(
      std::make_pair(kReprojectionErrorFactorTypeId, 10384));
  frame3_pose_factors.insert(std::make_pair(kPairwiseErrorFactorTypeId, 384));
  frame3_pose_factors.insert(
      std::make_pair(kObjectObservationFactorTypeId, 104));
  pose_factors_by_frame[10] = frame1_pose_factors;
  pose_factors_by_frame[510] = frame2_pose_factors;
  pose_factors_by_frame[190] = frame3_pose_factors;
  low_level_pg_state.pose_factors_by_frame_ = pose_factors_by_frame;

  low_level_pg_state.visual_feature_factors_by_frame_ = {
      {284,
       {std::make_pair(kReprojectionErrorFactorTypeId, 13),
        std::make_pair(kObjectObservationFactorTypeId, 420)}},
      {953, {std::make_pair(kLongTermMapFactorTypeId, 134)}},
      {344,
       {std::make_pair(kLongTermMapFactorTypeId, 42),
        std::make_pair(kObjectObservationFactorTypeId, 3),
        std::make_pair(kPairwiseRobotPoseFactorTypeId, 948)}}};

  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> feat1_factors;
  feat1_factors.insert(std::make_pair(kPairwiseRobotPoseFactorTypeId, 21));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> feat2_factors;
  feat2_factors.insert(std::make_pair(kObjectObservationFactorTypeId, 124));
  feat2_factors.insert(std::make_pair(kLongTermMapFactorTypeId, 13));
  util::BoostHashSet<std::pair<FactorType, FeatureFactorId>> feat3_factors;
  feat3_factors.insert(std::make_pair(kPairwiseErrorFactorTypeId, 139));
  feat3_factors.insert(std::make_pair(kShapeDimPriorFactorTypeId, 938));
  feat3_factors.insert(std::make_pair(kReprojectionErrorFactorTypeId, 492));
  low_level_pg_state.visual_factors_by_feature_ = {
      {24, feat1_factors}, {94, feat2_factors}, {301, feat3_factors}};

  Covariance<double, 6> pose_factor1_cov;
  pose_factor1_cov << 1.2, 4, 3.5, 10.4, -0.3, -20.3, 1.25, 4.5, 3.0, 11.4,
      -0.8, -21.3, 1.24, 4.4, 3.4, 12.4, -0.7, -22.3, 1.23, 4.3, 3.3, 13.4,
      -0.6, -23.3, 1.22, 4.2, 3.2, 14.4, -0.5, -24.3, 1.21, 4.1, 3.1, 15.4,
      -0.4, -25.3;
  Covariance<double, 6> pose_factor2_cov;
  pose_factor2_cov << 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 11.2, 12.3, 13.4, 14.5,
      15.6, 16.7, 1.21, 2.31, 3.41, 4.51, 5.61, 6.71, 21.2, 22.3, 23.4, 24.5,
      25.6, 26.7, 1.22, 2.32, 3.42, 4.52, 5.62, 6.72, 31.2, 32.3, 33.4, 34.5,
      35.6, 36.7;
  low_level_pg_state.pose_factors_ = {
      {123,
       RelPoseFactor(
           1,
           2,
           Pose3D<double>(Position3d<double>(4.2, 0.4, -0.3),
                          Orientation3D<double>(
                              -1 * M_PI, Eigen::Vector3d(0.4, -19.3, 48.2))),
           pose_factor1_cov)},
      {94,
       RelPoseFactor(
           3,
           4,
           Pose3D<double>(Position3d<double>(4.6, 0.2, -9.4),
                          Orientation3D<double>(
                              -M_PI / 3, Eigen::Vector3d(-9.3, 34.2, -0.2))),
           pose_factor2_cov)}};
  low_level_pg_state.factors_ = {
      {32, ReprojectionErrorFactor(1, 2, 3, PixelCoord<double>(1.2, 3.4), 4.2)},
      {832,
       ReprojectionErrorFactor(
           4, 3, 49, PixelCoord<double>(-38.4, 39.4), 1.3)}};

  low_level_pg_state.last_observed_frame_by_feature_ = {
      {4, 1}, {38, 183}, {188, 973}};
  low_level_pg_state.first_observed_frame_by_feature_ = {
      {5, 2}, {39, 184}, {189, 974}};

  ReprojectionLowLevelFeaturePoseGraphState reproj_pg_state;
  reproj_pg_state.min_feature_id_ = 10;
  reproj_pg_state.max_feature_id_ = 50;
  reproj_pg_state.feature_positions_ = {
      {5, Position3d<double>(1.2, 3.4, 5.6)},
      {6, Position3d<double>{2.3, 4.5, 6.7}},
      {7, Position3d<double>{-0.35, -483.3, 9.2}}};
  reproj_pg_state.low_level_pg_state_ = low_level_pg_state;

  ObjectAndReprojectionFeaturePoseGraphState pose_graph_state;
  pose_graph_state.reprojection_low_level_feature_pose_graph_state_ =
      reproj_pg_state;
  pose_graph_state.obj_only_pose_graph_state_ = obj_only_pose_graph_state;

  std::FILE *tmp_file = std::tmpfile();
  std::string tmp_file_name = fs::read_symlink(
      fs::path("/proc/self/fd") / std::to_string(fileno(tmp_file)));
  outputPoseGraphStateToFile(pose_graph_state, tmp_file_name);
  ObjectAndReprojectionFeaturePoseGraphState read_state;
  readPoseGraphStateFromFile(tmp_file_name, read_state);
  ASSERT_EQ(pose_graph_state, read_state);
}
