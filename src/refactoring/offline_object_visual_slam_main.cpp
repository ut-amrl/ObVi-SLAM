#include <base_lib/basic_utils.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/optimization/residual_creator.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/pose_graph_frame_data_adder.h>
#include <refactoring/roshan_bounding_box_front_end.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

std::vector<std::shared_ptr<ceres::IterationCallback>>
dummyCeresCallbackCreator(
    const vtr::UnassociatedBoundingBoxOfflineProblemData<
        vtr::StructuredVisionFeatureTrack,
        sensor_msgs::Image::ConstPtr> &input_problem_data,
    const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const vtr::FrameId &min_frame_optimized,
    const vtr::FrameId &max_frame_optimized) {
  // TODO replace with actual code later
  return {};
}

bool checkFactorRefresh(
    const std::pair<vtr::FactorType, vtr::FeatureFactorId> &factor,
    const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
    const util::EmptyStruct &) {
  return false;
}

void createPoseGraph(
    const vtr::UnassociatedBoundingBoxOfflineProblemData<
        vtr::StructuredVisionFeatureTrack,
        sensor_msgs::Image::ConstPtr> &input_problem_data,
    std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &pose_graph) {
  pose_graph = std::make_shared<vtr::ObjectAndReprojectionFeaturePoseGraph>(
      input_problem_data.getObjDimMeanAndCovByClass(),
      input_problem_data.getCameraExtrinsicsByCamera(),
      input_problem_data.getCameraIntrinsicsByCamera());
}

void visualizationStub(const vtr::UnassociatedBoundingBoxOfflineProblemData<
                           vtr::StructuredVisionFeatureTrack,
                           sensor_msgs::Image::ConstPtr> &input_problem_data,
                       std::shared_ptr<vtr::ObjAndLowLevelFeaturePoseGraph<
                           vtr::ReprojectionErrorFactor>> &pose_graph,
                       const vtr::FrameId &min_frame_optimized,
                       const vtr::FrameId &max_frame_optimized,
                       const vtr::VisualizationTypeEnum &visualization_stage) {
  // TODO fill in with actual visualization
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // Read in data
  // TODO read in initial trajectory estimates
  // TODO read in bounding boxes (either with identifiers or not)
  // Optional: if bbs have identifiers, also take in initial ellipsoid estimates
  // TODO read in images
  // TODO read in feature tracks (with identifier)
  // TODO read in intrinsics
  // TODO read in extrinsics

  // TODO optionally read in ground truth for object poses/dimensions
  // TODO optionally read in ground truth for data associations
  // TODO optionally read in ground truth for robot poses
  // TODO optionally read in ground truth for feature positions

  // TODO encapsulate data in offline problem data structure -- should we do the
  // data association as a preprocessing step or as it is running -- probably as
  // it is running?
  // TODO create pose graph from offline data

  std::function<bool()> continue_opt_checker = []() { return ros::ok(); };

  std::function<vtr::FrameId(const vtr::FrameId &)> window_provider_func =
      [](const vtr::FrameId &) {
        // For now, we'll just optimize the whole trajectory (so return 0 so we
        // start the optimization with node 0
        return 0;
      };

  std::function<bool(
      const std::pair<vtr::FactorType, vtr::FeatureFactorId> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const util::EmptyStruct &)>
      refresh_residual_checker = checkFactorRefresh;

  std::function<bool(
      const std::pair<vtr::FactorType, vtr::FeatureFactorId> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      util::EmptyStruct &)>
      cached_info_creator =
          [](const std::pair<vtr::FactorType, vtr::FeatureFactorId>
                 &factor_info,
             const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             util::EmptyStruct &cached_info) {
            return true;  // TODO maybe fill in with real info some day
          };
  std::function<bool(
      const std::pair<vtr::FactorType, vtr::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator =
          [&](const std::pair<vtr::FactorType, vtr::FeatureFactorId> &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                  &pose_graph,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return vtr::createResidual(factor_id,
                                       pose_graph,
                                       solver_residual_params,
                                       cached_info_creator,
                                       problem,
                                       residual_id,
                                       cached_info);
          };
  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &)>
      pose_graph_creator = createPoseGraph;
  std::function<double(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &,
      const vtr::FeatureId &,
      const vtr::CameraId &)>
      reprojection_error_provider;  // TODO
  std::function<sensor_msgs::Image::ConstPtr(
      const vtr::FrameId &,
      const vtr::CameraId &,
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &)>
      bb_context_retriever =
          [](const vtr::FrameId &frame_id,
             const vtr::CameraId &camera_id,
             const vtr::UnassociatedBoundingBoxOfflineProblemData<
                 vtr::StructuredVisionFeatureTrack,
                 sensor_msgs::Image::ConstPtr> &problem_data) {
            return problem_data.getImageForFrameAndCamera(frame_id, camera_id);
          };
  vtr::RoshanBbAssociationParams roshan_associator_params;  // TODO

  std::function<vtr::Covariance<double, 4>(const vtr::RawBoundingBox &,
                                           const vtr::FrameId &,
                                           const vtr::CameraId &,
                                           const vtr::RoshanImageSummaryInfo &)>
      covariance_generator;  // TODO
  vtr::RoshanBbFrontEndCreator<vtr::ReprojectionErrorFactor>
      roshan_associator_creator(roshan_associator_params, covariance_generator);
  std::function<std::shared_ptr<
      vtr::AbstractBoundingBoxFrontEnd<vtr::ReprojectionErrorFactor,
                                       vtr::RoshanAggregateBbInfo,
                                       sensor_msgs::Image::ConstPtr,
                                       vtr::RoshanImageSummaryInfo,
                                       vtr::RoshanBbInfo>>(
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &)>
      bb_associator_retriever =
          [&](const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                  &pg,
              const vtr::UnassociatedBoundingBoxOfflineProblemData<
                  vtr::StructuredVisionFeatureTrack,
                  sensor_msgs::Image::ConstPtr> &input_prob) {
            return roshan_associator_creator.getDataAssociator(pg);
          };
  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &)>
      frame_data_adder =
          [&](const vtr::UnassociatedBoundingBoxOfflineProblemData<
                  vtr::StructuredVisionFeatureTrack,
                  sensor_msgs::Image::ConstPtr> &problem_data,
              const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                  &pose_graph,
              const vtr::FrameId &frame_to_add) {
            vtr::addFrameDataAssociatedBoundingBox(problem_data,
                                                   pose_graph,
                                                   frame_to_add,
                                                   reprojection_error_provider,
                                                   bb_associator_retriever,
                                                   bb_context_retriever);
          };
  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<const vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      vtr::SpatialEstimateOnlyResults &)>
      output_data_extractor =
          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
                 vtr::StructuredVisionFeatureTrack,
                 sensor_msgs::Image::ConstPtr> &input_problem_data,
             const std::shared_ptr<
                 const vtr::ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
             vtr::SpatialEstimateOnlyResults &output_problem_data) {
            vtr::extractSpatialEstimateOnlyResults(pose_graph,
                                                   output_problem_data);
          };
  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &,
      const vtr::FrameId &)>
      ceres_callback_creator = dummyCeresCallbackCreator;
  std::function<void(
      const vtr::UnassociatedBoundingBoxOfflineProblemData<
          vtr::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr> &,
      const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph> &,
      const vtr::FrameId &,
      const vtr::FrameId &,
      const vtr::VisualizationTypeEnum &)>
      visualization_callback =
          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
                 vtr::StructuredVisionFeatureTrack,
                 sensor_msgs::Image::ConstPtr> &input_problem_data,
             const std::shared_ptr<vtr::ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             const vtr::FrameId &min_frame_id,
             const vtr::FrameId &max_frame_id,
             const vtr::VisualizationTypeEnum &visualization_type) {
            std::shared_ptr<vtr::ObjAndLowLevelFeaturePoseGraph<
                vtr::ReprojectionErrorFactor>>
                superclass_ptr = pose_graph;
            visualizationStub(input_problem_data,
                              superclass_ptr,
                              min_frame_id,
                              max_frame_id,
                              visualization_type);
          };
  pose_graph_optimization::OptimizationSolverParams solver_params;  // TODO
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params;

  vtr::OfflineProblemRunner<vtr::UnassociatedBoundingBoxOfflineProblemData<
                                vtr::StructuredVisionFeatureTrack,
                                sensor_msgs::Image::ConstPtr>,
                            vtr::ReprojectionErrorFactor,
                            vtr::SpatialEstimateOnlyResults,
                            util::EmptyStruct,
                            vtr::ObjectAndReprojectionFeaturePoseGraph>
      offline_problem_runner(residual_params,
                             continue_opt_checker,
                             window_provider_func,
                             refresh_residual_checker,
                             residual_creator,
                             pose_graph_creator,
                             frame_data_adder,
                             output_data_extractor,
                             ceres_callback_creator,
                             visualization_callback,
                             solver_params);

  // TODO populate all of these
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera;
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      camera_extrinsics_by_camera;
  std::unordered_map<vtr::FeatureId, vtr::StructuredVisionFeatureTrack>
      visual_features;
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses;
  std::unordered_map<
      std::string,
      std::pair<vtr::ObjectDim<double>, vtr::Covariance<double, 3>>>
      mean_and_cov_by_semantic_class;
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bounding_boxes;
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images;
  vtr::UnassociatedBoundingBoxOfflineProblemData<
      vtr::StructuredVisionFeatureTrack,
      sensor_msgs::Image::ConstPtr>
      input_problem_data(camera_intrinsics_by_camera,
                         camera_extrinsics_by_camera,
                         visual_features,
                         robot_poses,
                         mean_and_cov_by_semantic_class,
                         bounding_boxes,
                         images);

  vtr::OptimizationFactorsEnabledParams optimization_factors_enabled_params;
  optimization_factors_enabled_params.use_pom_ = false;
  optimization_factors_enabled_params.include_visual_factors_ = false;

  vtr::SpatialEstimateOnlyResults output_results;

  offline_problem_runner.runOptimization(
      input_problem_data, optimization_factors_enabled_params, output_results);

  // TODO save output results somewhere

  return 0;
}