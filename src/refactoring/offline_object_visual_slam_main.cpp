#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/optimization/residual_creator.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

std::vector<std::shared_ptr<ceres::IterationCallback>>
dummyCeresCallbackCreator(
    const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
        vslam_types_refactor::StructuredVisionFeatureTrack,
        sensor_msgs::Image::ConstPtr,
        vslam_types_refactor::RawBoundingBoxObservation> &input_problem_data,
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph,
    const vslam_types_refactor::FrameId &min_frame_optimized,
    const vslam_types_refactor::FrameId &max_frame_optimized) {
  // TODO replace with actual code later
  return {};
}

bool checkFactorRefresh(
    const std::pair<vslam_types_refactor::FactorType,
                    vslam_types_refactor::FeatureFactorId> &factor,
    const std::shared_ptr<
        vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
    const bool &) {
  return false;
}

void createPoseGraph(
    const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
        vslam_types_refactor::StructuredVisionFeatureTrack,
        sensor_msgs::Image::ConstPtr,
        vslam_types_refactor::RawBoundingBoxObservation> &input_problem_data,
    std::shared_ptr<vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
        &pose_graph) {
  pose_graph = std::make_shared<
      vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>(input_problem_data.getObjDimMeanAndCovByClass(), input_problem_data.getCameraExtrinsicsByCamera(), input_problem_data.getCameraIntrinsicsByCamera());

}

void visualizationStub(
    const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
        vslam_types_refactor::StructuredVisionFeatureTrack,
        sensor_msgs::Image::ConstPtr,
        vslam_types_refactor::RawBoundingBoxObservation> &input_problem_data,
    std::shared_ptr<vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
        vslam_types_refactor::ReprojectionErrorFactor>> &pose_graph,
    const vslam_types_refactor::FrameId &min_frame_optimized,
    const vslam_types_refactor::FrameId &max_frame_optimized,
    const vslam_types_refactor::VisualizationTypeEnum &visualization_stage) {
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

  std::function<vslam_types_refactor::FrameId(
      const vslam_types_refactor::FrameId &)>
      window_provider_func = [](const vslam_types_refactor::FrameId &) {
        // For now, we'll just optimize the whole trajectory (so return 0 so we
        // start the optimization with node 0
        return 0;
      };

  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
      const bool &)>
      refresh_residual_checker = checkFactorRefresh;

  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
      bool &)>
      cached_info_creator =
          [](const std::pair<vslam_types_refactor::FactorType,
                             vslam_types_refactor::FeatureFactorId>
                 &factor_info,
             const std::shared_ptr<
                 vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             bool &cached_info) {
            return true;  // TODO maybe fill in with real info some day
          };
  std::function<bool(
      const std::pair<vslam_types_refactor::FactorType,
                      vslam_types_refactor::FeatureFactorId> &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      bool &)>
      residual_creator =
          [&](
              const std::pair<vslam_types_refactor::FactorType,
                              vslam_types_refactor::FeatureFactorId> &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const std::shared_ptr<
                  vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
                  &pose_graph,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              bool &cached_info) {
            return vslam_types_refactor::createResidual(factor_id,
                                                        pose_graph,
                                                        solver_residual_params,
                                                        cached_info_creator,
                                                        problem,
                                                        residual_id,
                                                        cached_info);
          };
  std::function<void(
      const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
          vslam_types_refactor::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr,
          vslam_types_refactor::RawBoundingBoxObservation> &,
      std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &)>
      pose_graph_creator = createPoseGraph;
  std::function<void(
      const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
          vslam_types_refactor::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr,
          vslam_types_refactor::RawBoundingBoxObservation> &,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
      const vslam_types_refactor::FrameId &)>
      frame_data_adder;  // TODO
  std::function<void(
      const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
          vslam_types_refactor::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr,
          vslam_types_refactor::RawBoundingBoxObservation> &,
      const std::shared_ptr<
          const vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
      vslam_types_refactor::SpatialEstimateOnlyResults &)>
      output_data_extractor =
          [](const vslam_types_refactor::
                 UnassociatedBoundingBoxOfflineProblemData<
                     vslam_types_refactor::StructuredVisionFeatureTrack,
                     sensor_msgs::Image::ConstPtr,
                     vslam_types_refactor::RawBoundingBoxObservation>
                     &input_problem_data,
             const std::shared_ptr<const vslam_types_refactor::
                                       ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             vslam_types_refactor::SpatialEstimateOnlyResults
                 &output_problem_data) {
            vslam_types_refactor::extractSpatialEstimateOnlyResults(
                pose_graph, output_problem_data);
          };
  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
          vslam_types_refactor::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr,
          vslam_types_refactor::RawBoundingBoxObservation> &,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
      const vslam_types_refactor::FrameId &,
      const vslam_types_refactor::FrameId &)>
      ceres_callback_creator = dummyCeresCallbackCreator;
  std::function<void(
      const vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
          vslam_types_refactor::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr,
          vslam_types_refactor::RawBoundingBoxObservation> &,
      const std::shared_ptr<
          vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph> &,
      const vslam_types_refactor::FrameId &,
      const vslam_types_refactor::FrameId &,
      const vslam_types_refactor::VisualizationTypeEnum &)>
      visualization_callback =
          [](const vslam_types_refactor::
                 UnassociatedBoundingBoxOfflineProblemData<
                     vslam_types_refactor::StructuredVisionFeatureTrack,
                     sensor_msgs::Image::ConstPtr,
                     vslam_types_refactor::RawBoundingBoxObservation>
                     &input_problem_data,
             const std::shared_ptr<
                 vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
                 &pose_graph,
             const vslam_types_refactor::FrameId &min_frame_id,
             const vslam_types_refactor::FrameId &max_frame_id,
             const vslam_types_refactor::VisualizationTypeEnum
                 &visualization_type) {
            std::shared_ptr<
                vslam_types_refactor::ObjAndLowLevelFeaturePoseGraph<
                    vslam_types_refactor::ReprojectionErrorFactor>>
                superclass_ptr = pose_graph;
            visualizationStub(input_problem_data,
                              superclass_ptr,
                              min_frame_id,
                              max_frame_id,
                              visualization_type);
          };
  pose_graph_optimization::OptimizationSolverParams solver_params;  // TODO
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params;

  vslam_types_refactor::OfflineProblemRunner<
      vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
          vslam_types_refactor::StructuredVisionFeatureTrack,
          sensor_msgs::Image::ConstPtr,
          vslam_types_refactor::RawBoundingBoxObservation>,
      vslam_types_refactor::ReprojectionErrorFactor,
      vslam_types_refactor::SpatialEstimateOnlyResults,
      bool,
      vslam_types_refactor::ObjectAndReprojectionFeaturePoseGraph>
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

  vslam_types_refactor::UnassociatedBoundingBoxOfflineProblemData<
      vslam_types_refactor::StructuredVisionFeatureTrack,
      sensor_msgs::Image::ConstPtr,
      vslam_types_refactor::RawBoundingBoxObservation>
      input_problem_data;  // TODO

  vslam_types_refactor::OptimizationFactorsEnabledParams
      optimization_factors_enabled_params;
  optimization_factors_enabled_params.use_pom_ = false;
  optimization_factors_enabled_params.include_visual_factors_ = false;

  vslam_types_refactor::SpatialEstimateOnlyResults output_results;

  offline_problem_runner.runOptimization(
      input_problem_data, optimization_factors_enabled_params, output_results);

  // TODO save output results somewhere

  return 0;
}