//
// Created by amanda on 7/12/23.
//

#ifndef UT_VSLAM_RUN_OPT_UTILS_H
#define UT_VSLAM_RUN_OPT_UTILS_H

#include <ceres/iteration_callback.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/optimization/residual_creator.h>
#include <sensor_msgs/Image.h>

namespace vslam_types_refactor {

typedef ObjectAndReprojectionFeaturePoseGraph MainPg;
typedef std::shared_ptr<MainPg> MainPgPtr;
typedef IndependentEllipsoidsLongTermObjectMap<util::EmptyStruct> MainLtm;
typedef std::shared_ptr<MainLtm> MainLtmPtr;
typedef std::pair<FactorType, vslam_types_refactor::FeatureFactorId>
    MainFactorInfo;

typedef UnassociatedBoundingBoxOfflineProblemData<StructuredVisionFeatureTrack,
                                                  sensor_msgs::Image::ConstPtr,
                                                  MainLtm>
    MainProbData;

bool checkFactorRefresh(const MainFactorInfo &factor,
                        const MainPgPtr &,
                        const util::EmptyStruct &) {
  return false;
}
std::vector<std::shared_ptr<ceres::IterationCallback>>
dummyCeresCallbackCreator(const MainProbData &input_problem_data,
                          const MainPgPtr &pose_graph,
                          const FrameId &min_frame_optimized,
                          const FrameId &max_frame_optimized) {
  // TODO replace with actual code later
  return {};
}

bool dummyCachedInfoCreator(const MainFactorInfo &,
                            const MainPgPtr &,
                            util::EmptyStruct &) {
  return true;  // TODO maybe fill in with real info some day
}

std::function<
    bool(const MainFactorInfo &factor_id,
         const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
             &solver_residual_params,
         const MainPgPtr &pose_graph,
         const bool &debug,
         ceres::Problem *problem,
         ceres::ResidualBlockId &residual_id,
         util::EmptyStruct &cached_info)>
generateResidualCreator(
    std::function<bool(
        const MainFactorInfo &,
        const MainPgPtr &,
        const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
        const std::function<bool(
            const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)> &,
        ceres::Problem *,
        ceres::ResidualBlockId &,
        util::EmptyStruct &)> &long_term_map_residual_creator_func) {
  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)>
      cached_info_creator = dummyCachedInfoCreator;

  std::function<bool(
      const MainFactorInfo &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const MainPgPtr &,
      const bool &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator =
          [&](const MainFactorInfo &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const MainPgPtr &pose_graph,
              const bool &debug,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return createResidual(factor_id,
                                  pose_graph,
                                  solver_residual_params,
                                  cached_info_creator,
                                  long_term_map_residual_creator_func,
                                  problem,
                                  residual_id,
                                  cached_info,
                                  std::nullopt,
                                  debug);
          };
  return residual_creator;
}

FrameId provideOptimizationWindow(
    const FrameId &max_frame_to_opt,
    const FrameId &max_frame_id,
    const SlidingWindowParams &sliding_window_params) {
  // Optimize the full trajectory when we're on the last frame
  if (max_frame_to_opt == max_frame_id) {
    return 0;
  }
  if ((max_frame_to_opt % sliding_window_params.global_ba_frequency_) == 0) {
    return 0;
  }
  if (max_frame_to_opt < sliding_window_params.local_ba_window_size_) {
    return 0;
  }
  return max_frame_to_opt - sliding_window_params.local_ba_window_size_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_RUN_OPT_UTILS_H
