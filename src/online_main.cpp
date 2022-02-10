#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <iostream>

#include "slam_backend_solver.h"
#include "slam_solver_optimizer_params.h"
#include "visual_slam_ceres_visualization_callback.h"
#include "vslam_io.h"
#include "vslam_types.h"
#include "vslam_util.h"

using namespace std;
using namespace ros;
using namespace vslam_types;
using namespace vslam_util;

UTSLAMProblemOnline<StructuredFrameTrack> *slam_problem_ptr = nullptr;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ut_vslam_online");
  ros::NodeHandle n;

  slam_problem_ptr = new UTSLAMProblemOnline<StructuredFrameTrack>();

  // load Intrinsics and Extrinsics into slam_problem_ptr

  // subscribers - load values into slam_problem_ptr
  // Need: RobotPose, vector<VisionFeature> 
  // don't call AdjustTrajectoryToStartAtZero(); call it when saving poses instead

  // estimated 3D Feature positions 
  // TODO mean or the first one?
  // store in slam_problem_ptr

  ros::Rate loop_rate(20.0);
  while (slam_problem_ptr && ros::ok()) {
    ros::spinOnce();
    // solve slam problem
    
    loop_rate.sleep();
  }

  return 0;
}