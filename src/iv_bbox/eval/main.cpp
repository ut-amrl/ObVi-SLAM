#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iv_bbox/iv_bbox.h>

DEFINE_string(config, "", "");
using namespace vslam_types_refactor;
using namespace iv_bbox;

int main(int argc, char **argv) {


  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CameraIntrinsics<float> intrinsics(266.6545104980469, 266.6545104980469, 327.12762451171875, 181.9247589111328);
  CameraExtrinsics<float> extrinsics;
  // extrinsics.transl_ = Eigen::Vector3f(0,0,0.7874);
  extrinsics.transl_ = Eigen::Vector3f(0,0,-0.2);
  extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0)) * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();
  // extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.987, 0.0, 0.163, 0.0)) * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();
  Eigen::Affine3f robot_to_cam_tf;
  robot_to_cam_tf = Eigen::Translation3f(extrinsics.transl_) * (extrinsics.orientation_ * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse());
  robot_to_cam_tf = robot_to_cam_tf.inverse();

  IVBBox ivBBox(intrinsics, extrinsics, FLAGS_config);
  ivBBox.createDataset();
  // IVBBox ivBBox(intrinsics, extrinsics, "/home/tiejean/projects/ut_semantic_vslam/configs/iv_bbox/cones.yaml");

}