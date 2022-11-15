#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iv_bbox/iv_bbox.h>

DEFINE_string(config, "", "");
using namespace vslam_types_refactor;
using namespace iv_bbox;

int main(int argc, char **argv) {


  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // CameraIntrinsics<float> intrinsics(266.6545104980469, 266.6545104980469, 327.12762451171875, 181.9247589111328);
  CameraIntrinsics<float> intrinsics(670.95818, 673.34226, 617.95349, 518.92384); // TODO read calibrations from config file
  // CameraIntrinsics<float> intrinsics(681.99707, 681.99707, 596.75923, 503.14468); // TODO read calibrations from config file
  CameraExtrinsics<float> extrinsics;
  // extrinsics.transl_ = Eigen::Vector3f(0,0,0.7874);
  // extrinsics.transl_ = Eigen::Vector3f(0,0,-0.2);
  extrinsics.transl_ = Eigen::Vector3f(0,0,-0.1);
  // extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0)) * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();
  // extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.987, 0.0, 0.163, 0.0)) * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();
  extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.998, 0.0, 0.085, 0.0).normalized()) * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();
  Eigen::Affine3f robot_to_cam_tf;
  robot_to_cam_tf = Eigen::Translation3f(extrinsics.transl_) * (extrinsics.orientation_ * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse());
  robot_to_cam_tf = robot_to_cam_tf.inverse();

  IVBBox ivBBox(intrinsics, extrinsics, FLAGS_config);
  ivBBox.createDataset();

}