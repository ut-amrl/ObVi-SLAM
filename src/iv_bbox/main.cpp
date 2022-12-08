#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iv_bbox/iv_bbox.h>

DEFINE_string(config, "", "");
using namespace vslam_types_refactor;
using namespace iv_bbox;

int main(int argc, char **argv) {


  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // CameraIntrinsics<float> intrinsics(670.95818, 673.34226, 617.95349, 518.92384); // TODO read calibrations from config file
  // CameraExtrinsics<float> extrinsics;
  // extrinsics.transl_ = Eigen::Vector3f(0,0,-0.1);
  // extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.998, 0.0, 0.085, 0.0).normalized()) * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();

  CameraIntrinsics<float> intrinsics(730.5822, 729.8109, 609.5004, 539.4311);
  CameraExtrinsics<float> extrinsics;
  // extrinsics.transl_ = Eigen::Vector3f(0.01, 0.1, -0.106);
  // extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.99549089, 0.0, 0.1, 0.0)) 
  //                             * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();

  // extrinsics.transl_ = Eigen::Vector3f(0.01, 0.1, -0.1);
  // extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.99549089, 0.0, 0.09485717, 0.0)) 
  //                             * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();

  extrinsics.transl_ = Eigen::Vector3f(0.01, 0.1, -0.1);
  extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.99549089, 0.0, 0.07, 0.0)) 
                              * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();

  IVBBox ivBBox(intrinsics, extrinsics, FLAGS_config);
  ivBBox.createDataset(true);

}