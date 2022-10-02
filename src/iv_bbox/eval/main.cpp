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

#if 0
  iv_bbox::ImgArr stampedImages;
  iv_bbox::parseCompressedImage(FLAGS_bagfile, "/camera/rgb/image_raw/compressed", stampedImages);

  Position3d<float> ellipsoidPos;
  ellipsoidPos << (float)1.88, (float)-0.07, (float)0.25;
  Orientation3D<float> ellipsoidOrient(0, Eigen::Vector3f(0, 0, 1));
  Pose3D<float> ellipsoidPose;
  ellipsoidPose.transl_ = ellipsoidPos;
  ellipsoidPose.orientation_ = ellipsoidOrient;
  ObjectDim<float> ellipsoidDim;
  ellipsoidDim << (float)0.3, (float)0.3, (float)0.5;
  EllipsoidState<float> ellipsoidState(ellipsoidPose, ellipsoidDim);

  vslam_types_refactor::Position3d<float> robotPosePos;
  robotPosePos << (float)0.0, (float)0.0, (float)0.0;
  Orientation3D<float> robotPoseOrient = Eigen::AngleAxisf(Eigen::Quaternionf(1, 0, 0, 0));
  Pose3D<float> robotPose;s
  robotPose.transl_ = robotPosePos;
  robotPose.orientation_ = robotPoseOrient;

  BbCornerPair<float> bbCornerPair;
  bbCornerPair = getCornerLocationsPair(ellipsoidState, robotPose, extrinsics, intrinsics.camera_mat);
  
  float min_x, min_y, max_x, max_y;
  min_x = std::max((float)0,    bbCornerPair.first[0]);
  min_y = std::max((float)0,    bbCornerPair.first[1]);
  max_x = std::min((float)1280, bbCornerPair.second[0]);
  max_y = std::min((float)720,  bbCornerPair.second[1]);
  cout << min_x << "," << min_y << "," << max_x << "," << max_y << endl;
  cv::Mat img = stampedImages[0].second->clone();
  cv::rectangle(img, 
                cv::Point(min_x,min_y),
                cv::Point(max_x,max_y),
                cv::Scalar(0, 255, 0), 2);
  cv::imwrite("test.png", img);
#endif

  IVBBox ivBBox(intrinsics, extrinsics, FLAGS_config);
  ivBBox.createDataset();
  // IVBBox ivBBox(intrinsics, extrinsics, "/home/tiejean/projects/ut_semantic_vslam/configs/iv_bbox/cones.yaml");

}