#include <pangolin/pangolin.h>
#include <pose_viewer.h>

namespace vslam_viz {

PoseViewer::PoseViewer()
    : s_cam_(pangolin::OpenGlRenderState(
          pangolin::ProjectionMatrix(
              1024, 768, 1000, 1000, 512, 389, 0.1, 1000),
          pangolin::ModelViewLookAt(0.1, 0, 50, 0, 0, 0, 0.0, 0.0, -1.0))),
      handler_(s_cam_) {
  pangolin::CreateWindowAndBind("Main", 640, 480);
  glEnable(GL_DEPTH_TEST);

  d_cam_ =
      &pangolin::CreateDisplay()
           .SetBounds(
               0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
           .SetHandler(&handler_);
  d_cam_->Activate(s_cam_);  // If this is here it doesnt allow us to move the
  // camera in the window - should be in the callback
}

void PoseViewer::drawPoses(std::vector<vslam_types::SLAMNode> nodes) const {
  // Clear screen and activate view to render into
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  d_cam_->Activate(s_cam_);  // Needed here so we can move the window around

  // Extrinics
  vslam_types::CameraExtrinsics extrinsics{
      Eigen::Vector3f(0, 0, 0),
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse()};
  Eigen::Affine3d cam_to_robot_tf(
      Eigen::Translation3f(extrinsics.translation).cast<double>() *
      extrinsics.rotation.cast<double>());

  for (const auto& node : nodes) {
    Eigen::Transform<double, 3, Eigen::Affine> robot_pose_in_world =
        vslam_types::PoseArrayToAffine(&(node.pose[3]), &(node.pose[0]));

    Eigen::Transform<double, 3, Eigen::Affine> cam_mat =
        robot_pose_in_world * cam_to_robot_tf;

    pangolin::OpenGlMatrix Twc(cam_mat);

    DrawCamera(Twc);
  }

  // Swap frames and Process Events
  pangolin::FinishFrame();
  return;
}

void DrawCamera(pangolin::OpenGlMatrix const& Twc) {
  const float& w = 1;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();

#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif

  glLineWidth(1);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

}  // namespace vslam_viz