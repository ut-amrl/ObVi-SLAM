#include <pangolin/pangolin.h>
#include <pose_viewer.h>

namespace vslam_viz {

PoseViewer::PoseViewer()
    : s_cam_(pangolin::OpenGlRenderState(
          pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
          pangolin::ModelViewLookAt(-20, 20, -20, 0, 0, 0, pangolin::AxisY))),
      handler_(s_cam_) {
  pangolin::CreateWindowAndBind("Main", 640, 480);
  glEnable(GL_DEPTH_TEST);

  d_cam_ = &pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                .SetHandler(&handler_);
  d_cam_->Activate(s_cam_);  // If this is here it doesnt allow us to move the
  // camera in the window - should be in the callback
}

void PoseViewer::drawPoses(std::vector<vslam_types::SLAMNode> nodes) const {
  // Clear screen and activate view to render into
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  d_cam_->Activate(s_cam_);  // Needed here so we can move the window around

  // Render OpenGL Cube
  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();
  for (const auto& node : nodes) {
    DrawCamera(Twc.Translate(-node.pose[1], -node.pose[2], node.pose[0]));
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