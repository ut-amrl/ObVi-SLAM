#ifndef UT_VSLAM_POSE_VIEWER_H
#define UT_VSLAM_POSE_VIEWER_H

#include <pangolin/pangolin.h>
#include <vslam_types.h>

#include <vector>

namespace vslam_viz {
void DrawCamera(pangolin::OpenGlMatrix const& Twc);

class PoseViewer {
 public:
  PoseViewer();
  void drawPoses(std::vector<vslam_types::SLAMNode> nodes) const;

 private:
  pangolin::OpenGlRenderState s_cam_;
  pangolin::Handler3D handler_;
  pangolin::View* d_cam_;
};
}  // namespace vslam_viz

#endif  // UT_VSLAM_POSE_VIEWER_H
