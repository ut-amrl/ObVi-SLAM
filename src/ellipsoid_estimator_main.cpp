#include <bounding_box_factor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <shape_prior_factor.h>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // TODO fill in with ellipsoid estimation code (step 2)

  return 0;
}