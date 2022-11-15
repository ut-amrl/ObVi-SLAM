#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iv_bbox/iv_bbox_utils.h>

DEFINE_string(config, "", "");
using namespace vslam_types_refactor;
using namespace iv_bbox;



int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);


}