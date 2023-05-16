//
// Created by amanda on 5/6/23.
//

#include <file_io/cv_file_storage/sequence_file_storage_io.h>
#include <file_io/file_access_utils.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <types/sequence_utils.h>

using namespace vslam_types_refactor;

DEFINE_string(sequence_file_directory, "", "Directory for sequence files");
DEFINE_string(sequence_id, "", "Id/name for the sequence");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  CHECK(!FLAGS_sequence_file_directory.empty())
      << "Sequence file directory cannot be empty";
  CHECK(!FLAGS_sequence_id.empty()) << "Sequence id cannot be empty";

  std::vector<BagBaseNameAndWaypointFile> sequence_file_entries;

  sequence_file_entries.emplace_back(BagBaseNameAndWaypointFile(
      "1676766718", "waypoint_stamps_v1_1676766718"));
  sequence_file_entries.emplace_back(BagBaseNameAndWaypointFile(
      "1677097326", "waypoint_stamps_v1_1677097326"));

  std::string sequence_file_name =
      file_io::ensureDirectoryPathEndsWithSlash(FLAGS_sequence_file_directory) +
      FLAGS_sequence_id + file_io::kJsonExtension;

  SequenceInfo sequence_info;
  sequence_info.sequence_id_ = FLAGS_sequence_id;
  sequence_info.bag_base_names_and_waypoint_files = sequence_file_entries;

  writeSequenceInfo(sequence_file_name, sequence_info);

  return 0;
}