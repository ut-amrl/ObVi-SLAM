// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <file_io/cv_file_storage/sequence_file_storage_io.h>
#include <gtest/gtest.h>

#include <filesystem>

using namespace vslam_types_refactor;
namespace fs = std::filesystem;

TEST(SequenceInfo, ReadSequence) {
  std::string seq_file_name = "sequences/3059_9589.json";
  SequenceInfo read_seq;
  readSequenceInfo(seq_file_name, read_seq);
  ASSERT_EQ("3059_9589", read_seq.sequence_id_);
  ASSERT_EQ(2, read_seq.bag_base_names_and_waypoint_files.size());
  ASSERT_EQ("1669743059",
            read_seq.bag_base_names_and_waypoint_files[0].bag_base_name_);
  ASSERT_FALSE(read_seq.bag_base_names_and_waypoint_files[0]
                   .optional_waypoint_file_base_name_.has_value());
  ASSERT_EQ("1668019589",
            read_seq.bag_base_names_and_waypoint_files[1].bag_base_name_);
  ASSERT_FALSE(read_seq.bag_base_names_and_waypoint_files[1]
                   .optional_waypoint_file_base_name_.has_value());
}