#!/bin/bash

interpolate_bounding_boxes() {
  bag_time_str=$1
  bb_type_variant=$2
  bb_in_file_prefix=$3


#  bb_type_variant=merged
#  bb_in_file_prefix=bb_merged_
#  bb_type_variant=yolov3
#  bb_in_file_prefix=bb_chairs_
#  bb_type_variant=yolov5
#  bb_in_file_prefix=bb_cones_

  bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/ltm_test_2022_09_30/
  bag_file_orig_dir=${bag_file_base_dir}original/
  bag_file_bb_dir=${bag_file_base_dir}yolo_output/${bb_type_variant}/
  bag_file_loc_dir=${bag_file_base_dir}localization/${bb_type_variant}/

  bag_suffix=.bag
  txt_suffix=.txt

  spacing_suffix=spaced_out_0_1_
#  spacing_suffix=""

  bounding_box_file_prefix=bb_by_node_
  poses_by_node_prefix=localization_est_by_node_id_
  timestamps_by_node_prefix=timestamps_by_node_id_

  rosbag_file=${bag_file_orig_dir}${bag_time_str}${bag_suffix}
  bb_by_timestamp_file=${bag_file_bb_dir}${bb_in_file_prefix}${bag_time_str}${txt_suffix}
  bounding_boxes_by_node_id_file=${bag_file_bb_dir}${bounding_box_file_prefix}${spacing_suffix}${bag_time_str}${txt_suffix} # TODO fix
  poses_by_node_id_file=${bag_file_loc_dir}${poses_by_node_prefix}${spacing_suffix}${bag_time_str}${txt_suffix} # TODO fix
  nodes_by_timestamp_file=${bag_file_loc_dir}${timestamps_by_node_prefix}${spacing_suffix}${bag_time_str}${txt_suffix} # TODO fix

  make && ./bin/localization_rosbag_extraction_and_interpolation_for_bounding_boxes --rosbag_file_name ${rosbag_file} \
  --bb_by_timestamp_file_no_association ${bb_by_timestamp_file} --bb_by_node_out_file \
  ${bounding_boxes_by_node_id_file} --localization_est_out_file ${poses_by_node_id_file} --timestamp_by_node_id_file ${nodes_by_timestamp_file}
}

bag_time_missing_cone_1=2022-09-30-13-59-54
bag_time_missing_cone_2=2022-09-30-14-01-22
bag_time_sunny_cone_1=2022-09-30-14-03-17
bag_time_sunny_cone_2=2022-09-30-14-05-01
bag_time_shaded_cone_1=2022-09-30-14-07-18
bag_time_shaded_cone_2=2022-09-30-14-09-22
bag_time_shaded_cone_3=2022-09-30-14-11-04
bag_time_missing_cone_3=2022-09-30-14-15-27

bag_time_strings=(${bag_time_missing_cone_1} ${bag_time_missing_cone_2} ${bag_time_sunny_cone_1} ${bag_time_sunny_cone_2} ${bag_time_shaded_cone_1} ${bag_time_shaded_cone_2} ${bag_time_shaded_cone_3} ${bag_time_missing_cone_3})
#bag_time_strings=(${bag_time_shaded_cone_3})

  bb_type_variant_merged=merged
  bb_in_file_prefix_merged=bb_merged_
  bb_type_variant_chairs=yolov3
  bb_in_file_prefix_chairs=bb_chairs_
  bb_type_variant_cones=yolov5
  bb_in_file_prefix_cones=bb_cones_

for i in ${!bag_time_strings[@]}; do
  echo "i: ${i}"
  bag_time_string=${bag_time_strings[$i]}

  echo "Bag time string"
  echo ${bag_time_string}
  interpolate_bounding_boxes ${bag_time_string} ${bb_type_variant_merged} ${bb_in_file_prefix_merged}
  interpolate_bounding_boxes ${bag_time_string} ${bb_type_variant_chairs} ${bb_in_file_prefix_chairs}
  interpolate_bounding_boxes ${bag_time_string} ${bb_type_variant_cones} ${bb_in_file_prefix_cones}
done



