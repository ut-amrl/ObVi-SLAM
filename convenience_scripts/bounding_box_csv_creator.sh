#!/bin/bash

extract_bounding_boxes() {
  bag_time_str=$1

  bb_type_variant=yolov3
  bb_in_file_prefix=bb_chairs_
  bag_file_prefix=chairs_
#  bb_type_variant=yolov5
#  bb_in_file_prefix=bb_cones_
#  bag_file_prefix=cones_

  bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/ltm_test_2022_09_30/
  bag_file_bb_dir=${bag_file_base_dir}yolo_output/${bb_type_variant}/

  bag_suffix=.bag
  txt_suffix=.txt

  bb_rosbag_file=${bag_file_bb_dir}${bag_file_prefix}${bag_time_str}${bag_suffix}
  bb_by_timestamp_file=${bag_file_bb_dir}${bb_in_file_prefix}${bag_time_str}${txt_suffix}

  python src/data_preprocessing_utils/extract_all_bounding_boxes.py  --bag_file ${bb_rosbag_file} \
    --out_file_name ${bb_by_timestamp_file}
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

for i in ${!bag_time_strings[@]}; do
  echo "i: ${i}"
  bag_time_string=${bag_time_strings[$i]}

  echo "Bag time string"
  echo ${bag_time_string}
  extract_bounding_boxes ${bag_time_string}
done



