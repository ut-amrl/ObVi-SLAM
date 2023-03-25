#!/bin/bash

rosbag_name=$1

target_string="/all_observed_bbs/latest/cam_1_bb/camera_info"

topicRetrievalCommand="rosbag info ${rosbag_name} | grep ${target_string}"
#target_line=$(${topicRetrievalCommand})
eval target_line=\`${topicRetrievalCommand}\`
echo ${target_line}

if [ -z "${target_line}" ]
then
  echo "No matching topic found in bag"
  exit
else
  echo "Matching topic is ${topicInBag}"
fi


topicInBag=/${target_line#*/}
topicInBag=$(echo "${topicInBag}" | sed 's/camera_info.*/camera_info/')
echo ${topicInBag}

if [ -z "${topicInBag}" ]
then
  echo "No matching topic found in bag"
  exit
else
  echo "Matching topic is ${topicInBag}"
fi

topicPrefix=$(echo ${topicInBag} | sed 's/\/all_observed_bbs\/latest\/cam_1_bb\/camera_info//')
echo ${topicPrefix}

if [ -z "${topicPrefix}" ]
then
  roslaunch launch/ovslam_rviz.launch &
else
  underscore_based_prefix=${topicPrefix#*/}
  underscore_based_prefix=${underscore_based_prefix%*/}
  echo ${underscore_based_prefix}
  roslaunch launch/ovslam_rviz.launch topics_prefix:=${topicPrefix} underscore_based_prefix:=${underscore_based_prefix} &
fi

rosbag play --clock -r 40 ${rosbag_name} --pause


#suffix_removed_string


