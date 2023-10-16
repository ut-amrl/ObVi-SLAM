WORKDIR=/root/ObVi-SLAM-Evaluation/

oa_slam_dir=${WORKDIR}OA-SLAM/
camera_file=${oa_slam_dir}/Cameras/jackal_zed_rectified.yaml
root_data_dir=${WORKDIR}data/

sequence_file_base_name="evaluation_2023_07_v1_00_15"
# TODO fix me
sequence_file=/root/ObVi-SLAM-Evaluation/ObVi-SLAM/sequences/${sequence_file_base_name}.json

cd ${oa_slam_dir}

./bin/oa-slam_stereo_multi_sessions \
    Vocabulary/ORBvoc.txt \
    ${camera_file} \
    ${root_data_dir} \
    ${sequence_file} \
    null \
    points+objects