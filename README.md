# ObVi-SLAM
ObVi-SLAM is a joint object-visual SLAM approach aimed at long-term multi-session robot deployments. 

[[Paper with added appendix](https://arxiv.org/abs/2309.15268)] [[Video](https://youtu.be/quJOgnEdaZ0)]

Offline execution instructions coming soon. 
ROS implementation coming late 2023/early 2024. 

Please email amanda.adkins4242@gmail.com with any questions! 


## Evaluation
For information on how to set up and run the comparison algorithms, see our [evaluation repo](https://github.com/ut-amrl/ObVi-SLAM-Evaluation).

## Installation Instructions
<!-- TODO
- dockerfile version (recommended)
- native version -->

We rely on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) to extract visual features from image observations and motion tracking, [YOLO](https://github.com/ultralytics/yolov5) to detect objects, and [amrl_msgs](https://github.com/ut-amrl/amrl_msgs.git) to publish and subscribe to SLAM-related ROS messages. We provide our own customized versions of these libraries to facilitate your use: [ORB-SLAM2](https://github.com/ut-amrl/ORB_SLAM2), [YOLO](https://github.com/ut-amrl/yolov5), and [amrl_msgs](https://github.com/ut-amrl/amrl_msgs.git). To install them inside the container:
```
# compile ORB-SLAM2
git clone https://github.com/ut-amrl/ORB_SLAM2 ORB_SLAM2
git checkout writeTimestamps
cd ORB_SLAM2
chmod +x build.sh
./build.sh

# install amrl_msgs
git clone https://github.com/ut-amrl/amrl_msgs.git
cd amrl_msgs
git checkout orbSlamSwitchTraj
echo "export ROS_PACKAGE_PATH="`pwd`":$ROS_PACKAGE_PATH" >> ~/.bashrc
source ~/.bashrc
make

# install yolo
git clone https://github.com/ut-amrl/yolov5
cd yolov5
git checkout ROS
```
If you want to install them outside the container, refer to their README page for further instruction. Our [YOLO](https://github.com/ut-amrl/yolov5) package depends on [amrl_msgs](https://github.com/ut-amrl/amrl_msgs.git). You will need to install [amrl_msgs](https://github.com/ut-amrl/amrl_msgs.git) first to use our YOLO detector for ROS.

## Minimal Execution Instructions
<!-- TODO
- Explain files needed and their structure (intrinsics, extrinsics, visual features, bounding box (opt), images?,
- Explain how to run given these files -->
This is a quick start guide to run ObVi-SLAM from ROS Bagfiles inside the docker container. First, refer to [this page](https://github.com/ut-amrl/ros-noetic-docker/tree/ObViSLAMEvaluation) for Docker setup.

### File structures
By default, we assume to have a following data root directory structures:
```
root_data_dir
|- calibration (stores calibration files)
|-- base_link_to_os1.txt
|-- base_link_to_zed_left_camera_optical_frame.txt
|-- base_link_to_zed_right_camera_optical_frame.txt
|-- camera_matrix.txt
|-- extrinsics.txt
|-- lego_loam_bl.txt
|-- ov_slam_bl.txt
|-- odom_bl.txt
|- original_data (stores raw bagfiles)
|-- <baganme1.bag>
|-- <baganme2.bag>
|-- <...>
|- orb_out (stores feature and pose initialization from ORB-SLAM2 Frontend)
|- orb_post_process (processed ORB-SLAM2 Frontend results)
|- ut_vslam_results (ObVi-SLAM results)
|- lego_loam_out (Optional; stores pose estimations for LeGOLOAM)
```

### Calibration Files
To start, we require the following calibration files under a directory named "calibration" under the root data directory:
- base_link_to_os1.txt - Transformation from base link to the LIDAR frame in the form of "transl_x, transl_y, transl_z, quat_x, quat_y, quat_z, quat_w". The first line is the header, and raw data starts from the second line. (The following transformation files share the same format.)
- base_link_to_zed_left_camera_optical_frame.txt - Transformation from base link to the left camera frame
- base_link_to_zed_right_camera_optical_frame.txt - Transformation from base link to the right camera frame
- camera_matrix.txt - Camera instrinsics file in the form of "camera_id, img_width, img_height, mat_00, mat_01, mat_02, mat_10, mat_11, mat_12, mat_20, mat_21, mat_22". The first line is the header, and raw data starts from the second line. In our calibration file, left camera is referred by camera id 1, and the right one has camera id 2.
- extrinsics.txt - Camera extrinsics file. It transform the map frame to the camera frame (world coordinate --> camera coordinate). Order and ids of cameras should match camera_matrix.txt file.
- lego_loam_bl.txt (Optional) - Transformation from base link to the base frame of LeGO-LOAM. By default it symlinks to base_link_to_os1.txt. This file is only required for evaluation.
- ov_slam_bl.txt (Optional) - Transformation from base link to the base frame of ObVi-SLAM. By default it is an identity transform. This file is only required for evaluation.
- odom_bl.txt (Optional) - Transformation from base link to the odometry frame. By default it is an identity transform too. This file is only required for evaluation.

If you have different namings for those files, you can either softlink or change the calibration filenames inside the evaluation scripts later.

<!-- ## Results from ROS bag sequence
TODO (Taijing, start here)
- Explain how to preprocess rosbag to get the data needed for minimal execution above -->
### Feature Frontend
Open two terminals and run the following commands to uncompress image topics:
```
rosrun image_transport republish compressed in:=/zed/zed_node/left/image_rect_color raw  out:=/camera/left/image_raw
rosrun image_transport republish compressed in:=/zed/zed_node/right/image_rect_color raw out:=/camera/right/image_raw
```
In a third terminal, run:
```
bash ./convenience_scripts/docker/high_res_orbslam2_multibags_executor.sh
```
This script will preprocess all the bagfiles in sequence files specified by `sequence_file_base_names`. It reads camera calibarations from the file indicated by `orb_slam_configuration_file`. You may want to change it to point to your own camera calibration file. If you want to run for your own bagfiles, you can create your own sequence file and replace `sequence_file_base_names` with the one you created.

### Object Detection
We used YOLOv5 for object detection. To run the object detector, in a new terminal run:
```
cd <path to yolov5 project root directory>
python3 detect_ros.py --weights <path to model weight> --img <image width in pixels> --conf <detection confidence threshold>
```
This script will start a ROS service named `yolov5_detect_objs` that allows users to query bounding boxes by image observations. 

### Run ObVi-SLAM
Run:
```
bash ./convenience_scripts/docker/high_res_ut_vslam_sequence_executor.sh
```
In the script, `sequence_file_base_name` is the filename of the sequence file (without suffix ".json"), and `config_file_base_name` is the filename of the configuration file (withouth suffix ".yaml"). You can change them to match with your sequence file and configuration file setups.


<!-- ## Configuration File Guide
TODO 
- Explain how to modify configuration file -- which parameters will someone need to modify for different environment, (lower priority): explain each of the parameters in the config file

## Evaluation
For our experiments, we used [YOLOv5](https://github.com/ut-amrl/yolov5/tree/ROS) (based on [this repo](https://github.com/ultralytics/yolov5)) with [this model](https://drive.google.com/file/d/15xv-Se991Pzes7R3KfyPBkuSQ7TeCb1T/view?usp=sharing). 

We used detections with labels 'lamppost', 'treetrunk', 'bench', and 'trashcan' with [this configuration file](https://github.com/ut-amrl/ObVi-SLAM/blob/main/config/base7a_1_fallback_a_2.json). 

Please contact us if you would like to obtain the videos on which we performed the evaluation. 

## TODOs
- Add installation instructions
- Add offline execution instructions

