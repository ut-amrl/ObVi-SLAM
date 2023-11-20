# ObVi-SLAM
ObVi-SLAM is a joint object-visual SLAM approach aimed at long-term multi-session robot deployments. 

[[Paper](https://arxiv.org/abs/2309.15268)] [[Video](https://youtu.be/quJOgnEdaZ0)]

Offline execution instructions coming soon. 
ROS implementation coming late 2023/early 2024. 

Please email amanda.adkins4242@gmail.com with any questions! 


## Evaluation
For information on how to set up and run the comparison algorithms, see our [evaluation repo](https://github.com/ut-amrl/ObVi-SLAM-Evaluation).


## Extended Results
See the [version of our paper](https://drive.google.com/file/d/1Cf6QfheKa09mJO8oqgUqdTUC3y12JXRN/view?usp=share_link) with an appendix containing extended results and the full ablation study details.


## Installation Instructions
TODO
- dockerfile version (recommended)
- native version

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

This is a quick start guide to run ObVi-SLAM from ROS Bagfiles. To start, we require the following calibration files under a directory named "calibration" under the root data directory:
- base_link_to_os1.txt - Transformation from base link to the LIDAR frame in the form of "transl_x, transl_y, transl_z, quat_x, quat_y, quat_z, quat_w". (The following transformation files share the same format.)
- base_link_to_zed_left_camera_optical_frame.txt - Transformation from base link to the left camera frame
- base_link_to_zed_right_camera_optical_frame.txt - Transformation from base link to the right camera frame
- camera_matrix.txt - Camera instrinsics file in the form of "camera_id, img_width, img_height, mat_00, mat_01, mat_02, mat_10, mat_11, mat_12, mat_20, mat_21, mat_22". In our calibration file, left camera is referred by camera id 1, and the right one has camera id 2.
- extrinsics.txt - Camera extrinsics file. It transform the map frame to the camera frame (world coordinate --> camera coordinate)
- lego_loam_bl.txt - Transformation from base link to the base frame of LeGO-LOAM. By default it symlinks to base_link_to_os1.txt.
- ov_slam_bl.txt - Transformation from base link to the base frame of ObVi-SLAM. By default it is an identity transform.
- odom_bl.txt - Transformation from base link to the odometry frame. By default it is an identity transform too.

If you have different namings for those files, you can either softlink or change the calibration filenames inside the evaluation scripts later.

## Results from ROS bag sequence
TODO (Taijing, start here)
- Explain how to preprocess rosbag to get the data needed for minimal execution above

## Configuration File Guide
TODO 
- Explain how to modify configuration file -- which parameters will someone need to modify for different environment, (lower priority): explain each of the parameters in the config file

## Evaluation
Our YOLO model: TODO

## TODOs
- Add installation instructions
- Add offline execution instructions
- Add YOLO model

