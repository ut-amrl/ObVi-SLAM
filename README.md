# ObVi-SLAM
ObVi-SLAM is a joint object-visual SLAM approach aimed at long-term multi-session robot deployments. 

[[Paper with added appendix](https://arxiv.org/abs/2309.15268)] [[Video](https://youtu.be/quJOgnEdaZ0)]

Offline execution instructions coming soon. 
ROS implementation coming late 2023/early 2024. 

Please email amanda.adkins4242@gmail.com with any questions! 


## Evaluation
For information on how to set up and run the comparison algorithms, see our [evaluation repo](https://github.com/ut-amrl/ObVi-SLAM-Evaluation).

## Installation Instructions
TODO
- dockerfile version (recommended)
- native version

## Minimal Execution Instructions
TODO
- Explain files needed and their structure (intrinsics, extrinsics, visual features, bounding box (opt), images?,
- Explain how to run given these files


## Results from ROS bag sequence
TODO (Taijing, start here)
- Explain how to preprocess rosbag to get the data needed for minimal execution above

## Configuration File Guide
TODO 
- Explain how to modify configuration file -- which parameters will someone need to modify for different environment, (lower priority): explain each of the parameters in the config file

## Evaluation
For our experiments, we used [YOLOv5](https://github.com/ut-amrl/yolov5/tree/ROS) (based on [this repo](https://github.com/ultralytics/yolov5)) with [this model](https://drive.google.com/file/d/15xv-Se991Pzes7R3KfyPBkuSQ7TeCb1T/view?usp=sharing). 

We used detections with labels 'lamppost', 'treetrunk', 'bench', and 'trashcan' with [this configuration file](https://github.com/ut-amrl/ObVi-SLAM/blob/main/config/base7a_1_fallback_a_2.json). 

Please contact us if you would like to obtain the videos on which we performed the evaluation. 

## TODOs
- Add installation instructions
- Add offline execution instructions

