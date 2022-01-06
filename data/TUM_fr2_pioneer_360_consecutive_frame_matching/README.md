# TUM freiburg2 pioneer 360 dataset
Extracted from freiburg2 pioneer 360 dataset (https://vision.in.tum.de/data/datasets/rgbd-dataset/download) using ORB\_SLAM2 (https://github.com/TieJean/ORB\_SLAM2). To generate other data from the TUM dataset, run:
`./Examples/RGB-D/mytest Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml Path_to_sequence_folder associate_file path_to_output_foler`
and then create a calibration/camera\_matrix.txt.

Frame 114 is missing, as less than 20 matches were found. Need to change vslam\_io.cpp to run this dataset.

## download dataset
`wget https://vision.in.tum.de/data/datasets/rgbd-dataset/download#`


