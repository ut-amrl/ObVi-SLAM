# ut_vslam

### Clone and Build
1. `git clone https://github.com/ut-amrl/ut_vslam.git` 
1. `cd ut_vslam`
1. `make`

### Run a Demo Program
Run the following command from the base directory. Note - make sure to modify the path to reflect BOTH the data set and the calibration. TODO hard code calibration path relative to dataset so user only enters one path.

    ./bin/main --data_path=`pwd`/data/vslam_set2/ --calibration_path=`pwd`/data/vslam_set2/calibration/camera_matrix.txt

This will run a program that reads in the poses and feature tracks into a UTSLAMProblem structure and then prints both to the terminal.