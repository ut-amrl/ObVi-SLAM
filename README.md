# ut_vslam

### Prerequisites
1. Ubuntu packages:
    ```
    sudo apt install libglew-dev libgl1-mesa-dev
    ```
1. Build and install Pangolin v0.6: https://github.com/stevenlovegrove/Pangolin/tree/v0.6
    ```
    cd [WHEREVER YOU WANT TO CLONE PANGOLIN SRC]
    git clone --recursive --recurse-submodules git@github.com:stevenlovegrove/Pangolin.git
    git checkout v0.6
    mkdir build
    cd build
    cmake ..
    make -j`nproc`
    sudo make install
    ```

### Clone and Build
1. `git clone https://github.com/ut-amrl/ut_vslam.git` 
1. `cd ut_vslam`
1. `make`

### Run a Demo Program
Run the following command from the base directory. Note - make sure to modify the path to reflect BOTH the data set and the calibration. TODO hard code calibration path relative to dataset so user only enters one path.

    ./bin/main --dataset_path=`pwd`/data/vslam_set6/ --output_path=`pwd`/output/ --save_poses true

This will run a program that reads in the poses and feature tracks into a UTSLAMProblem structure and then prints both to the terminal.
