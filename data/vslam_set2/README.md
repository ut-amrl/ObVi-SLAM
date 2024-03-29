# VSLAM SET2 Sequence
The sequence contains simulated date acquired in matlab. Ground truth poses, measurements, and 3D features are available. The feature point cloud contained 100 randomly generated points drawn from a uniform distribution. Not all points are imaged. 

In the sequence the camera moves from (-20, 0 , 0) to (-10, 0, 0) in a straight line with a series of 0.5m steps.

 ## Frame Coordinates
Coordinates given as (x, y, z, q<sub>x</sub>, q<sub>y</sub>, q<sub>z</sub>, q<sub>w</sub>) where q<sub>i</sub> is quaternion representation of rotation. Units are in meters.

## Label Format
ID | x | y | 
--- | --- | --- | 
1 | 400 | 344 |
2 | 200 | 566 |
4 | 650 | 668 |
k | x<sub>k</sub> | y<sub>k</sub> |

The keypoints for each frame are stored in a text file. There are as many rows as there are keypoints. Each keypoint has a integer ID that is consistent across all frames and corresponds to a 3D feature ID. The coordinates are given in pixel coordinates.

## Data File
The first entry in the data file is the frame ID, the second is the map frame pose, and the third is the table of labeled keypoints.

## Calibration 
The images are 480 x 640 and their camera matrix is given by;  
K = [400, 0.0, 320, 0.0, 400, 240, 0.0, 0.0, 1.0]  


The camera_matrix.txt files stores the camera calibration matrix in the from "fx fy cx cy".
