import cv2
import numpy as np
import os

n = 2489
datadir = "/root/LTOV-SLAM-Evaluation/data/cube_slam_in/_2023_05_11_18_35_54/1/visualization"
savedir = "/root/LTOV-SLAM-Evaluation/data/cube_slam_in/_2023_05_11_18_35_54/1/video"

kStride = 9
kVideoName = "video.avi"


if not os.path.exists(savedir):
    os.mkdir(savedir)
savepath = os.path.join(savedir, kVideoName)

img_array = []
for fileid in range(0, n, kStride):
    filepath = os.path.join(datadir, str(fileid)+".png")
    if not os.path.exists(filepath):
        print("Failed to open file ", filepath)
        continue
    img = cv2.imread(filepath)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)

print("savepath: ", savepath)
out = cv2.VideoWriter(savepath,cv2.VideoWriter_fourcc(*'DIVX'), 10, size)
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()

