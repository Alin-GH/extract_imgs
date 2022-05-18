# extract and save images from rosbag 

## Introduction
rosbags contain different topics 

for images : /dev/video0/compressed      : sensor_msgs/CompressedImage   
             /dev/video1/compressed      : sensor_msgs/CompressedImage   
             /dev/video2/compressed      : sensor_msgs/CompressedImage   
             /dev/video3/compressed      : sensor_msgs/CompressedImage   
             /dev/video4/compressed      : sensor_msgs/CompressedImage   
             /dev/video5/compressed      : sensor_msgs/CompressedImage   
             /dev/video6/compressed      : sensor_msgs/CompressedImage   
             /dev/video7/compressed      : sensor_msgs/CompressedImage

## Requirements
All codes are tested under the following environment:
*   Ubuntu
*   Python 3.7
*   roabag 
*   rospy
*   numpy



## run 
1. use `conda` to manage the environment:
```
conda create -n pcd python=3.7
```

2. activate environment:
```
conda activate pcd
```

3. extract infomations:

```
./toe.sh
```


## result-like 

       FoldName             Description
----------------------------------------------------------------------------
   1    camera_f30         images for front FOV30  
   
   2    camera_f60         images for front FOV60  

   3    camera_f120        images for front FOV120  

   3    camera_fd120       images for down-front FOV120



