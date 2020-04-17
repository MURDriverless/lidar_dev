# lidar_dev

This repository contains the LiDAR node used for perception development.

Currently, the project requires ROS and PCL to run.

## Bench Test Details

The current rosbag file has the LiDAR mounted at about 0.45m height.

## Dependencies

Currently the ground segmentation part relies on the following package, and its dependencies.

```
https://github.com/lorenwel/linefit_ground_segmentation
https://github.com/catkin/catkin_simple.git
https://github.com/ethz-asl/glog_catkin.git
```

