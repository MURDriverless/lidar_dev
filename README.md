# lidar_dev

This repository contains the LiDAR node used for perception development.

Currently, the project requires ROS and PCL to run.

## Bench Test Details

The current rosbag file has the LiDAR mounted at about 0.45m height.

Use the `2020-04-13-15-31-43.bag` available on the MUR Driverless sharepoint to playback on a loop.

```
rosbag play -l 2020-04-13-15-31-43.bag
```

Then call the launch file as follows.

```
roslaunch lidar_dev cluster_pipeline.launch
```

## Dependencies

Currently the ground segmentation part relies on the following package, and its dependencies.

```
https://github.com/lorenwel/linefit_ground_segmentation
https://github.com/catkin/catkin_simple.git
https://github.com/ethz-asl/glog_catkin.git
```

