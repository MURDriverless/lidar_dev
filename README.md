# lidar_dev

This repository contains the LiDAR node used for perception development.

Currently, the project requires ROS and PCL to run.

## Bench Test Details

The current rosbag file has the LiDAR mounted at about 0.45m height.

Use the `2020-04-13-15-31-43.bag` available on the MUR Driverless sharepoint to playback on a loop. Note that the rosbag file was recorded using `lidar_mode=1024x10` which corresponds to 1024 horizontal resolution and 10 Hz.

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

So to set this up, go to the catkin workspace. For example `~/Document/catkin_ws`. It is also assumed that you have the full `ros-melodic-desktop-full` installed.

```
cd ~/Document/catkin_ws/src
git clone https://github.com/MURDriverless/lidar_dev
git clone https://github.com/lorenwel/linefit_ground_segmentation
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/glog_catkin.git
```

Also install the following if you do not have `glog-dev`.

```
sudo apt install libgoogle-glog-dev
```

## Config Changes

For indoor testing with `2020-04-13-15-31-43.bag` change the `sensor_height` to `0.45` in the `segmentation_params.yaml`.

The config file is located at

```
catkin_ws/src/linefit_ground_segmentation/linefit_ground_segmentation_ros/launch/segmentation_params.yaml
```

## References

References for TensorRT related implementation.
* https://github.com/cv-core/tensorrt_ros
* https://github.com/NVIDIA/TensorRT/tree/master/samples/opensource/sampleOnnxMNIST
* https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html

## Todo

- [x] Apply basic rotation to point cloud to correct for starting orientation.
- [x] Fix issue with cylindrical volume reconstruction
- [x] Debug horizontal offset problem in cropping lidar image, align lidar with traffic cone and see if caluclated `u, v` values are accurate
  - [x] Try to see if the horizontal offset is "constant" at different fix orientation as per Andrew Huang's suggestion.
  - [x] Implement temporary work around with `magic_offset`.
  - [x] Test if the offset works for `1024 x 64` mode
- [x] Implement lidar image saving feature, resizing all lidar image crops to `32 x 32`
- [ ] Refector current codes
- [ ] Test `carcrop_node` on indoor sensor data
- [ ] Make a fork for the official `ouster_example` for the purpose of custom `img_node`
- [ ] Update readme with data collection instructions.