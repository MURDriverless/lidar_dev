cmake_minimum_required(VERSION 2.8.3)
project(lidar_dev)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  mur_common
  ouster_ros
  cv_bridge
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ouster_ros_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

add_executable(segmentation_node src/segmentation.cpp)
target_link_libraries(segmentation_node ${catkin_LIBRARIES})

add_executable(boxcrop_node src/boxcrop.cpp)
target_link_libraries(boxcrop_node ${catkin_LIBRARIES})

add_executable(cluster_node src/cluster.cpp)
target_link_libraries(cluster_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(cluster_node ${catkin_EXPORTED_TARGETS})