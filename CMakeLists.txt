cmake_minimum_required(VERSION 2.8.3)
project(lidar_dev)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-unused-variable")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

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
  cv_bridge
)

catkin_package()

#-------------------------------------------------------------------------------
# CUDA
#-------------------------------------------------------------------------------
# find_package(CUDA 10.0 REQUIRED)
# set(CUDA_VERBOSE_BUILD ON)
# set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})

# find_package(CUDNN REQUIRED)
# include_directories(${CUDNN_INCLUDE_DIR})

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
# find_path(TENSORRT_INCLUDE_DIR NvInfer.h
#   HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES include)
# MESSAGE(STATUS "Found TensorRT headers at ${TENSORRT_INCLUDE_DIR}")

# find_library(TENSORRT_LIBRARY_INFER nvinfer
#   HINTS ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES lib lib64 lib/x64)

# find_library(TENSORRT_LIBRARY_INFER_PLUGIN nvinfer_plugin
#   HINTS  ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES lib lib64 lib/x64)

# find_library(TENSORRT_LIBRARY_PARSER nvparsers
#   HINTS  ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES lib lib64 lib/x64)

# set(TENSORRT_LIBRARY ${TENSORRT_LIBRARY_INFER} ${TENSORRT_LIBRARY_INFER_PLUGIN} ${TENSORRT_LIBRARY_PARSER})

# MESSAGE(STATUS "Find TensorRT libs at ${TENSORRT_LIBRARY}")
# find_package_handle_standard_args(
#   TENSORRT DEFAULT_MSG TENSORRT_INCLUDE_DIR TENSORRT_LIBRARY)

# if(NOT TENSORRT_FOUND)
#   message(ERROR
#     "Cannot find TensorRT library.")
# endif()


#-------------------------------------------------------------------------------
# Build Libraries
#-------------------------------------------------------------------------------
# add_library(classifier src/lidarImgClassifier.cpp src/Utils.cpp)
# target_link_libraries(classifier ${OPENCV_LIBS} ${catkin_LIBRARIES} nvonnxparser)

#-------------------------------------------------------------------------------
# Executables
#-------------------------------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
)

add_executable(segmentation_node src/segmentation.cpp)
target_link_libraries(segmentation_node ${catkin_LIBRARIES})

add_executable(boxcrop_node src/boxcrop.cpp)
target_link_libraries(boxcrop_node ${catkin_LIBRARIES})

add_executable(carcrop_node src/carcrop.cpp)
target_link_libraries(carcrop_node ${catkin_LIBRARIES})

add_executable(lidar_tf_node src/transform_pc.cpp)
target_link_libraries(lidar_tf_node ${catkin_LIBRARIES})

add_executable(conepose_node src/conepose.cpp)
target_link_libraries(conepose_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(p3d_tf_node src/p3d_tf.cpp)
target_link_libraries(p3d_tf_node ${catkin_LIBRARIES})

add_executable(cluster_node src/cluster.cpp)
target_link_libraries(cluster_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(cluster_node classifier ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(cluster_node ${catkin_EXPORTED_TARGETS})