/*
 * lidarNode.cpp
 *
 * High level manager for the lidar node.
 * Will call a point cloud clustering class which performs clustering of the
 * obstacles and then perform classification using the lidar intensity image.
 */

#include "ros/ros.h"
#include "cluster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clusterNode");
    ros::NodeHandle nh("~");

    // prepare parameters
    ClusterParams params;
    nh.param("/cluster_node/cluster_tol", params.cluster_tol, params.cluster_tol);
    nh.param("/cluster_node/cluster_min", params.cluster_min, params.cluster_min);
    nh.param("/cluster_node/cluster_max", params.cluster_max, params.cluster_max);
    nh.param("/cluster_node/reconst_radius", params.reconst_radius, params.reconst_radius);
    nh.param("/cluster_node/marker_sx", params.marker_sx, params.marker_sx);
    nh.param("/cluster_node/marker_sy", params.marker_sy, params.marker_sy);
    nh.param("/cluster_node/marker_sz", params.marker_sz, params.marker_sz);
    nh.param("/cluster_node/marker_alpha", params.marker_alpha, params.marker_alpha);
    nh.param("/cluster_node/marker_r", params.marker_r, params.marker_r);
    nh.param("/cluster_node/marker_g", params.marker_g, params.marker_g);
    nh.param("/cluster_node/marker_b", params.marker_b, params.marker_b);
    nh.param("/cluster_node/lidar_hori_res", params.lidar_hori_res, params.lidar_hori_res);
    nh.param("/cluster_node/lidar_vert_res", params.lidar_vert_res, params.lidar_vert_res);
    nh.param("/cluster_node/filter_factor", params.filter_factor, params.filter_factor);
    nh.param("/cluster_node/magic_offset", params.magic_offset, params.magic_offset);
    nh.param("/cluster_node/clf_img_w", params.clf_img_w, params.clf_img_w);
    nh.param("/cluster_node/clf_img_h", params.clf_img_h, params.clf_img_h);
    nh.param("/cluster_node/clf_max_batch", params.clf_max_batch, params.clf_max_batch);
    nh.param("/cluster_node/clf_onnx", params.clf_onnx, params.clf_onnx);
    nh.param("/cluster_node/clf_trt", params.clf_trt, params.clf_trt);
    nh.param("/cluster_node/experiment_no", params.experiment_no, params.experiment_no);
    nh.param("/cluster_node/cone_colour", params.cone_colour, params.cone_colour);
    nh.param("/cluster_node/save_path", params.save_path, params.save_path);

    // start node
    ClusterDetector cluster_detector(nh, params);

    ros::spin();
}