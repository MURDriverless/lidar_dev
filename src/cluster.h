#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>

#define BLUE_STR "BLUE"
#define ORANGE_STR "ORANGE"
#define YELLOW_STR "YELLOW"
#define UNKNOWN_STR "na"

struct ClusterParams {
    ClusterParams():
        cluster_tol(0.08),
        cluster_min(10),
        cluster_max(250),
        reconst_radius(0.6),
        marker_sx(0.35),
        marker_sy(0.35),
        marker_sz(0.7),
        marker_alpha(0.5),
        marker_r(0.0),
        marker_g(1.0),
        marker_b(0.0) {}
    
    // cluster tolerance 
    double cluster_tol;
    // cluster minimum number of points
    int cluster_min;
    // cluster maximum number of points
    int cluster_max;
    // radius of cluster reconstruction
    double reconst_radius;

    // marker scale x
    double marker_sx;
    // marker scale y
    double marker_sy;
    // marker scale z
    double marker_sz;
    // marker alpha
    double marker_alpha;
    // marker reg
    double marker_r;
    // marker green
    double marker_g;
    // marker blue
    double marker_b;
};

void set_marker_properties(visualization_msgs::Marker *marker, pcl::PointXYZ centre, int n, std::string frame_id);
void cloud_cluster_cb(const sensor_msgs::PointCloud2ConstPtr &obstacles_msg, const sensor_msgs::PointCloud2ConstPtr &ground_msg);
int num_expected_points(const pcl::PointXYZ &centre);

#endif // CLUSTER_H_