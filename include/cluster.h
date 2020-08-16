#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <limits>

// PCL specific includes
#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>

#include "ouster_ros/point_os1.h"
#include "lidarImgClassifier.h"

struct ClusterParams
{
    ClusterParams() : cluster_tol(0.08),
                      cluster_min(10),
                      cluster_max(250),
                      reconst_radius(0.6),
                      marker_sx(0.35),
                      marker_sy(0.35),
                      marker_sz(0.7),
                      marker_alpha(0.5),
                      marker_r(0.0),
                      marker_g(1.0),
                      marker_b(0.0),
                      lidar_hori_res(1024),
                      lidar_vert_res(64),
                      filter_factor(1.0),
                      magic_offset(9),
                      experiment_no(0),
                      cone_colour("b"),
                      save_path("/tmp/lidar_imgs/") {}

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

    // lidar horizontal resolution
    int lidar_hori_res;
    // lidar vertical resolution
    int lidar_vert_res;

    // filter factor
    double filter_factor;
    // temporary solution for fixing lidar bounding box offset
    int magic_offset;

    // lidar data collection experiment count
    int experiment_no;
    // traffic cone type used
    std::string cone_colour;
    // save path for lidar image crops
    std::string save_path;
};

class ClusterDetector
{
public:
    ClusterDetector(std::string clf_onnx, std::string clf_trt);
    void cloud_cluster_cb(
        const sensor_msgs::PointCloud2ConstPtr &obstacles_msg,
        const sensor_msgs::PointCloud2ConstPtr &ground_msg,
        const sensor_msgs::ImageConstPtr &intensity_msg);

private:
    int num_expected_points(const pcl::PointXYZ &centre);
    void set_marker_properties(
        visualization_msgs::Marker *marker,
        pcl::PointXYZ centre,
        int n,
        std::string frame_id);

    std::unique_ptr<LidarImgClassifier> lidarImgClassifier_;
    int clfImgW;
    int clfImgH;
    int maxBatch;
};

#endif // CLUSTER_H_