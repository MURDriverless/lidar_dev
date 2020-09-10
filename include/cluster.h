#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>

#include "classifier.h"

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
                      clf_img_w(32),
                      clf_img_h(32),
                      clf_max_batch(50),
                      clf_onnx("/model/lidar_cone_classifier.onnx"),
                      clf_trt("/model/lidar_cone_classifier.trt"),
                      experiment_no(0),
                      cone_colour("b"),
                      save_path("/tmp/lidar_imgs/") {}


    double cluster_tol;             // cluster tolerance
    int cluster_min;                // cluster minimum number of points
    int cluster_max;                // cluster maximum number of points
    double reconst_radius;          // radius of cluster reconstruction

    double marker_sx;               // marker scale x
    double marker_sy;               // marker scale y
    double marker_sz;               // marker scale z
    double marker_alpha;            // marker alpha
    double marker_r;                // marker reg
    double marker_g;                // marker green
    double marker_b;                // marker blue

    int lidar_hori_res;             // lidar horizontal resolution
    int lidar_vert_res;             // lidar vertical resolution
    double filter_factor;           // filter factor
    int magic_offset;               // temp solution for lidar bbox offset

    int clf_img_w;                  // classifier image width
    int clf_img_h;                  // classifier image height
    int clf_max_batch;              // classifier input max batch
    std::string clf_onnx;           // classifier onnx model path
    std::string clf_trt;            // classifier trt model path

    int experiment_no;              // lidar data collection experiment count
    std::string cone_colour;        // traffic cone type used
    std::string save_path;          // save path for lidar image crops
};

class ClusterDetector
{
public:
    ClusterDetector(ros::NodeHandle n, ClusterParams p);
    void cloud_cluster_cb(
        const sensor_msgs::PointCloud2ConstPtr &obstacles_msg,
        const sensor_msgs::PointCloud2ConstPtr &ground_msg,
        const sensor_msgs::ImageConstPtr &intensity_msg);

private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> ground_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> obstacles_sub;
    message_filters::Subscriber<sensor_msgs::Image> intensity_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image> mySyncPolicy;
    typedef message_filters::Synchronizer<mySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Publisher cluster_pub;
    ros::Publisher markers_pub;
    ros::Publisher results_pub;
    ros::Publisher intensity_image_pub;

    ClusterParams params;

    cv::Rect cloud_to_bbox(const pcl::PointCloud<pcl::PointXYZ> &cluster);
    cv::Mat cloud_to_img(
        const pcl::PointCloud<pcl::PointXYZ> &cluster,
        const cv_bridge::CvImagePtr &cv_ptr);

    int num_expected_points(const pcl::PointXYZ &centre);
    void set_marker_properties(
        visualization_msgs::Marker *marker,
        pcl::PointXYZ centre,
        int n,
        int cone_type,
        std::string frame_id);
    void CloudToImage(
        const pcl::PointCloud<pcl::PointXYZ> &cluster,
        const std_msgs::Header &lidar_header,
        const cv_bridge::CvImagePtr &cv_ptr,
        const int &experiment_no,
        const int &frame_count,
        const int &cone_count,
        const std::string &cone_colour);
    std::unique_ptr<LidarImgClassifier> lidarImgClassifier_;
};

#endif // CLUSTER_H_