// process the /ground_segmentation/obstacle_cloud output from the segmentation node

#include "cluster.h"
#include <mur_common/cone_msg.h>

using PointOS1 = ouster_ros::OS1::PointOS1;

ros::Publisher pub;                 // publisher for all reconstructed cluster clouds
ros::Publisher markers_pub;         // publisher for cylinder markers
ros::Publisher results_pub;         // publisher for cone_msg
ros::Publisher intensity_image_pub; // publisher for intensity images

ClusterParams params;

ClusterDetector::ClusterDetector(std::string clf_onnx, std::string clf_trt)
{
    // set classifier parameters
    clfImgW = 32;
    clfImgH = 32;
    maxBatch = 50;

    lidarImgClassifier_.reset(
        new LidarImgClassifier(
            clf_onnx,
            clf_trt,
            clfImgW,
            clfImgH,
            maxBatch));
}

/**
 * num_expected_points 
 * computes the number of expected points for a traffic cone given its 3D
 * centre position.
 * @param centre centre coordinate (x, y, z) of the traffic cone
 * @return the number of expected points in point cloud
 * 
 * TODO: grab OS1 row/col scales from sensor instead of hard coding
 */
int ClusterDetector::num_expected_points(const pcl::PointXYZ &centre)
{
    // small cones  228 x 228 x 325 mm (base diag = 0.322)
    // big cones    285 x 285 x 505 mm
    double d = sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);
    static double hc = 0.31;                                 // cone height
    static double wc = 0.30;                                 // cone width
    static double rv = 2 * M_PI / 8 / params.lidar_vert_res; // angular resolution vertical
    static double rh = 2 * M_PI / params.lidar_hori_res;     // angular resolution horizontal

    // compute and return number of expected points
    double E = 0.5 * hc / (2 * d * tan(rv / 2)) * wc / (2 * d * tan(rh / 2));
    return (int)E;
}

/**
 * CloudToImage
 * converts a pointcloud cluster to a spherical projection intensity image.
 * @param cluster point cloud to be converted to an image
 * @return single channel intensity image
 * 
 * TODO: grab OS1 row/col scales from sensor instead of hard coding
 */
void CloudToImage(
    const pcl::PointCloud<PointOS1> &cluster,
    const std_msgs::Header &lidar_header,
    const cv_bridge::CvImagePtr &cv_ptr,
    const int &experiment_no,
    const int &frame_count,
    const int &cone_count,
    const std::string &cone_colour)
{
    int row_scale = params.lidar_vert_res; // lidar vertical resolution
    int col_scale = params.lidar_hori_res; // lidar horizontal resolution

    // TODO: grab fov from lidar config
    // using os1 gen1 specs
    // see https://ouster.com/products/os1-lidar-sensor/
    float fov_vert = 33.2 / 180.0 * M_PI; // 45 deg
    float fov_up = fov_vert / 2;
    float fov_down = fov_vert / 2;

    float yaw_offset = M_PI; // add offset to work in os1_sensor frame

    int u_min = INT_MAX;
    int u_max = INT_MIN;
    int v_min = INT_MAX;
    int v_max = INT_MIN;

    // loop through each point in the cloud
    for (int i = 0; i < cluster.size(); ++i)
    {

        // ? try rotate cloud by 180
        float x = -cluster[i].x;
        float y = -cluster[i].y;
        float z = cluster[i].z;

        float r = sqrt(x * x + y * y + z * z);
        float yaw = atan2(y, x);
        float pitch = asin(z / r);

        // image coordinate rows
        int u = row_scale * (1 - (pitch + fov_down) / fov_vert);
        // image coordinate columns
        int v = col_scale * 0.5 * ((yaw / M_PI) + 1);

        if (u < u_min)
        {
            u_min = u;
        }
        if (u > u_max)
        {
            u_max = u;
        }
        if (v < v_min)
        {
            v_min = v;
        }
        if (v > v_max)
        {
            v_max = v;
        }
    }

    // DEBUG print
    // std::cout << "Printing umin, umax, vmin, vmax: " << std::endl;
    // std::cout << u_min << " " << u_max << " " << col_scale - v_min << " " << col_scale - v_max << std::endl;

    // TODO: fix magic offset static (credits to Andrew Huang)
    int magic_offset = params.magic_offset;
    int left = col_scale - v_max - magic_offset;
    int right = col_scale - v_min - magic_offset;
    int top = u_min;
    int bot = u_max;

    // expand bounding box to capture extra area
    float expand_factor = 0.1;
    int width = right - left;
    int height = bot - top;
    int w_expand = width * expand_factor;
    int h_expand = height * expand_factor;
    left = std::max(left - w_expand, 0);
    right = std::min(right + w_expand, col_scale);
    top = std::max(top - h_expand, 0);
    bot = std::min(bot + h_expand, row_scale);

    cv::Rect box(cv::Point(left, top), cv::Point(right, bot));
    cv::Mat roi = cv_ptr->image(box);

    // draw box on full image
    cv::rectangle(cv_ptr->image, cv::Point(left, top), cv::Point(right, bot), cv::Scalar(0, 255, 0));

    // prepare image saving path
    std::stringstream ss;
    ss << "lidar_";
    ss << std::setw(2) << std::setfill('0') << experiment_no << "_";
    ss << std::setw(3) << std::setfill('0') << frame_count << "_";
    ss << cone_count << "_";
    ss << cone_colour;
    std::string img_path = params.save_path + ss.str() + ".jpg";
    ROS_INFO("image path = %s\n", img_path.data());

    // resize image
    cv::Mat resized;
    cv::resize(roi, resized, cv::Size(32, 32));

    // cv::imshow("resized image", resized);
    // cv::waitKey(30);

    // supprot for writing JPG
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    // ! will only work if directory exists
    // TODO: update code to create directory if not exist
    bool result = cv::imwrite(img_path, resized, compression_params);
    std::cout << "imwrite result = " << result << std::endl;
}

// perform euclidean clustering
void ClusterDetector::cloud_cluster_cb(
    const sensor_msgs::PointCloud2ConstPtr &obstacles_msg,
    const sensor_msgs::PointCloud2ConstPtr &ground_msg,
    const sensor_msgs::ImageConstPtr &intensity_msg)
{
    // time callback run time as performance measure
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    pcl::PointCloud<PointOS1>::Ptr input(new pcl::PointCloud<PointOS1>);
    pcl::PointCloud<PointOS1>::Ptr input_ground(new pcl::PointCloud<PointOS1>);

    pcl::fromROSMsg(*obstacles_msg, *input);
    pcl::fromROSMsg(*ground_msg, *input_ground);

    // Use a conditional filter to remove points at the origin (0, 0, 0)
    pcl::ConditionOr<PointOS1>::Ptr range_cond(new pcl::ConditionOr<PointOS1>());

    range_cond->addComparison(pcl::FieldComparison<PointOS1>::ConstPtr(
        new pcl::FieldComparison<PointOS1>("x", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<PointOS1>::ConstPtr(
        new pcl::FieldComparison<PointOS1>("y", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<PointOS1>::ConstPtr(
        new pcl::FieldComparison<PointOS1>("z", pcl::ComparisonOps::GT, 0.0)));

    range_cond->addComparison(pcl::FieldComparison<PointOS1>::ConstPtr(
        new pcl::FieldComparison<PointOS1>("x", pcl::ComparisonOps::LT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<PointOS1>::ConstPtr(
        new pcl::FieldComparison<PointOS1>("y", pcl::ComparisonOps::LT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<PointOS1>::ConstPtr(
        new pcl::FieldComparison<PointOS1>("z", pcl::ComparisonOps::LT, 0.0)));

    // build the filter
    pcl::ConditionalRemoval<PointOS1> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(input);
    condrem.setKeepOrganized(false);

    // apply filter
    pcl::PointCloud<PointOS1>::Ptr cloud_filtered(new pcl::PointCloud<PointOS1>);
    condrem.filter(*cloud_filtered);
    std::cout << "PointCloud after conditional filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // create KdTree object
    pcl::search::KdTree<PointOS1>::Ptr tree(new pcl::search::KdTree<PointOS1>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointOS1> ec;
    ec.setClusterTolerance(params.cluster_tol); // 8cm (affects resulting cluster size)
    ec.setMinClusterSize(params.cluster_min);   // minimum number of points
    ec.setMaxClusterSize(params.cluster_max);   // maximum number of points
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pcl::PointCloud<PointOS1>::Ptr clustered_cloud(new pcl::PointCloud<PointOS1>);

    // vector to store marker points
    std::vector<pcl::PointXYZ> marker_points;

    std::cout << "Num of clusters" << cluster_indices.size() << std::endl;

    // copy intensity image here, so that multiple bbox can be drawn
    cv_bridge::CvImagePtr intensity_cv_ptr;
    static int frame_count = 0;
    int cone_count = 0;

    try
    {
        intensity_cv_ptr = cv_bridge::toCvCopy(intensity_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", intensity_msg->encoding.c_str());
    }

    // outer loop goes through all the clusters we found
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<PointOS1>::Ptr cloud_cluster(new pcl::PointCloud<PointOS1>);

        for (const int &index : it->indices)
            cloud_cluster->points.push_back(cloud_filtered->points[index]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // extract centroid of cluster
        pcl::PointXYZ centre;
        pcl::computeCentroid(*cloud_cluster, centre);

        // compute distance to cluster, before cylindrical reconsturction
        double d = sqrt(centre.x * centre.x + centre.y * centre.y + centre.z * centre.z);

        // TODO: cylindrical reconsturction BEFORE rule based filter
        // perform cylindrical reconstruction from ground points
        pcl::ConditionAnd<PointOS1>::Ptr cyl_cond(new pcl::ConditionAnd<PointOS1>());

        Eigen::Matrix3f cylinderMatrix;
        cylinderMatrix(0, 0) = 1.0;
        cylinderMatrix(1, 1) = 1.0;

        Eigen::Vector3f cylinderPosition;
        cylinderPosition << -centre.x, -centre.y, 0;

        double radius = params.reconst_radius;
        float cylinderScalar = -(radius * radius) + centre.x * centre.x + centre.y * centre.y;

        pcl::TfQuadraticXYZComparison<PointOS1>::Ptr cyl_comp(new pcl::TfQuadraticXYZComparison<PointOS1>(pcl::ComparisonOps::LE, cylinderMatrix, cylinderPosition, cylinderScalar));

        cyl_cond->addComparison(cyl_comp);
        pcl::PointCloud<PointOS1>::Ptr recovered(new pcl::PointCloud<PointOS1>);

        // build and apply filter
        condrem.setCondition(cyl_cond);
        condrem.setInputCloud(input_ground);
        condrem.setKeepOrganized(false);
        condrem.filter(*recovered); // ? Main issue lies in the data that is recovered ?

        *cloud_cluster += *recovered;

        // apply rule based filter
        double filter_factor = params.filter_factor; // used for tuning
        std::cout << "[potential] NumExpectedPoints = " << filter_factor * num_expected_points(centre) << std::endl;
        std::cout << "[potential] num actual points = " << cloud_cluster->size() << std::endl;
        std::cout << "[potential] distance to cone  = " << d << std::endl;

        // skip processing step if there is insufficient points
        int expected = num_expected_points(centre);
        if (cloud_cluster->size() < filter_factor * expected)
        {
            continue;
        }

        std::cout << "[confirmed] Printing x, y, r   " << centre.x << " " << centre.y << " " << radius << std::endl;

        std::cout << "[confirmed] NumExpectedPoints = " << filter_factor * num_expected_points(centre) << std::endl;
        std::cout << "[confirmed] num actual points = " << cloud_cluster->size() << std::endl;
        std::cout << "[confirmed] distance to cone  = " << d << std::endl;

        // add to marker points
        marker_points.push_back(centre);

        // ! collect data for lidar image classification
        CloudToImage(*cloud_cluster, obstacles_msg->header, intensity_cv_ptr,
                     params.experiment_no, frame_count, cone_count, params.cone_colour);
        cone_count++;

        // join each cloud cluster into one combined cluster (visualisation)
        *clustered_cloud = *recovered;

        // DEBUG: print info about cluster size
        // std::cout << " PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        // std::cout << " PointCloud representing the Combined Cluster: " << clustered_cloud->points.size() << " data points." << std::endl;

        // break;
    }

    frame_count++;

    // prepare marker array
    visualization_msgs::MarkerArray marker_array_msg;
    marker_array_msg.markers.resize(marker_points.size());
    for (int i = 0; i < marker_points.size(); ++i)
    {
        set_marker_properties(&marker_array_msg.markers[i], marker_points[i], i, input->header.frame_id);
    }

    std::cout << "NUM OF MARKERS = " << marker_points.size() << std::endl;

    // prepare results msg (in cone_msg format)
    mur_common::cone_msg cone_msg;
    for (int i = 0; i < marker_points.size(); ++i)
    {
        cone_msg.x.push_back(marker_points[i].x);
        cone_msg.y.push_back(marker_points[i].y);
        cone_msg.colour.push_back("na");
    }
    cone_msg.frame_id = input->header.frame_id;

    // set additional header info and convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*clustered_cloud, output);
    output.header.frame_id = input->header.frame_id;
    output.header.stamp = ros::Time::now();

    ROS_INFO("About to publish cluster output \n");

    // publish the output data
    pub.publish(output);
    markers_pub.publish(marker_array_msg);
    results_pub.publish(cone_msg);

    // measure and print runtime performance
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);

    // ! display intensity image
    cv::imshow("view", intensity_cv_ptr->image);
    cv::waitKey(30);
}

// function to set the marker properties
void ClusterDetector::set_marker_properties(
    visualization_msgs::Marker *marker,
    pcl::PointXYZ centre,
    int n,
    std::string frame_id)
{
    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time();
    marker->ns = "my_namespace";
    marker->id = n;
    marker->type = visualization_msgs::Marker::CYLINDER;
    marker->action = visualization_msgs::Marker::ADD;

    marker->pose.position.x = centre.x;
    marker->pose.position.y = centre.y;
    marker->pose.position.z = centre.z;

    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    marker->scale.x = params.marker_sx;
    marker->scale.y = params.marker_sy;
    marker->scale.z = params.marker_sz;

    // alpha and RGB settings
    marker->color.a = params.marker_alpha;
    marker->color.r = params.marker_r;
    marker->color.g = params.marker_g;
    marker->color.b = params.marker_b;

    marker->lifetime = ros::Duration(0.5);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "cluster_node");
    ros::NodeHandle nh;

    // Parse parameters
    nh.param("/cluster/cluster_tol", params.cluster_tol, params.cluster_tol);
    nh.param("/cluster/cluster_min", params.cluster_min, params.cluster_min);
    nh.param("/cluster/cluster_max", params.cluster_max, params.cluster_max);
    nh.param("/cluster/reconst_radius", params.reconst_radius, params.reconst_radius);
    nh.param("/cluster/marker_sx", params.marker_sx, params.marker_sx);
    nh.param("/cluster/marker_sy", params.marker_sy, params.marker_sy);
    nh.param("/cluster/marker_sz", params.marker_sz, params.marker_sz);
    nh.param("/cluster/marker_alpha", params.marker_alpha, params.marker_alpha);
    nh.param("/cluster/marker_r", params.marker_r, params.marker_r);
    nh.param("/cluster/marker_g", params.marker_g, params.marker_g);
    nh.param("/cluster/marker_b", params.marker_b, params.marker_b);
    nh.param("/cluster/lidar_hori_res", params.lidar_hori_res, params.lidar_hori_res);
    nh.param("/cluster/lidar_vert_res", params.lidar_vert_res, params.lidar_vert_res);
    nh.param("/cluster/filter_factor", params.filter_factor, params.filter_factor);
    nh.param("/cluster/magic_offset", params.magic_offset, params.magic_offset);
    nh.param("/cluster/experiment_no", params.experiment_no, params.experiment_no);
    nh.param("/cluster/cone_colour", params.cone_colour, params.cone_colour);
    nh.param("/cluster/save_path", params.save_path, params.save_path);

    // ! Initialise ClusterDetector
    const std::string clf_onnx = ros::package::getPath("lidar_dev") + "/models/lidar_cone_classifier.onnx";
    const std::string clf_trt = ros::package::getPath("lidar_dev") + "/models/lidar_cone_classifier.trt";
    ClusterDetector clusterDetector(clf_onnx, clf_trt);

    // Create a ROS subscriber for ground plane and potential obstacles
    message_filters::Subscriber<sensor_msgs::PointCloud2> ground_sub(nh, "ground_segmentation/obstacle_cloud", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> obstacles_sub(nh, "ground_segmentation/ground_cloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> intensity_sub(nh, "img_node/intensity_image", 1);

    // ! Approximate time sync policy
    // TODO: Sync properly with new non-Ouster driver
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image> mySyncPolicy;

    message_filters::Synchronizer<mySyncPolicy>
        sync(mySyncPolicy(10), ground_sub, obstacles_sub, intensity_sub);

    sync.registerCallback(boost::bind(&ClusterDetector::cloud_cluster_cb, &clusterDetector, _1, _2, _3));

    // ! exact time sync policy
    // Pass both subscribed message into the same callback
    // message_filters::TimeSynchronizer
    //     <sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image>
    //     sync(ground_sub, obstacles_sub, intensity_sub, 10);
    // sync.registerCallback(boost::bind(&cloud_cluster_cb, _1, _2, _3));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("cluster_output", 1);
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);
    results_pub = nh.advertise<mur_common::cone_msg>("cone_messages", 1);
    intensity_image_pub = nh.advertise<sensor_msgs::Image>("lidar_crop_image", 1);

    // Spin
    ros::Duration(0.5).sleep();
    ros::spin();
}