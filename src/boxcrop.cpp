#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <mur_common/timing_msg.h>

#define POINT_CLOUD_TOPIC "os1_cloud_node/points"
#define CARCROP_TOPIC "/mur/lidar/boxcrop_output"
#define HEALTH_TOPIC "/mur/lidar/boxcrop_health"

ros::Publisher pub;
ros::Publisher health_pub;

void push_health(uint64_t logic_start, uint64_t lidar_start)
{
	mur_common::timing_msg h;

	uint64_t wall_current = ros::WallTime::now().toNSec();
	uint64_t current = ros::Time::now().toNSec();

	float logic_time = (wall_current - logic_start) * 1e-6;
	float lidar_time = (current - lidar_start) * 1e-6;

	h.compute_time = logic_time;
	h.full_compute_time = lidar_time;
	h.header.stamp = ros::Time::now();

	health_pub.publish(h);
}

// perform crop box filtering
// this is mainly used for indoor testing
void cloud_cropbox_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	ros::WallTime start = ros::WallTime::now();
	
	// Container for original & filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert given message into PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Parameters chosen for 2020-04-13-15-31-43.bag

	// ! Orientation 1 (cone in front of lidar)
	double minX = -3.5;
	double maxX = 0.0;
	double minY = -0.8;
	double maxY = 0.8;
	double minZ = -1.0;
	double maxZ = 1.0;

	// ! Orientation 2 (cone right of lidar)
	// double minX = -0.8;
	// double maxX = 0.8;
	// double minY = 0;
	// double maxY = 3.5;
	// double minZ = -1.0;
	// double maxZ = 1.0;

	// ! Orientation 3 (cone left of lidar)
	// double minX = -0.8;
	// double maxX = 0.8;
	// double minY = -3.5;
	// double maxY = 0;
	// double minZ = -1.0;
	// double maxZ = 1.0;

	// Perform crop box filtering
	pcl::CropBox<pcl::PCLPointCloud2> box;
	box.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	box.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	box.setInputCloud(cloudPtr);
	box.filter(cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	// Publish the output data
	pub.publish(output);

	// Publish timing info
	push_health(start.toNSec(), cloud_msg->header.stamp.toNSec());
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "pcl_boxcrop");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe(POINT_CLOUD_TOPIC, 1, cloud_cropbox_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>(CARCROP_TOPIC, 1);
	health_pub = nh.advertise<mur_common::timing_msg>(HEALTH_TOPIC, 1);

	// Spin
	ros::spin();
}