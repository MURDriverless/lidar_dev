#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>

ros::Publisher pub;
double minX = 0;
double maxX = 0;
double minY = 0;
double maxY = 0;
double minZ = 0;
double maxZ = 0;

/**
 * cloud_cb
 * Main callback function that performs filtering on the input point cloud
 * message. Points near the robot/car chassis are filtered out.
 * @param cloud_msg input point cloud
 */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert given message into PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Rotate 180 deg and flip
	// ! when working in mursim the os1 lidar frame seems to be aligned with os1 sensor frame
	// ! if that's the case, then we dont' need to rotate and flip
	// ! update: it seems like this is a discrepancy on how the OS1 is simulated
	double min_X = minX;
	double max_X = maxX;
	double min_Y = minY;
	double max_Y = maxY;
	double min_Z = minZ;
	double max_Z = maxZ;

	// Perform crop box filtering
	pcl::CropBox<pcl::PCLPointCloud2> box;
	box.setMin(Eigen::Vector4f(min_X, min_Y, min_Z, 1.0));
	box.setMax(Eigen::Vector4f(max_X, max_Y, max_Z, 1.0));
	box.setInputCloud(cloudPtr);
	box.setNegative(true);
	box.filter(cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	// Publish the output data
	pub.publish(output);
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "carcrop");
	ros::NodeHandle nh;
	std::string input_topic = "os1_cloud_node/points";

	// Load up parameters
	nh.param("/carcrop/minX", minX, 0.0);
	nh.param("/carcrop/minY", minY, 0.0);
	nh.param("/carcrop/minZ", minZ, 0.0);
	nh.param("/carcrop/maxX", maxX, 0.0);
	nh.param("/carcrop/maxY", maxY, 0.0);
	nh.param("/carcrop/maxZ", maxZ, 0.0);
	nh.getParam("/carcrop/input_topic", input_topic);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("os1_cloud_node/points", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("carcrop_output", 1);

	// Spin
	ros::spin();
}