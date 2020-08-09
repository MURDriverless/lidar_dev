#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>



ros::Publisher pub;

// perform crop box filtering
void cloud_cropbox_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// convert given message into PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

    // parameters chosen for 2020-04-13-15-31-43.bag

	// ! Orientation 1
	// double minX = -3.5;
	// double maxX = 0.0;
	// double minY = -0.8;
	// double maxY = 0.8;
	// double minZ = -1.0;
	// double maxZ = 1.0;

	// ! Orientation 2
	// double minX = -0.8;
	// double maxX = 0.8;
	// double minY = 0;
	// double maxY = 3.5;
	// double minZ = -1.0;
	// double maxZ = 1.0;

	// ! Orientation 3
	double minX = -0.8;
	double maxX = 0.8;
	double minY = -3.5;
	double maxY = 0;
	double minZ = -1.0;
	double maxZ = 1.0;

	// perform crop box filtering
	pcl::CropBox<pcl::PCLPointCloud2> box;
	box.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	box.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	box.setInputCloud(cloudPtr);
	box.filter(cloud_filtered);

	// convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	// publish the output data
	pub.publish(output);
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "pcl_boxcrop");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cropbox_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("boxcrop_output", 1);

	// Spin
	ros::spin();
}