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

// perform voxel grid filtering
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize(0.1, 0.1, 0.1);
	sor.filter(cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	// Publish the data
	pub.publish(output);
}

// perform passthrough filtering
void cloud_passthrough_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// convert given message into PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// perform passthrough filtering
	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud(cloudPtr);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.0, 0);
	pass.setFilterLimitsNegative(false);
	pass.filter(cloud_filtered);

	// convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	// publish the output data
	pub.publish(output);
}

// perform crop box filtering
void cloud_cropbox_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// convert given message into PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	double minX = -4.0;
	double maxX = 0.0;

	double minY = -1.0;
	double maxY = 0.9;

	double minZ = -1.0;
	double maxZ = 1.0;

	// perform crop box filtering
	pcl::CropBox<pcl::PCLPointCloud2> box;
	box.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	box.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	box.setInputCloud(cloudPtr);
	box.filter(cloud_filtered);

	// skip voxel grid downsample and jump directly to segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(2000);
	seg.setDistanceThreshold(0.3);

	// convert PCLPointCloud2 to PointXYZ for segmentation
	pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(cloud_filtered, *cropped_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr RansacFiltered(new pcl::PointCloud<pcl::PointXYZ>());

	seg.setInputCloud(cropped_cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0){
		ROS_INFO("Could not estimate a planar model for given point cloud");
	}

	// create filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cropped_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*RansacFiltered);

	// convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*RansacFiltered, output);

	// print plane coefficient
	// note that the plane equation is of the form ax + by + cz + d = 0
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	ROS_INFO("[Plane Equation] a=%f, b=%f, c=%f, d=%f\n", a, b, c, d);

	pub.publish(output);
}

void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr &input)
{
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input, cloud);

	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud.makeShared());
	seg.segment(inliers, coefficients);

	// Publish the model coefficients
	pcl_msgs::ModelCoefficients ros_coefficients;
	pcl_conversions::fromPCL(coefficients, ros_coefficients);
	pub.publish(ros_coefficients);
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "pcl_segmentation");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cropbox_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2>("seg_output", 1);
	// pub = nh.advertise<pcl_msgs::ModelCoefficients>("seg_output", 1);

	// Spin
	ros::spin();
}