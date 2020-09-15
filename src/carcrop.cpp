#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud_msg, *input);

	// ! apply cylindrical filter on input cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cylin_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr cyl_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

	Eigen::Matrix3f cylinderMatrix;
	cylinderMatrix.setZero(3, 3);
	cylinderMatrix(0, 0) = 1.0;
	cylinderMatrix(1, 1) = 1.0;

	Eigen::Vector3f cylinderPosition;
	cylinderPosition << 0, 0, 0;

	double radius = 2;
	float cylinderScalar = -(radius * radius) + 1 + 1;

	pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr
		cyl_comp(new pcl::TfQuadraticXYZComparison<pcl::PointXYZ>(
			pcl::ComparisonOps::GE, cylinderMatrix, cylinderPosition, cylinderScalar));

	cyl_cond->addComparison(cyl_comp);
	pcl::PointCloud<pcl::PointXYZ>::Ptr recovered(new pcl::PointCloud<pcl::PointXYZ>);

	// build and apply filter
	condrem.setCondition(cyl_cond);
	condrem.setInputCloud(input);
	condrem.setKeepOrganized(false);
	condrem.filter(*cloud_filtered);

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
	pcl::CropBox<pcl::PointXYZ> box;
	box.setMin(Eigen::Vector4f(min_X, min_Y, min_Z, 1.0));
	box.setMax(Eigen::Vector4f(max_X, max_Y, max_Z, 1.0));
	box.setInputCloud(cloud_filtered);
	box.setNegative(true);
	box.filter(*cylin_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cylin_filtered, output);
	// pcl_conversions::fromPCL(cloud_filtered, output);

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