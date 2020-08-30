#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// ros::Publisher pub;
// tf2_ros::Buffer tf_buffer;
// tf2_ros::TransformListener tf_listener;

class TransformPointCloud
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        geometry_msgs::TransformStamped transform;
        try
        {
            // (target_frame, source_frame) preserves world coordinates but
            // changes parent to target_frame
            //
            // (source_frame, target_frame) shifts point cloud to target_frame
            // but frame parent does not change
            //
            const std::string target_frame = "frame1";
            const std::string source_frame = msg->header.frame_id;
            transform = tf_buffer_.lookupTransform(
                target_frame,
                source_frame,
                ros::Time(0));

            sensor_msgs::PointCloud2 cloud_out;
            tf2::doTransform(*msg, cloud_out, transform);
            pub_.publish(cloud_out);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

public:
    TransformPointCloud() :
        tf_listener_(tf_buffer_)
    {
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_transformed", 1);
        sub_ = nh_.subscribe("os1_cloud_node/points", 1, &TransformPointCloud::pointCloudCallback, this);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_pc");
    TransformPointCloud transform_point_cloud;

    ros::spin();
}