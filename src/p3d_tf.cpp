/*
 * p3d_tf.cpp
 *
 * ROS node that subscribes to the published topic from p3d gazebo plugin,
 * then generates and publish a tf2 frame which can be used for recovering
 * ground truth cone colour.
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

class P3dToTf
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    void p3dCallback(const nav_msgs::OdometryConstPtr &odom)
    {
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = odom->header.stamp;
        transformStamped.header.frame_id = odom->header.frame_id;
        transformStamped.child_frame_id = odom->child_frame_id;

        transformStamped.transform.translation.x = odom->pose.pose.position.x;
        transformStamped.transform.translation.y = odom->pose.pose.position.y;
        transformStamped.transform.translation.z = odom->pose.pose.position.z;

        transformStamped.transform.rotation.x = odom->pose.pose.orientation.x;
        transformStamped.transform.rotation.y = odom->pose.pose.orientation.y;
        transformStamped.transform.rotation.z = odom->pose.pose.orientation.z;
        transformStamped.transform.rotation.w = odom->pose.pose.orientation.w;

        br.sendTransform(transformStamped);
    }

public:
    P3dToTf() :
        nh_("~")
    {
        sub_ = nh_.subscribe("/odom", 1, &P3dToTf::p3dCallback, this);
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "p3d_tf_node");
    P3dToTf p3d_to_tf;

    ros::spin();
}