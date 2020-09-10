/*
 * conepose.cpp
 *
 * ROS node to recover all the known links and corresponding poses.
 * Then filter out potential links of interest, that is, blue and yellow cones.
 *  - Subscribes to /lidar_cone_centres (cluster_node)
 *  - Convert iterate through point cloud and combine that with ground truth colour
 *    then publish a mur_common cone_msg
 *  - Publishes to /cone_messages_sim
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetLinkState.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/distances.h>

#include <mur_common/cone_msg.h>

#define BLUE_STR "BLUE"
#define ORANGE_STR "ORANGE"
#define YELLOW_STR "YELLOW"
#define BIG_STR "BIG"
#define UNKNOWN_STR "na"

class AddConeColour
{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher markers_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /*
     * pointCloudCallback
     * Callback function to process the incoming point cloud and tag on the
     * ground truth colour.
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        geometry_msgs::TransformStamped transform;
        const std::string target_frame = "map"; // use "odom" with tortoisebot
        const std::string source_frame = msg->header.frame_id;
        sensor_msgs::PointCloud2 cloud_out;

        try
        {
            // (target_frame, source_frame) preserves world coordinates but
            // changes parent to target_frame
            // (source_frame, target_frame) shifts point cloud to target_frame
            // but frame parent does not change
            // ! want to do some time travel here
            transform = tf_buffer_.lookupTransform(
                target_frame,
                source_frame,
                msg->header.stamp);
            tf2::doTransform(*msg, cloud_out, transform);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        const std::string model_name = "track";
        std::vector<std::string> cone_link_names;   // orange_cone_xx, blue_cone_xx, big_cone_xx
        std::vector<pcl::PointXYZ> cone_link_poses; // store cone link positions

        ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
        gazebo_msgs::GetModelProperties gmp_srv;
        gmp_srv.request.model_name = model_name;

        if (client.call(gmp_srv))
        {
            cone_link_names = gmp_srv.response.body_names;

            for (auto const &cone : cone_link_names)
            {
                client = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
                gazebo_msgs::GetLinkState gls_srv;
                gls_srv.request.link_name = model_name + "::" + cone;

                if (client.call(gls_srv))
                {
                    pcl::PointXYZ pos;
                    pos.x = gls_srv.response.link_state.pose.position.x;
                    pos.y = gls_srv.response.link_state.pose.position.y;
                    pos.z = gls_srv.response.link_state.pose.position.z;
                    cone_link_poses.push_back(pos);
                }
                else
                {
                    ROS_ERROR("Failed to call service get link state");
                    return;
                }
            }
        }
        else
        {
            ROS_ERROR("Failed to call servce get model properties");
            return;
        }

        // By this point, we should have all the detected cones and ground truth in the same world frame.
        // Loop through the detected cones and build the required cone_msg
        mur_common::cone_msg cone_msg;

        pcl::PointCloud<pcl::PointXYZ> cones_lidar;
        pcl::fromROSMsg(*msg, cones_lidar);

        pcl::PointCloud<pcl::PointXYZ> cones_world;
        pcl::fromROSMsg(cloud_out, cones_world);

        visualization_msgs::MarkerArray marker_array_msg;
        marker_array_msg.markers.resize(cones_lidar.size());

        for (int i = 0; i < cones_lidar.size(); ++i)
        {
            std::string cone_colour = FindConeColour(cones_world[i], cone_link_poses, cone_link_names);
            cone_msg.x.push_back(cones_lidar[i].x);
            cone_msg.y.push_back(cones_lidar[i].y);
            // cone_msg.x.push_back(cones_world[i].x);
            // cone_msg.y.push_back(cones_world[i].y);
            cone_msg.colour.push_back(cone_colour);

            // Update marker for visualisation
            setMarkerProperties(
                &marker_array_msg.markers[i],
                cones_lidar[i],
                i,
                cone_colour,
                msg->header.frame_id);
        }
        cone_msg.frame_id = msg->header.frame_id;

        pub_.publish(cone_msg);
        markers_pub_.publish(marker_array_msg);
    }

    /*
     * FindConeColour
     * Find and return the ground truth cone colour.
     * Will return one of the following [BLUE_STR, YELLOW_STR, UNKNOWN_STR]
     */
    std::string FindConeColour(
        pcl::PointXYZ detected,
        std::vector<pcl::PointXYZ> link_pos,
        std::vector<std::string> link_names)
    {
        // threshold for euclidean distance from detected to true link position
        const float thresh = 0.4;
        int idx = -1;

        for (int i = 0; i < link_pos.size(); ++i)
        {
            float dist = pcl::euclideanDistance(detected, link_pos[i]);

            // DEBUG
            // if (dist < 2.0)
            // {
            //     ROS_INFO("dist = %f", dist);
            // }

            if (dist < thresh)
            {
                idx = i;
                break;
            }
        }

        if (idx == -1)
        {
            ROS_INFO("FindConeColour idx is -1");
            return UNKNOWN_STR;
        }

        // found the ground truth cone, proceed to return the colour
        // orange_cone_xx, blue_cone_xx, yellow_cone_xx, big_cone_xx
        const std::string name = link_names[idx];

        if (name.find("blue") != std::string::npos)
        {
            return BLUE_STR;
        }
        else if (name.find("orange") != std::string::npos)
        {
            return ORANGE_STR;
        }
        else if (name.find("yellow") != std::string::npos)
        {
            return YELLOW_STR;
        }
        else if (name.find("big") != std::string::npos)
        {
            return BIG_STR;
        }
        else
        {
            return UNKNOWN_STR;
        }
    }

    void setMarkerProperties(
        visualization_msgs::Marker *marker,
        pcl::PointXYZ centre,
        int n,
        std::string colour,
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

        marker->scale.x = 0.3;
        marker->scale.y = 0.3;
        marker->scale.z = 0.7;

        // alpha and RGB settings
        marker->color.a = 0.4;

        if (colour == BLUE_STR)
        {
            marker->color.r = 0.0;
            marker->color.g = 0.0;
            marker->color.b = 1.0;
        }
        else if (colour == ORANGE_STR)
        {
            marker->color.r = 1.0;
            marker->color.g = 0.65;
            marker->color.b = 0.0;
        }
        else if (colour == YELLOW_STR)
        {
            marker->color.r = 1.0;
            marker->color.g = 1.0;
            marker->color.b = 0.0;
        }
        else if (colour == BIG_STR)
        {
            marker->color.r = 1.0;
            marker->color.g = 0.0;
            marker->color.b = 0.0;
        }
        else // UNKNOWN_STR
        {
            marker->color.r = 1.0;
            marker->color.g = 1.0;
            marker->color.b = 1.0;
        }

        marker->lifetime = ros::Duration(0.5);
    }

public:
    AddConeColour() :
        tf_listener_(tf_buffer_),
        nh_("~")
    {
        sub_ = nh_.subscribe("/cluster/lidar_cone_centres", 1, &AddConeColour::pointCloudCallback, this);
        pub_ = nh_.advertise<mur_common::cone_msg>("cone_messages_sim", 1);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cone_markers_sim", 1);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conepose_node");
    AddConeColour add_cone_colour;

    ros::spin();
}
