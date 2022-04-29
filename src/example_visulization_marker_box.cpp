/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Modified:  06 November 2020
    Description: 
        Create a box and visulaize it in RViz.
**/
#include <geometry_msgs/PoseArray.h>       // Array of robot poses
#include <random_numbers/random_numbers.h> // Uniform/Normal random number generator
#include <ros/ros.h>                       // ROS functionality
#include <visualization_msgs/Marker.h>     // RVIZ visualization marker

ros::Publisher pubLandMarks; // LandMarks visualization publisher
random_numbers::RandomNumberGenerator rng;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_visulization_marker_box");
    ros::NodeHandle nh;
    // Create and publish the lanmarks location as visulazition message on the topic "Landmarks"
    pubLandMarks = nh.advertise<visualization_msgs::Marker>("LandMarks", 1, true);

    random_numbers::RandomNumberGenerator rng(0); // Random number generator

    visualization_msgs::Marker msg;

    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "BoundingBox";
    msg.type = visualization_msgs::Marker::CUBE;
    msg.action = visualization_msgs::Marker::MODIFY; // ADD, MODIFY, DELETE, DELETEALL
    msg.id = 0;
    msg.lifetime = ros::Duration(-1);
    msg.mesh_resource = ""; // "package://pr2_description/meshes/base_v0/base.dae"
    msg.color.r = 1;
    msg.color.g = 0;
    msg.color.b = 0;
    msg.color.a = 0.25;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    msg.scale.x = 2; // width
    msg.scale.y = 2; // length
    msg.scale.z = 1; // height

    pubLandMarks.publish(msg);

    ros::spin();
    return 0;
}
