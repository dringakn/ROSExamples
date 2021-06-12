/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Desciption: Geodatic to Cartesian Conversion.
 *          Subscribe to a GPS message and use the fromLL serive provided by the robot_localization 
 *          "navsat_transform_node" to convert Geodatic into Cartesian coordinates.
 *          Subscribes to the NavSatFix message and publish it as PointStamped message to be visulized
 *          in RViz.
 *    Note:
 *          The first GPS message is used as a reference for subsequent messages.
 **/

#include <ros/ros.h>                    // ROS
#include <bits/stdc++.h>                // Standard C++
#include <robot_localization/FromLL.h>  // Robot localization LL to XYZ service
#include <sensor_msgs/NavSatFix.h>      // Incoming GPS message
#include <geometry_msgs/PointStamped.h> // Outgoing GPS message
ros::ServiceClient client;              // GPS point conversion service client
ros::Publisher pub;                     // Cartesian point publisher
ros::Subscriber sub;                    // GPS point subscriber

bool geodatic2Cartesian(ros::ServiceClient &client, geographic_msgs::GeoPoint &ll, geometry_msgs::Point &xyz)
{
    bool result = false;
    robot_localization::FromLL srv;
    srv.request.ll_point = ll;
    if (client.call(srv))
    {
        xyz = srv.response.map_point;
        result = true;
        // ROS_INFO("%3.2f, %3.2f, %3.2f -> %3.2f, %3.2f, %3.2f", srv.request.ll_point.latitude,
        //          srv.request.ll_point.longitude,
        //          srv.request.ll_point.altitude,
        //          srv.response.map_point.x,
        //          srv.response.map_point.y,
        //          srv.response.map_point.z);
    }

    return result;
}

void callback(const sensor_msgs::NavSatFix::ConstPtr &msgIn)
{
    geographic_msgs::GeoPoint ll;
    ll.latitude = msgIn->latitude;
    ll.longitude = msgIn->longitude;
    ll.altitude = msgIn->altitude;
    geometry_msgs::PointStamped xyz;
    static geometry_msgs::Point initPos;
    static bool init = false;
    if (geodatic2Cartesian(client, ll, xyz.point))
    {
        xyz.header.frame_id = "odom";
        xyz.header.stamp = ros::Time::now();
        if (!init)
        {
            initPos = xyz.point;
            init = true;
            ROS_INFO("Init. Pos.: %3.2f, %3.2f, %3.2f", initPos.x, initPos.y, initPos.z);
        }
        else
        {
            xyz.point.x -= initPos.x;
            xyz.point.y -= initPos.y;
            xyz.point.z -= initPos.z;
            pub.publish(xyz);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_gps_geodatic_to_cartesian");
    ros::NodeHandle nh("~");
    client = nh.serviceClient<robot_localization::FromLL>("/fromLL", false);
    ros::service::waitForService("/fromLL", ros::Duration(10));           // -1 to wait for infinity
    sub = nh.subscribe<sensor_msgs::NavSatFix>("geodatic", 10, callback); // Initialize subscriber
    pub = nh.advertise<geometry_msgs::PointStamped>("cartesian", 10);     // Initialize publisher
    ros::spin();
    return 0;
}
