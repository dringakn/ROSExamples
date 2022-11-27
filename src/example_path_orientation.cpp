#include <bits/stdc++.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>                       // Path message
#include <geometry_msgs/PoseStamped.h>           // Pose in the path list
#include <tf2/LinearMath/Quaternion.h>           // tf2 quaternion
#include <tf2/LinearMath/Vector3.h>              // tf2 vector/point
#include <tf2/LinearMath/Scalar.h>               // tf2 real number
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // to/from geometry messages conversion

tf2::Quaternion calculateOrientation(tf2::Vector3 start, tf2::Vector3 end)
{
    tf2::Vector3 direction = (end - start).normalize();
    tf2Scalar yaw = tf2Atan2Fast(direction.y(), direction.x()); // heading/yaw angle
    if (isnan(yaw))
        yaw = 0;
    // Note: -negative is used to flip positive pitch direction
    tf2Scalar pitch = -tf2Atan2Fast(direction.z(), tf2Sqrt((direction.x() * direction.x()) + (direction.y() * direction.y())));
    if (isnan(pitch))
        pitch = 0;
    tf2Scalar roll = 0; // No roll/bank angle is needed
    tf2::Quaternion q;
    q.setEulerZYX(yaw, pitch, roll);
    q.normalize();
    std::cout << "Start: (" << start.x() << ", " << start.y() << ", " << start.z() << ") End: (" << end.x() << ", " << end.y() << ", " << end.z()
              << ") Yaw: " << tf2Degrees(yaw) << ", Pitch: " << tf2Degrees(pitch) << ", Roll: " << tf2Degrees(roll) << std::endl;
    return q;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "example_path_orientation");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<nav_msgs::Path>("/path", 1, true);
    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time();

    geometry_msgs::PoseStamped p;
    tf2::Vector3 p1(0, 0, 0);
    tf2::Vector3 p2(1, 1, 0);
    tf2::Vector3 p3(2, 2, 1);
    tf2::Vector3 p4(2, 2, 2);
    tf2::Quaternion q1 = calculateOrientation(p1, p2).normalize();
    tf2::Quaternion q2 = calculateOrientation(p2, p3).normalize();
    tf2::Quaternion q3 = calculateOrientation(p3, p4).normalize();

    p.header = msg.header;
    tf2::toMsg(p1, p.pose.position);
    p.pose.orientation = tf2::toMsg(q1);
    msg.poses.push_back(p);

    tf2::toMsg(p2, p.pose.position);
    p.pose.orientation = tf2::toMsg(q2);
    msg.poses.push_back(p);

    tf2::toMsg(p3, p.pose.position);
    p.pose.orientation = tf2::toMsg(q3);
    msg.poses.push_back(p);

    tf2::toMsg(p4, p.pose.position);
    p.pose.orientation.x = p.pose.orientation.y = p.pose.orientation.z = 0, p.pose.orientation.w = 1;
    msg.poses.push_back(p);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}