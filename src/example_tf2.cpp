/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        It seems tf2 is working under the hood of tf.
        tf::Transform can be used to do the math algebra operations.
        tf::Transform has '*' overloaded operator for the transformation of vector3, quaternion and transformation object.
        Example:
            tf::Vector3 v(p.x, p.y, p.z);
            v = t2 * v;
            p.x = v.x();
            p.y = v.y();
            p.z = v.z();
        Use tf2 because it has a cleaner interface.
        There are two helper classes to send and receive transforms published on ROS,
            tf::TransformBroadcaster
            tf::TransformListener
        tf::MessageFilter is the recommended method for receiving any sensor data (that has ROS header) from ROS.
        It provides callback whenever the data can be available in the target frame.
        tf Quternion, Vector3, Point, Transform, Pose are the typedef of bullet types.
        The stamped version has stamp, frame_id_, child_frame_id fields.
        ROS uses two quaternion datatypes: msg and 'tf.'
        To convert between them in C++, use the methods of tf2_geometry_msgs.
        Example:
            #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
            tf2::Quaternion q_orig, q_rot, q_new;
            // Get the original orientation of 'commanded_pose'
            tf2::convert(commanded_pose.pose.orientation , q_orig);
            double r=3.14159, p=0, y=0;
            // Rotate the previous pose by 180* about X
            q_rot.setRPY(r, p, y);
            q_new = q_rot*q_orig;  // Calculate the new orientation
            q_new.normalize();
            // Stuff the new rotation back into the pose. This requires conversion into a msg type
            tf2::convert(q_new, commanded_pose.pose.orientation);
        Use the tf2_bullet, tf2_eigen, tf2_kdl, tf2_geometry_msgs, tf2_sensor_msgs as to the specific data types.
        Here are a few references I have explored.
        http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf
        http://docs.ros.org/melodic/api/tf/html/c++/
        http://wiki.ros.org/Papers/TePRA2013_Foote?action=AttachFile&do=view&target=TePRA2013_Foote.pdf
        http://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28C%2B%2B%29
        http://wiki.ros.org/tf2_ros
        https://www.ros.org/reps/rep-0103.html

**/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/MinMax.h>
#include <tf2/LinearMath/QuadWord.h>
#include <tf2/utils.h>                           // getEulerYPR(q, y, p, r), getYaw(q)
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Conversion between msg and tf2 object
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>     // Conversion between msg and tf2 object (sensor_msgs::PointCloud2)
#include <geometry_msgs/PointStamped.h>          // Transformation point

#define DEG2RAD(x) ((x)*M_PI / 180.0)
#define RAD2DEG(x) ((x)*180.0 / M_PI)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_tf_node");
    ros::NodeHandle nh;

    tf2::Quaternion q(0, 0, 0, 1); // x,y,z,w

    // Convert message type to tf2 type quaternion
    geometry_msgs::Quaternion quat_msg;
    tf2::fromMsg(quat_msg, q);

    // Create quaternion using Euler angles
    q.setRPY(0, 0, DEG2RAD(90)); // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();

    // Convert quaternion to Euler angles
    double yaw, pitch, roll;
    tf2::getEulerYPR(q, yaw, pitch, roll);
    ROS_INFO("Yaw:%f, Pitch:%f, Roll:%f", RAD2DEG(yaw), RAD2DEG(pitch), RAD2DEG(roll));

    // Linear Algebra
    tf2::angleShortestPath(q, q);
    tf2::dot(q, q);
    tf2::inverse(q);
    // tf2::lerp(v1, v2, t);
    // tf2::quatRotate(q, v);
    // tf2::shortestArcQuat(v1, v2);
    // tf2::tf2Angle(v1, v2);
    // tf2::tf2Cross(v1, v2);
    // tf2::tf2Distance2(v1, v2);
    // tf2::tf2Dot(v1, v2);
    // tf2::tf2Triple(v1, v2, v3);
    // tf2::tf2PlaneSpace1(n, p, q);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}