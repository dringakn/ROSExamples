/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
      Publish a static transform from "world" to "map" frame
      use view_frames to view the tf graph (evince frames.pdf)
      or rosrun tf tf_monitor world map
      or rosrun tf tf_echo world map
      or rostopic echo /tf
      To publishe a static transform between two frames use the following
      rosrun tf static_transform_publisher x y z yaw pitch roll parent child t_ms
    Notes:
        The class tf::Transform has built-in implementation of multiplication operator (*)
        for the transformation of vector3, quaternion and transformation object.
        For example:
        tf::Vector3 v(p.x, p.y, p.z);
        v = t2 * v;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();

**/

#include <nav_msgs/Odometry.h>         //Odometry message
#include <ros/ros.h>                   // ROS functionality
#include <tf/transform_broadcaster.h>  // Transform broadcaster
#include <tf/transform_listener.h>     // Transform listener

using namespace std;

tf::TransformBroadcaster *br = NULL;  // Transform broadcaster object
tf::TransformListener *li = NULL;     // Transform listener object

/**
 * @brief Odometry callback function.
 *  Publish a tf world->map
 *
 * @param msg
 */
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  //   li.transformPoint("map", pt_w, pt_m);
  // Listen for map to world transformation
  // try {
  //   // 5sec buffer, ros::Time(0) means we want to transform now.
  //   li.waitForTransform("world", "map", ros::Time(0), ros::Duration(5));
  //   li.lookupTransform("world", "map", ros::Time(0), st);
  //   ROS_INFO("%f,%f,%f,%f", st.getOrigin().x(), st.getOrigin().y(),
  //            st.getOrigin().z(), st.getRotation().w());
  // } catch (tf::TransformException ex) {
  //   ROS_INFO("%s", ex.what());
  // }
}

void printTF(tf::Transform &t)
{
  ROS_INFO("Pos: [%+05.2f, %+05.2f, %+05.2f], Rot: [%+05.2f, %+05.2f, %+05.2f, %+05.2f]", t.getOrigin().x(),
           t.getOrigin().y(), t.getOrigin().z(), t.getRotation().x(), t.getRotation().y(), t.getRotation().z(),
           t.getRotation().w());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_tf_gps");
  ros::NodeHandle nh("~");
  br = new tf::TransformBroadcaster();
  li = new tf::TransformListener(ros::Duration(10));

  // Basic coordinate math
  // Compose two transforms; map->robot, robot->sensor
  tf::Transform t_m_r(tf::createQuaternionFromYaw(0), tf::Vector3(10, 0, 0));
  tf::Transform t_r_s(tf::createQuaternionFromYaw(0), tf::Vector3(1, 0, 0));
  tf::Transform t_m_s = t_m_r * t_r_s;
  printTF(t_m_s);

  // Get the tf between map -> robot
  tf::Transform t_s_r = t_r_s.inverse();
  tf::Transform t = t_m_s * t_s_r;  // Should be t_m_r
  printTF(t);

  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}