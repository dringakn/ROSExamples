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
**/

#include <geometry_msgs/PointStamped.h>  // Transformation point
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>  // Transform broadcaster
#include <tf/transform_listener.h>     // Transform listener

void timerCallback(const tf::TransformListener &li) {
  geometry_msgs::PointStamped pt_w;
  // the time stamp is ros::Time() instead of ros::Time::now()
  pt_w.header.stamp = ros::Time();
  pt_w.header.frame_id = "world";
  pt_w.point.x = 6;
  pt_w.point.y = 6;
  pt_w.point.z = 0;
  geometry_msgs::PointStamped pt_m;
  li.transformPoint("map", pt_w, pt_m);
  ROS_INFO("%f,%f,%f", pt_m.point.x, pt_m.point.y, pt_m.point.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_tf_node");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  static tf::TransformBroadcaster br;
  tf::TransformListener li(ros::Duration(10));
  tf::Transform t;
  tf::StampedTransform st;

  // tf::Transform has '*' overloaded operator for the transformation of
  // vector3, quaternion and transformation object.
  // tf::Vector3 v(p.x, p.y, p.z);
  // v = t2 * v;
  // p.x = v.x();
  // p.y = v.y();
  // p.z = v.z();

  ros::Timer timer = nh.createTimer(
      ros::Duration(2), boost::bind(&timerCallback, boost::ref(li)), false);

  while (ros::ok()) {
    // Broadcast map to world transformation
    t.setOrigin(tf::Vector3(5, 5, 0));
    t.setRotation(tf::createQuaternionFromYaw(0));
    br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "world", "map"));
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

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}