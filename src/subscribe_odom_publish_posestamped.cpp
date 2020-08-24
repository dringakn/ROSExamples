/*
 * subscribe_odom_publish_posestamped.cpp
 *
 *      Author: Dr. -Ing. Ahmad Kamal Nasir
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sub_odom_pub_posestamped");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string fromFrame(""), toFrame("");
  tf::TransformListener listener(ros::Duration(10));
  ros::Publisher pub =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
  // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  // ros::console::levels::Debug); ros::console::notifyLoggerLevelsChanged();

  geometry_msgs::PoseWithCovarianceStamped msg;
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    pnh.param<std::string>("from_frame", fromFrame, "odom");
    pnh.param<std::string>("to_frame", toFrame, "bobcat_base");
    ROS_DEBUG_THROTTLE(1, "from:%s\tto:%s", fromFrame.c_str(), toFrame.c_str());
    try {
      tf::StampedTransform t;
      listener.lookupTransform(fromFrame, toFrame, ros::Time(0), t);
      msg.header.frame_id = fromFrame;
      msg.header.stamp = ros::Time::now();
      msg.pose.pose.position.x = t.getOrigin().getX();
      msg.pose.pose.position.y = t.getOrigin().getY();
      msg.pose.pose.position.z = t.getOrigin().getZ();
      msg.pose.pose.orientation.x = t.getRotation().getX();
      msg.pose.pose.orientation.y = t.getRotation().getY();
      msg.pose.pose.orientation.z = t.getRotation().getZ();
      msg.pose.pose.orientation.w = t.getRotation().getW();
      pub.publish(msg);
    } catch (const tf::TransformException& e) {
      ROS_WARN("sub_odom_pub_posestamped: %s", e.what());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}