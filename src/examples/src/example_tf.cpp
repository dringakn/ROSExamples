/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include "ros/ros.h"
#include "tf/transform_broadcaster.h" // Transform publisher
#include "tf/transform_listener.h"    // Transform subscriber
int main(int argc, char **argv) {
  ros::init(argc, argv, "example_tf_node");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}