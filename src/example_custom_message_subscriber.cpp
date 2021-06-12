/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/

#include <ros/ros.h>
#include "ros_examples/mymsg.h"

void messageCallback(const ros_examples::mymsg::ConstPtr &msg) {
  ROS_INFO("I heard: [%f]", msg->ax);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mycustommsgsub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("mymessage", 1000, messageCallback);
  ros::spin();
  return 0;
}