/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include "examples/mymsg.h"
#include "ros/ros.h"
void messageCallback(const examples::mymsg::ConstPtr &msg) {
  ROS_INFO("I heard: [%f]", msg->ax);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "mycustommsgsub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("mymessage", 1000, messageCallback);
  ros::spin();
  return 0;
}