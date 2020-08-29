/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Desciption: Subscribe to the custom messages "sensor" and "command"
 **/

#include <ros/ros.h>
#include <ros_examples/cmdmsg.h>
#include <ros_examples/sensmsg.h>

void subCallback1(const ros_examples::cmdmsg::ConstPtr& msg) {
  ROS_INFO("VL:%f VR:%f", msg->vl, msg->vr);
}

void subCallback2(const ros_examples::sensmsg::ConstPtr& msg) {
  ROS_INFO("F:%f L:%f R:%f", msg->front, msg->left, msg->right);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_custom_message_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("command", 1000, subCallback1);
  ros::Subscriber sub2 = nh.subscribe("sensor", 1000, subCallback2);
  ros::spin();
  return 0;
}