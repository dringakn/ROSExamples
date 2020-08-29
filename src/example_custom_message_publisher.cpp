/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <sstream>

#include "ros/ros.h"
#include "ros_examples/mymsg.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mycustommsgpub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ros_examples::mymsg>("mymessage", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros_examples::mymsg msg;
    msg.ax = 1.0f;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}