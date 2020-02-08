/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include "examples/mymsg.h"
#include "ros/ros.h"
#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mycustommsgpub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<examples::mymsg>("mymessage", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    examples::mymsg msg;
    msg.ax = 1.0f;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}