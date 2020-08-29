/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <ros/ros.h>

void timer_cb(const ros::TimerEvent& event) {
  ROS_INFO("Timer: LD=%f", event.profile.last_duration.toSec());
  // ros::Duration(0.001).sleep();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_timer_simple");
  ros::NodeHandle nh;
  ros::Rate rate(1000);
  // Timer callback is processed only when ros::spinOnce() is called
  // Or the ros::spin() has the controll.
  ros::Timer timer = nh.createTimer(ros::Duration(0.25), timer_cb, false);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  // ros::spin();
  return 0;
}