/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "random_numbers/random_numbers.h"

int main(int argc, char **argv) {
  // rosrun examples example_robot_navigation_command /cmd_vel:=/turtle1/cmd_vel
  ros::init(argc, argv, "example_robot_navigation_command");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist msg;
  random_numbers::RandomNumberGenerator rng;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    msg.linear.x = rng.gaussian01();
    msg.angular.z = rng.gaussian01();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}