/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "random_numbers/random_numbers.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_random_numbers");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  random_numbers::RandomNumberGenerator rng;
  while (ros::ok()) {
    ROS_INFO("NormalReal(0,1): %+04.2f UniformInteger(-5,5): %+01d ",
             rng.gaussian(0, 1), rng.uniformInteger(-5, 5));
    ROS_INFO("UniformReal(0,1): %+04.2f", rng.uniformReal(-1, 1));
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}