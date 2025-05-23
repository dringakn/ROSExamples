/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description: Create a topic which publishes a uniform random number [0-99]
 *                 every 100 milliseconds
 **/
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "example_random_number_topic");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  random_numbers::RandomNumberGenerator rng;
  ros::Publisher pub = nh.advertise<std_msgs::Float32>("topic", 1000);

  while (ros::ok()) {
    std_msgs::Float32 msg;
    msg.data = rng.uniformInteger(0, 99);
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}