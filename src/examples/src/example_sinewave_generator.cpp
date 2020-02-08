/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include <math.h>
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
  // amplitude, frequency and stdev are specified at runtime
  // while noise standard deviation can be changed during execution
  // rosparam set /stdev 0.2
  double amplitude = 1, frequency = 1, stdev = 0.0;
  if (argc > 1) {
    amplitude = atof(argv[1]);
    if (argc > 2) {
      frequency = atof(argv[2]);
      if (argc > 3) {
        stdev = atof(argv[3]);
      }
    }
  }

  ros::init(argc, argv, "function generator");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float64>("signal", 1);
  random_numbers::RandomNumberGenerator rng;
  ros::Rate rate(1000);        // 1Khz sampling rate
  nh.getParam("stdev", stdev); // Get updated value from parmeter server
  ROS_INFO("A:%04.2f F:%04.2f N(0,%04.2f)", amplitude, frequency, stdev);

  while (ros::ok()) {
    nh.getParam("stdev", stdev);
    std_msgs::Float64 msg;
    msg.data =
        amplitude * sin(2 * M_PI * frequency * ros::Time::now().toSec()) +
        rng.gaussian(0, stdev);
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}