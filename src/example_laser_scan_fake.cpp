/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/

// modify find_packages(... sensor_msgs ...)
#include <math.h>
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_laser_scan_fake");
  ros::NodeHandle nh;
  ros::Publisher laser_pub =
      nh.advertise<sensor_msgs::LaserScan>("laser", 1, true);
  ros::Rate loop_rate(30);
  sensor_msgs::LaserScan laser;
  laser.header.seq = 0;
  laser.header.frame_id = "laser";
  laser.header.stamp = ros::Time::now();
  laser.angle_min = (argc >= 1) ? atof(argv[1]) : 0;
  laser.angle_max = (argc >= 2) ? atof(argv[2]) : M_PI;
  laser.angle_increment = (argc >= 3) ? atof(argv[3]) : M_PI / 180.0;
  laser.time_increment = 1.0 / 30;
  laser.scan_time = 1.0 / 30;
  laser.range_min = (argc >= 4) ? atof(argv[4]) : 1;
  laser.range_max = (argc >= 5) ? atof(argv[5]) : 30;
  random_numbers::RandomNumberGenerator rng;
  int n = abs((laser.angle_max - laser.angle_min) / laser.angle_increment), i;
  laser.ranges.resize(n);
  laser.intensities.resize(n);

  while (ros::ok()) {
    laser.header.stamp = ros::Time::now();
    laser.scan_time = 1.0 / 30;
    for (float angle = laser.angle_min, i = 0; angle < laser.angle_max;
         angle += laser.angle_increment, i++) {
      laser.ranges[i] = rng.uniformReal(laser.range_min, laser.range_max);
      laser.intensities[i] = rng.uniformReal(laser.range_min, laser.range_max);
    }
    laser_pub.publish(laser);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}