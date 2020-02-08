/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/

// modify find_packages(... geometry_msgs nav_msgs ...)
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random_numbers/random_numbers.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_occupancy_grid_map");
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  ros::Rate loop_rate(1);
  nav_msgs::OccupancyGrid map;
  map.header.seq = 0;
  map.header.frame_id = "map";
  map.header.stamp = ros::Time::now();
  map.info.map_load_time = ros::Time::now();
  map.info.width = (argc >= 1) ? atof(argv[1]) : 10;
  map.info.height = (argc >= 2) ? atof(argv[2]) : 10;
  map.info.resolution = (argc >= 3) ? atof(argv[3]) : 1;
  map.info.origin = geometry_msgs::Pose();
  random_numbers::RandomNumberGenerator rng;
  map.data.resize(map.info.width * map.info.height);
  for (size_t i = 0; i < map.info.width; i++)
    for (size_t j = 0; j < map.info.height; j++)
      map.data[i * map.info.width + j] = rng.uniformInteger(-1, 100);
  map_pub.publish(map);

  while (ros::ok()) {
    // map_pub.publish(map);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}