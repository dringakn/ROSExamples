/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/

// modify find_packages(... geometry_msgs nav_msgs ...)
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_occupancy_grid_map_sample");
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  nav_msgs::OccupancyGrid map;
  map.header.seq = 0;
  map.header.frame_id = "map";
  map.header.stamp = ros::Time::now();
  map.info.map_load_time = ros::Time::now();
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 1;
  map.info.origin = geometry_msgs::Pose();
  map.data.resize(map.info.width * map.info.height, 0);
  // -1=Unknown, 0=Free, 100=Occupied
  map_pub.publish(map);
  ros::spin();

  return 0;
}