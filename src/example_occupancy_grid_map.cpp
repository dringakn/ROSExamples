/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
// Usage: rosrun examples example_occupancy_grid_map 10 10 1 0
// modify find_packages(... geometry_msgs nav_msgs ...)
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_occupancy_grid_map");
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  random_numbers::RandomNumberGenerator rng;
  nav_msgs::OccupancyGrid map;
  map.header.seq = 0;
  map.header.frame_id = "map";
  map.header.stamp = ros::Time::now();
  map.info.map_load_time = ros::Time::now();
  map.info.width = (argc > 1) ? atoi(argv[1]) : 10;
  map.info.height = (argc > 2) ? atoi(argv[2]) : 10;
  map.info.resolution = (argc > 3) ? atof(argv[3]) : 1;
  ROS_INFO("Map: %d x %d @ %0.3f", map.info.width, map.info.height,
           map.info.resolution);
  if (argc > 4 && atoi(argv[4]) == 1) {
    map.info.origin.position.x = rng.uniformInteger(-5, 5);
    map.info.origin.position.y = rng.uniformInteger(-5, 5);
    map.info.origin.position.z = rng.uniformInteger(-5, 5);
  } else {
    map.info.origin = geometry_msgs::Pose();
  }
  map.data.resize(map.info.width * map.info.height);
  //-1=Unknown, 0=Free, 100=Occupied
  for (size_t idx = 0; idx < map.data.size(); idx++)
    map.data[idx] = 100 * rng.uniformInteger(0, 1);

  map_pub.publish(map);

  tf::TransformBroadcaster br;
  tf::Transform t;
  t.setOrigin(tf::Vector3(0, 0, 0));
  t.setRotation(tf::createQuaternionFromRPY(0, 0, 0));

  ros::Rate rate(10);
  while (ros::ok()) {
    // parent, child: broadcast map to world
    br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "world",
                                          map.header.frame_id));
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}