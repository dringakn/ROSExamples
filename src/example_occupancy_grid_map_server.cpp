/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Create a random OGM and send it on service request.
        For testing:
        rosservice call /get_map "{}"
    Notes:
        modify find_packages(... geometry_msgs nav_msgs ...)

*/

#include <geometry_msgs/Pose.h>      // Map pose
#include <nav_msgs/GetMap.h>         // Map Service
#include <nav_msgs/OccupancyGrid.h>  // Map message
#include <ros/ros.h>                 // ROS related stuff

bool getmap_cb(nav_msgs::GetMap::Request &req,
               nav_msgs::GetMap::Response &res) {
  res.map.header.seq = 0;
  res.map.header.frame_id = "map";
  res.map.header.stamp = ros::Time::now();
  res.map.info.map_load_time = ros::Time::now();
  res.map.info.width = 10;
  res.map.info.height = 10;
  res.map.info.resolution = 1;
  res.map.info.origin = geometry_msgs::Pose();
  res.map.data.resize(res.map.info.width * res.map.info.height, 0);
  // -1=Unknown, 0=Free, 100=Occupied
  for (auto &&c : res.map.data) {
    c = -1 + rand() % 101;
  }
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_occupancy_grid_map_server");
  ros::NodeHandle nh;

  ros::ServiceServer server;
  server = nh.advertiseService("get_map", getmap_cb);

  ros::spin();

  return 0;
}