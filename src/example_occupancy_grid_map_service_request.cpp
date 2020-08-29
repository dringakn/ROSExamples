/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/

// modify find_packages(... geometry_msgs nav_msgs ...)
// rosrun map_server map_server map.yaml
// rosrun example example_occupancy_grid_map_service_request static_map grid.txt

#include <nav_msgs/GetMap.h>         // Service
#include <nav_msgs/OccupancyGrid.h>  // Occupancy grid map
#include <ros/ros.h>                 // ROS

#include <fstream>  // Text file
#include <vector>   // vector container

using namespace std;

int width, height;
double resolution;
vector<vector<bool>> grid;  // store occupancy map (0=free, 1=occupied)

bool requestMap(ros::NodeHandle &, const char *);
void readMap(const nav_msgs::OccupancyGrid &);
void saveMapToFile(const char *);

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_occupancy_grid_map_service_request");
  ros::NodeHandle n;
  const char *topicName = (argc >= 1) ? argv[1] : "static_map";
  const char *fileName = (argc >= 2) ? argv[2] : "grid.txt";
  if (requestMap(n, topicName)) {
    ROS_INFO("Map request successful.");
    saveMapToFile(fileName);
    return 1;
  } else {
    ROS_INFO("Map service call failed.");
    return -1;
  }
}

bool requestMap(ros::NodeHandle &nh, const char *topicName) {
  while (!ros::service::waitForService(
      topicName, 3000))  // Number of millisecons to wait, -1 for infinite
    ROS_INFO("Waiting for the map service (%s) to become available", topicName);
  ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>(topicName);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if (mapClient.call(req, res)) {
    readMap(res.map);
    return true;
  } else {
    return false;
  }
}

void readMap(const nav_msgs::OccupancyGrid &map) {
  width = map.info.width;
  height = map.info.height;
  resolution = map.info.resolution;
  ROS_INFO("Map: %d x %d @ %0.3f m/px", width, height, resolution);
  grid.resize(height, vector<bool>(width));
  int idx = 0;
  for (size_t r = 0; r < height; r++)
    for (size_t c = 0; c < width; c++) {
      // Occupied or Unknown(-1) are set as 1
      grid[r][c] = (map.data[idx] == 0) ? 0 : 1;
      idx++;
    }
}

void saveMapToFile(const char *fileName) {
  ofstream gridFile;
  gridFile.open(fileName);
  for (size_t r = 0; r < height; r++) {
    for (size_t c = 0; c < width; c++) {
      gridFile << (grid[r][c]) ? "1" : "0";
    }
    gridFile << endl;
  }
  gridFile.close();
  ROS_INFO("%s saved successfully.", fileName);
}