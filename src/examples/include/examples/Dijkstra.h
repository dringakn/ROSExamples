/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
#include <cstddef>                     // nullptr
#include <geometry_msgs/Point.h>       // Start/Goal Point
#include <geometry_msgs/Pose.h>        // Map origin
#include <geometry_msgs/PoseStamped.h> // Goal point: move_base/goal
#include <geometry_msgs/PoseWithCovarianceStamped.h> // Start point: /initialpose
#include <iostream>                                  // cout, cin
#include <nav_msgs/MapMetaData.h>                    // OGM meta data
#include <nav_msgs/OccupancyGrid.h>                  // Occupancy Grid Map
#include <nav_msgs/Path.h>                           // Resultant path
#include <queue>                                     // priority_queue
#include <random_numbers/random_numbers.h>           // random number
#include <ros/ros.h>                                 // ros functionality
#include <tf/transform_listener.h>                   // Transform listener

using namespace std;

#define INF FLT_MAX

class point {
public:
  int x, y;
  float cost;
  point(int _x = 0, int _y = 0, float _c = INF) : x(_x), y(_y), cost(_c) {}
  bool operator>(const point &rhs) const { return cost > rhs.cost; }
  bool operator<(const point &rhs) const { return cost < rhs.cost; }
};

class Dijkstra {
private:
  geometry_msgs::PointStamped start;
  geometry_msgs::PointStamped goal;
  nav_msgs::OccupancyGrid::ConstPtr map;
  nav_msgs::Path path;

public:
  Dijkstra();
  virtual ~Dijkstra();
  void setStartPoint(geometry_msgs::PointStamped &);
  void setGoalPoint(geometry_msgs::PointStamped &);
  void setMap(const nav_msgs::OccupancyGrid::ConstPtr &);
  float shortestPath();
  nav_msgs::Path getPath();
};