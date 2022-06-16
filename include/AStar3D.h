/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *      Given a start point, end point and an UFO (Unknown, Free, Occupied) map find the shortest path using A* algorithm.
 *      The input points are of standard ROS message type geometry_msgs::PointStamped and the UFO is of type
 *      ufo::map::OccupancyMap.
 *
 *      The output path is of type nav_msgs::Path.
 **/
#include <cstddef>                                   // nullptr
#include <geometry_msgs/Point.h>                     // Start/Goal Point
#include <geometry_msgs/Pose.h>                      // Map origin
#include <geometry_msgs/PoseStamped.h>               // Goal point: move_base/goal
#include <geometry_msgs/PoseWithCovarianceStamped.h> // Start point: /initialpose
#include <iostream>                                  // cout, cin
#include <ufo/map/occupancy_map.h>                   // ufo map
#include <ufomap_msgs/UFOMapMetaData.h>              // ufo map meta data
#include <ufomap_msgs/UFOMapStamped.h>               // UFOMap ROS msg
#include <ufomap_msgs/conversions.h>                 // To convert between UFO and ROS
#include <ufomap_ros/conversions.h>                  // To convert between UFO and ROS
#include <nav_msgs/Path.h>                           // Resultant path
#include <queue>                                     // priority_queue
#include <random_numbers/random_numbers.h>           // random number
#include <ros/ros.h>                                 // ros functionality
#include <tf/transform_listener.h>                   // Transform listener

using namespace std;

#define INF FLT_MAX
/**
 * @brief Intermediate data structure to store voxel cell or node information.
 *        The members includes x(int), y(int), z(int), cost(float). Overloaded with
 *        ">" and "<" operators to compare cost value.
 *
 */
class point
{
public:
    int x, y, z;
    float cost;
    point(int _x = 0, int _y = 0, int _z = 0, float _c = INF) : x(_x), y(_y), z(_z), cost(_c) {}
    bool operator>(const point &rhs) const { return cost > rhs.cost; }
    bool operator<(const point &rhs) const { return cost < rhs.cost; }
};

class AStar3D
{
private:
    geometry_msgs::PointStamped start;
    geometry_msgs::PointStamped goal;
    ufo::map::OccupancyMap::ConstPtr map;
    nav_msgs::Path path;
    float gcost[3][3][3]; // Depth x Rows x Cols
    bool skip_unknown;

public:
    AStar3D();
    virtual ~AStar3D();
    void setStartPoint(geometry_msgs::PointStamped &);
    void setGoalPoint(geometry_msgs::PointStamped &);
    void setMap(const nav_msgs::OccupancyGrid::ConstPtr &);
    int shortestPath();
    nav_msgs::Path getPath();
};