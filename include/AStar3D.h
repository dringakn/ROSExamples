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
    geometry_msgs::PointStamped start;              // Starting location
    geometry_msgs::PointStamped goal;               // Goal location
    nav_msgs::Path path;                            // Resultant path
    bool occupied_space, free_space, unknown_space; // UFO search flags
    ufo::map::DepthType min_depth;                  // UFO depth level for search
    ufo::geometry::BoundingVar bounding_volume;     // Bounding geometry for search
    float gcost[3][3][3];                           // Depth x Rows x Cols

protected:
    bool findNearestVoxel(ufo::map::Point3 position, ufo::map::Point3 *result); // Internal use

public:
    ufo::map::OccupancyMap map;                        // The UFO map
    AStar3D(bool, bool, bool, ufo::map::DepthType);    // Constructor (occ, free, unknown, depth)
    virtual ~AStar3D();                                // Destructor
    void setStartPoint(geometry_msgs::PointStamped &); // Set starting locatoin
    void setGoalPoint(geometry_msgs::PointStamped &);  // Set goal location
    void setBounds(ufo::geometry::BoundingVar *);      // Set bounding geometry for search
    int shortestPath();                                // Search path
    nav_msgs::Path getPath();                          // Get the previous calculated path
};