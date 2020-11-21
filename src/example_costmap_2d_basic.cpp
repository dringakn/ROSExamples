/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Create a random OGM and send it on service request.
        For testing:
        rosservice call /get_map "{}"

    This package provides an implementation of a 2D costmap that takes in
    sensor data from the world, builds a 2D or 3D occupancy grid of the data
    (depending on whether a voxel based implementation is used), and inflates
    costs in a 2D costmap based on the occupancy grid and a user specified
    inflation radius. This package also provides support for map_server based
    initialization of a costmap, rolling window based costmaps, and parameter
    based subscription to and configuration of sensor topics.

    Published Topics:
      /obstacles (nav_msgs/GridCells)
      /inflated_obstacles (nav_msgs/GridCells)
      /unknown_space (nav_msgs/GridCells)
      /voxel_grid (costmap_2d/VoxelGrid)
    Subscribed Topics:
      <point_cloud_topic> (sensor_msgs/PointCloud)
      <laser_scan_topic> (sensor_msgs/LaserScan)
      "map" (nav_msgs/OccupancyGrid)
    Parameters:
      /global_frame (string, default: "/map")
      /robot_base_frame (string, default: "base_link")
      /transform_tolerance (double, default: 0.2)
      /update_frequency (double, default: 5.0)
      /publish_frequency (double, default: 0.0)
      Global costmap parameters:
      /max_obstacle_height (double, default: 2.0)
      /obstacle_range (double, default: 2.5)
      /raytrace_range (double, default: 3.0)
      /cost_scaling_factor (double, default: 10.0)
      exp(-1.0 * cost_scaling_factor * (distance_from_obstacle -
    inscribed_radius)) * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1), where
    costmap_2d::INSCRIBED_INFLATED_OBSTACLE is currently 254. Robot description
    parameters: /inflation_radius (double, default: 0.55) /footprint (list,
    default: []) /robot_radius (double, default: 0.46) Sensor management
    parameters: /observation_sources (string, default: "")
      /<source_name>/topic (string, default: source_name)
      /<source_name>/sensor_frame (string, default: "")
      /<source_name>/observation_persistence (double, default: 0.0)
      /<source_name>/expected_update_rate (double, default: 0.0)
      /<source_name>/data_type (string, default: "PointCloud")
      /<source_name>/clearing (bool, default: false)
      /<source_name>/marking (bool, default: true)
      /<source_name>/max_obstacle_height (double, default: 2.0)
      /<source_name>/min_obstacle_height (double, default: 0.0)
      /<source_name>/obstacle_range (double, default: 2.5)
      /<source_name>/raytrace_range (double, default: 3.0)
      Map management parameters:
      /static_map (bool, default: true)
      /rolling_window (bool, default: false)
      /unknown_cost_value (int, default: "0")
      /publish_voxel_map (bool, default: false)
      /lethal_cost_threshold (int, default: 100)
      /map_topic (string, default: "map")
      ...

    Notes: modify
    find_packages(... geometry_msgs nav_msgs ...)

*/

#include <costmap_2d/cost_values.h>           // Definations of LETHAL, FREE, UNKNOWN
#include <costmap_2d/costmap_2d.h>            // ROS costmap_2d
#include <costmap_2d/costmap_2d_publisher.h>  // Costmap publisher node
#include <costmap_2d/costmap_2d_ros.h>        // Costmap2dROS (complete handling)
#include <costmap_2d/costmap_math.h>          // distanceToLine(), intersects
#include <costmap_2d/footprint.h>             // Footprint math
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/testing_helper.h>  // Printing, counting, copy, layers
#include <geometry_msgs/Pose.h>         // Map pose
#include <nav_msgs/GetMap.h>            // Map Service
#include <nav_msgs/OccupancyGrid.h>     // Map message
#include <ros/ros.h>                    // ROS related stuff
#include <tf/transform_listener.h>      // TF Listener

using namespace std;

/**
 * @brief Create a Costmap2D object from OccupancyGrid message.
 *
 * @param msg Pointer to OccupancyGrid message.
 * @return costmap_2d::Costmap2D
 */
costmap_2d::Costmap2D createCostmap2D(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  costmap_2d::Costmap2D cm(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x,
                           msg->info.origin.position.y, costmap_2d::FREE_SPACE);
  std::copy(msg->data.begin(), msg->data.end(), cm.getCharMap());
  // const unsigned int COLS = cm.getSizeInCellsX();
  // for (int r = 0; r < msg->info.height; r++)
  //   for (int c = 0; c < msg->info.width; c++)
  //     cm.setCost(c, r, msg->data[r * COLS + c]);
  return cm;
}

/**
 * @brief Update the existing costmap with the OccupancyGrid message
 *
 * @param cm Reference to the costmap to be updated.
 * @param msg Pointer to OccupancyGrid message.
 */
void updateCostmap2D(costmap_2d::Costmap2D& cm, const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  cm.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x,
               msg->info.origin.position.y);
  std::copy(msg->data.begin(), msg->data.end(), cm.getCharMap());
  // const unsigned int COLS = cm.getSizeInCellsX();
  // for (int r = 0; r < msg->info.height; r++)
  //   for (int c = 0; c < msg->info.width; c++)
  //     cm.setCost(c, r, msg->data[r * COLS + c]);
}

/**
 * @brief Get the Occupancy Grid Map(OG)
 *
 * @param map OGM
 * @param topic Service topic name
 * @return true if the service is available and map recieved
 * @return false otherewise
 */
bool getMap(nav_msgs::OccupancyGrid& map, std::string topic = "/map")
{
  if (ros::service::waitForService(topic, ros::Duration(1)))
  {
    ros::ServiceClient client = ros::service::createClient<nav_msgs::GetMap>(topic);
    if (client.exists())
    {
      nav_msgs::GetMap::Request req;
      nav_msgs::GetMap::Response res;
      if (client.call(req, res))
      {
        map = res.map;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
    return true;
  }
  else
  {
    return false;
  }
}

// costmap_2d::Costmap2D costmap(100, 100, 0.1, 0, 0, costmap_2d::FREE_SPACE);  // non nullptr
costmap_2d::Costmap2D costmap;  // nullptr
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  updateCostmap2D(costmap, msg);
}

int main(int argc, char* argv[])
{
  cout.precision(1);

  ros::init(argc, argv, "example_costmap_2d_basic");
  ros::NodeHandle nh("~");
  std::string file_name;
  if (nh.param<std::string>("file_name", file_name, "delme.pgm"))
    nh.setParam("file_name", file_name);
  ROS_INFO("File name: %s", file_name.c_str());

  ros::Subscriber subMap = nh.subscribe("/map", 1, mapCallBack);

  /*
    While each cell in the costmap can have one of 255 different cost values,
    the underlying structure that it uses is capable of representing only
    three (costmap_2d::LETHAL_OBSTACLE, costmap_2d::FREE_SPACE,
    costmap_2d::NO_INFORMATION). Specifically, each cell in the data structure
    can be either free, occupied, or unknown. Each status has a special cost
    value assigned to it upon projection into the costmap. Columns that have a
    certain number of occupied cells (mark_threshold parameter) are assigned
    a costmap_2d::LETHAL_OBSTACLE cost, columns that have a certain number of
    unknown cells (unknown_threshold parameter) are assigned a
    costmap_2d::NO_INFORMATION cost, and other columns are assigned a
    costmap_2d::FREE_SPACE cost.
    costmap_2d::INSCRIBED_INFLATED_OBSTACLE
  */

  // costmap.cellDistance(double world_distance);
  // costmap.convexFillCells(vector<MapLocation>, vector<MapLocation>)
  // costmap.copyCostmapWindow(map,x,y,dx,dy)
  // costmap.getCost()
  // costmap.getIndex()
  // costmap.getOriginX()
  // costmap.getResolution()
  // costmap.getSizeInCellsX()
  // costmap.getSizeInMetersX()
  // costmap.indexToCells()
  // costmap.mapToWorld()
  // costmap.polygonOutlineCells()
  // costmap.resetMap()
  // costmap.resizeMap()
  // costmap.saveMap(filename)
  // costmap.setConvexPolygonCost()
  // costmap.setCost(x,y,cost)
  // costmap.updateOrigin(x,y)
  // costmap.worldToMap(wx,wy,mx,my)
  // costmap.worldToMapEnforceBounds(wx,wy,mx,my)
  // costmap.worldToMapNoBounds(wx,wy,mx,my)

  // Wait for the map to be available
  ROS_INFO("Waiting for map...");
  while (costmap.getCharMap() == nullptr)
  {
    ROS_INFO_THROTTLE(1, "Waiting for map...");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Map received.");
  // Save the current map to a pgm file
  if (costmap.saveMap(file_name))
    ROS_INFO("Map saved to: %s", file_name.c_str());
  else
    ROS_INFO("Error saving map.");

  //   for (int i = 0; i < costmap.getSizeInCellsX(); ++i) {
  //     for (int j = 0; j < costmap.getSizeInCellsY(); ++j) {
  //       cout << (int)costmap.getCost(i, j) << " ,";
  //     }
  //     cout << endl;
  //   }

  vector<costmap_2d::MapLocation> polygon, polygon_cells;
  // Polygon coordinates are in the map coordinates
  polygon.push_back({ 0, 0 });
  polygon.push_back({ 0, 5 });
  polygon.push_back({ 5, 5 });
  polygon.push_back({ 5, 0 });
  costmap.convexFillCells(polygon, polygon_cells);
  ROS_INFO("convexFillCells(...): %d", polygon_cells.size());
  int idx = 0;
  for (auto&& c : polygon_cells)
    cout << idx++ << ": " << c.x << "," << c.y << endl;

  polygon_cells.clear();
  costmap.polygonOutlineCells(polygon, polygon_cells);
  ROS_INFO("polygonOutlineCells(...): %d", polygon_cells.size());
  idx = 0;
  for (auto&& c : polygon_cells)
    cout << idx++ << ": " << c.x << "," << c.y << endl;

  vector<geometry_msgs::Point> polygon1;
  geometry_msgs::Point pt;
  pt.x = 0, pt.y = 0, polygon1.push_back(pt);
  pt.x = 0, pt.y = 5, polygon1.push_back(pt);
  pt.x = 5, pt.y = 5, polygon1.push_back(pt);
  pt.x = 5, pt.y = 0, polygon1.push_back(pt);
  if (costmap.setConvexPolygonCost(polygon1, costmap_2d::LETHAL_OBSTACLE))
    ROS_INFO("Success setConvexPolygonCost(...)");
  else
    ROS_INFO("Failed setConvexPolygonCost(...)");

  costmap_2d::Costmap2D cm1;
  if (costmap.copyCostmapWindow(cm1, 0, 0, 5, 5))
    ROS_INFO("Success copyCostmapWindow(...)");
  else
    ROS_INFO("Failed copyCostmapWindow(...)");

  ROS_INFO("cellDistance(1): %d", costmap.cellDistance(0.1));

  return 0;
}