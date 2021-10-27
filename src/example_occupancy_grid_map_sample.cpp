/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/

// modify find_packages(... geometry_msgs nav_msgs ...)
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

inline bool set_occupancy_grid_cell(nav_msgs::OccupancyGrid &map, geometry_msgs::Point pt, uint8_t value)
{
  // Return false if pt is outside map
  // index = total_columns x row_index + col_index
  unsigned int x = floor(pt.x / map.info.resolution);
  unsigned int y = floor(pt.y / map.info.resolution);
  if ((x < map.info.width) && (y < map.info.height))
  {
    map.data[map.info.width * y + x] = value;
    return true;
  }
  return false;
}

inline bool get_occupancy_grid_cell(nav_msgs::OccupancyGrid &map, geometry_msgs::Point pt, uint8_t &value)
{
  // Return false if pt is outside map
  // index = total_columns x row_index + col_index
  unsigned int x = floor(pt.x / map.info.resolution);
  unsigned int y = floor(pt.y / map.info.resolution);
  if ((x < map.info.width) && (y < map.info.height))
  {
    value = map.data[map.info.width * y + x];
    return true;
  }
  return false;
}

inline bool horziontal_line(nav_msgs::OccupancyGrid &map, geometry_msgs::Point pt, uint8_t value)
{
  // Return false if pt is outside map
  // Draw a horizontal line passing through the point pt (along the width or x-axis)
  // index = total_columns x row_index + col_index
  unsigned int x = floor(pt.x / map.info.resolution);
  unsigned int y = floor(pt.y / map.info.resolution);
  if ((x < map.info.width) && (y < map.info.height))
  {
    for (int i = 0; i < map.info.width; i++)
      map.data[map.info.width * y + i] = value;
    return true;
  }
  return false;
}

inline bool vertical_line(nav_msgs::OccupancyGrid &map, geometry_msgs::Point pt, uint8_t value)
{
  // Return false if pt is outside map
  // Draw a vertical line passing through the point pt (along the height or y-axis)
  // index = total_columns x row_index + col_index
  unsigned int x = floor(pt.x / map.info.resolution);
  unsigned int y = floor(pt.y / map.info.resolution);
  if ((x < map.info.width) && (y < map.info.height))
  {
    for (int i = 0; i < map.info.height; i++)
      map.data[map.info.width * i + x] = value;
    return true;
  }
  return false;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "example_occupancy_grid_map_sample");
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  nav_msgs::OccupancyGrid map; // Create OGM message

  map.header.stamp = ros::Time::now();
  map.header.frame_id = "map";             // frame name of the map
  map.info.origin = geometry_msgs::Pose(); // the pose of the cell(0,0)
  map.info.map_load_time = ros::Time::now();

  const float COLS = 10, ROWS = 5, RES = 0.25;
  map.info.width = COLS / RES;  // number of cells along the x-axis
  map.info.height = ROWS / RES; // number of cells along the y-axis
  map.info.resolution = RES;    // side length of cell in meters

  map.data.resize(map.info.width * map.info.height, 0);

  unsigned int index;
  const uint8_t UNKNOWN = -1, FREE = 0, OCCUPIED = 100;

  geometry_msgs::Point pt;
  uint8_t value;
  pt.x = pt.y = 4.75;
  ROS_INFO("%d", set_occupancy_grid_cell(map, pt, OCCUPIED));
  ROS_INFO("%d", get_occupancy_grid_cell(map, pt, value));
  ROS_INFO("%d", value);
  pt.x = pt.y = 1;
  ROS_INFO("%d", horziontal_line(map, pt, OCCUPIED));
  ROS_INFO("%d", vertical_line(map, pt, OCCUPIED));
  map_pub.publish(map);
  ros::spin();

  return 0;
}