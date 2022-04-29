/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/

// modify find_packages(... geometry_msgs nav_msgs ...)
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h> // OGM
#include <tf2/utils.h>              // getEulerYPR(), Quaterion, Vector, transform

inline void resize_map(nav_msgs::OccupancyGrid &map, const double &width, const double &height, const double &res, int8_t value = -1)
{
  // const int8_t UNKNOWN = -1, FREE = 0, OCCUPIED = 100; // constant definations
  map.info.resolution = res;              // side length of cell in meters
  map.info.width = width / res;           // number of cells along the x-axis (width)
  map.info.height = height / res;         // number of cells along the y-axis (height)
  map.data.resize(width * height, value); // resize underlying array with default value
  // set the position of the cell(0,0) with respect to the origin, AT CENTER
  map.info.origin.position.x = -(((width / (2 * res)) + 0.5) * res);  // Width in meters, considering discretization
  map.info.origin.position.y = -(((height / (2 * res)) + 0.5) * res); // Height in meters, considering discretization
  map.info.origin.orientation.x = map.info.origin.orientation.y = map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;
}

inline bool get_grid_cell_index(nav_msgs::OccupancyGrid &map, const unsigned int &c, const unsigned int &r, unsigned int &index)
{
  // c=x, r=y => p(x,y)=p(c,r)
  // the first grid cell is at (0,0) while the last one is at (width-1, height-1)
  // Return false if col and row are outside map
  if ((c < map.info.width) && (r < map.info.height))
  {
    // index = total_columns x row_index + col_index
    index = map.info.width * r + c;
    return true;
  }
  return false;
}

inline void to_cartesian(nav_msgs::OccupancyGrid &map, const unsigned int &c, const unsigned int &r, double &x, double &y)
{
  x = (c + 0.5) * map.info.resolution + map.info.origin.position.x;
  y = (r + 0.5) * map.info.resolution + map.info.origin.position.y;
}

inline void to_gridcell(nav_msgs::OccupancyGrid &map, const double &x, const double &y, unsigned int &c, unsigned int &r)
{
  // (x:0, y:0) => (c:-1, r:-1) because lower limit is included in previous gridcell
  // (x:0.99, y:0.99) => (c:0, r:0) because lower limit is included in previous gridcell
  // (x:1, y:1) => (c:1, r:1) because lower limit is included in previous gridcell
  c = round((x - map.info.origin.position.x) / map.info.resolution - 0.5);
  r = round((y - map.info.origin.position.y) / map.info.resolution - 0.5);
}

inline bool set_grid_cell_occupancy(nav_msgs::OccupancyGrid &map, const unsigned int &c, const unsigned int &r, int8_t value)
{
  // c=x, r=y => p(x,y)=p(c,r)
  // the first cell is (0,0) while the last one is (width-1, height-1)
  // Return false if pt is outside map
  if ((c < map.info.width) && (r < map.info.height))
  {
    // index = total_columns x row_index + col_index
    map.data[map.info.width * r + c] = value;
    return true;
  }
  return false;
}

inline bool get_grid_cell_occupancy(nav_msgs::OccupancyGrid &map, const unsigned int &c, const unsigned int &r, int8_t &value)
{
  // c=x, r=y => p(x,y)=p(c,r)
  // the first cell is (0,0) while the last one is (width-1, height-1)
  // Return false if pt is outside map
  if ((c < map.info.width) && (r < map.info.height))
  {
    // index = total_columns x row_index + col_index
    value = map.data[map.info.width * r + c];
    return true;
  }
  return false;
}

inline bool set_occupancy_wrt_origin(nav_msgs::OccupancyGrid &map, const geometry_msgs::Point &pt, const int8_t &value)
{
  // Return false if pt is outside map
  // input point (pt) is in map frame, translate wrt origin
  // unsigned int c = floor((map.info.origin.position.x + pt.x) / map.info.resolution);
  // unsigned int r = floor((map.info.origin.position.y + pt.y) / map.info.resolution);
  unsigned int c, r;
  to_gridcell(map, pt.x, pt.y, c, r);
  return set_grid_cell_occupancy(map, c, r, value);
}

inline bool get_occupancy_wrt_origin(nav_msgs::OccupancyGrid &map, const geometry_msgs::Point &pt, int8_t &value)
{
  // Return false if pt is outside map
  // input point (pt) is in map frame, translate wrt origin
  // unsigned int c = floor((map.info.origin.position.x + pt.x) / map.info.resolution);
  // unsigned int r = floor((map.info.origin.position.y + pt.y) / map.info.resolution);
  unsigned int c, r;
  to_gridcell(map, pt.x, pt.y, c, r);
  return get_grid_cell_occupancy(map, c, r, value);
}

inline bool set_horziontal_occupancy_wrt_origin(nav_msgs::OccupancyGrid &map, const geometry_msgs::Point &pt, const int8_t &value)
{
  // Return false if pt is outside map
  // input point (pt) is in map frame, translate wrt origin
  // Draw a horizontal line passing through the point pt (along the width or x-axis)
  unsigned int c, r;
  to_gridcell(map, pt.x, pt.y, c, r);

  if ((c < map.info.width) && (r < map.info.height))
  {
    for (int i = 0; i < map.info.width; i++)
      map.data[map.info.width * r + i] = value;
    return true;
  }
  return false;
}

inline bool set_vertical_occupancy_wrt_origin(nav_msgs::OccupancyGrid &map, const geometry_msgs::Point &pt, const int8_t &value)
{
  // Return false if pt is outside map
  // input point (pt) is in map frame, translate wrt origin
  // Draw a vertical line passing through the point pt (along the height or y-axis)
  unsigned int c, r;
  to_gridcell(map, pt.x, pt.y, c, r);
  if ((c < map.info.width) && (r < map.info.height))
  {
    for (int i = 0; i < map.info.height; i++)
      map.data[map.info.width * i + c] = value;
    return true;
  }
  return false;
}

int main(int argc, char *argv[])
{
  std::cout.precision(3);                              // Show three places after the decimal
  std::cout.setf(std::ios::fixed | std::ios::showpos); // Set fixed point format and '+'

  ros::init(argc, argv, "example_occupancy_grid_map_sample");
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  nav_msgs::OccupancyGrid map;                          // Create OGM message
  const int8_t UNKNOWN = -1, FREE = 0, OCCUPIED = 100; // constant definations
  const float COLS = 1000, ROWS = 1000, RES = 0.25;           // in meters

  map.info.width = COLS / RES;  // number of cells along the x-axis (width)
  map.info.height = ROWS / RES; // number of cells along the y-axis (height)
  map.info.resolution = RES;    // side length of cell in meters
  ROS_INFO("Width: %d, Height:%d, Res:%5.2f", map.info.width, map.info.height, map.info.resolution);

  map.data.resize(map.info.width * map.info.height, UNKNOWN); // resize underlying array with default value

  map.header.frame_id = "map";               // frame name of the map
  map.header.stamp = ros::Time::now();       // message publish time
  map.info.map_load_time = ros::Time::now(); // map load/creation time

  // set the position of the cell(0,0) with respect to the origin
  map.info.origin.position.x = -(((COLS / (2 * RES)) + 0.5) * RES); // Width in meters, considering discretization
  map.info.origin.position.y = -(((ROWS / (2 * RES)) + 0.5) * RES); // Height in meters, considering discretization
  // map.info.origin.position.x = -9; // Width in meters, considering discretization
  // map.info.origin.position.y = -4; // Height in meters, considering discretization
  map.info.origin.orientation.x = map.info.origin.orientation.y = map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;
  ROS_INFO("origin: %5.2f, %5.2f", map.info.origin.position.x, map.info.origin.position.y);

  // Test conversion of row,col index to the crtesian coordinates
  // convert the grid cell location (col, row) into cartesian coordinates (x, y)
  double x, y;
  to_cartesian(map, atof(argv[1]), atof(argv[2]), x, y);
  ROS_INFO("to_cartesian (%3.1f, %3.1f): %3.1f, %3.1f", atof(argv[1]), atof(argv[2]), x, y);

  // Test conversion from crtesian coordinates to the row,col index
  // convert the cartesian coordinates (x, y) into grid cell location (col, row)
  unsigned int c, r;
  to_gridcell(map, atof(argv[1]), atof(argv[2]), c, r);
  ROS_INFO("to_gridcell (%3.1f, %3.1f): %d, %d", atof(argv[1]), atof(argv[2]), c, r);

  // Test set/get grid cell occupancy
  int8_t value = OCCUPIED; // value at the pt location
  // ROS_INFO("set_grid_cell_occupancy: %d", set_grid_cell_occupancy(map, 9, 4, value)); // set occupancy value at grid cell location
  // ROS_INFO("get_grid_cell_occupancy: %d", get_grid_cell_occupancy(map, 9, 4, value)); // get occupancy value at grid cell location
  // ROS_INFO("get_grid_cell_occupancy: %d", value);                                     // display value

  // Test set/get occupancy using cartesian coordinates (wrt origin)
  geometry_msgs::Point pt;                                                            // location with in the map wrt origin [meters]
  pt.x = 1;
  pt.y = 1;                                                                  // set cartesian coordinates
  ROS_INFO("set_occupancy_wrt_origin: %d", set_occupancy_wrt_origin(map, pt, value)); // set occupancy value at location
  ROS_INFO("get_occupancy_wrt_origin: %d", get_occupancy_wrt_origin(map, pt, value)); // get occupancy value at location
  ROS_INFO("%d", value);                                                              // display value
  // pt.x = pt.y = 1;                                                                    // set cartesian coordinates
  // ROS_INFO("set_occupancy_wrt_origin: %d", set_occupancy_wrt_origin(map, pt, value)); // set occupancy value at location
  // ROS_INFO("get_occupancy_wrt_origin: %d", get_occupancy_wrt_origin(map, pt, value)); // get occupancy value at location
  // ROS_INFO("%d", value);                                                              // display value
  // pt.x = pt.y = 2;                                                                    // set cartesian coordinates
  // ROS_INFO("set_occupancy_wrt_origin: %d", set_occupancy_wrt_origin(map, pt, value)); // set occupancy value at location
  // ROS_INFO("get_occupancy_wrt_origin: %d", get_occupancy_wrt_origin(map, pt, value)); // get occupancy value at location
  // ROS_INFO("%d", value);                                                              // display value
  // pt.x = pt.y = 3;                                                                    // set cartesian coordinates
  // ROS_INFO("set_occupancy_wrt_origin: %d", set_occupancy_wrt_origin(map, pt, value)); // set occupancy value at location
  // ROS_INFO("get_occupancy_wrt_origin: %d", get_occupancy_wrt_origin(map, pt, value)); // get occupancy value at location
  // ROS_INFO("%d", value);                                                              // display value
  // pt.x = pt.y = -1;                                                                   // set cartesian coordinates
  // ROS_INFO("set_occupancy_wrt_origin: %d", set_occupancy_wrt_origin(map, pt, value)); // set occupancy value at location
  // ROS_INFO("get_occupancy_wrt_origin: %d", get_occupancy_wrt_origin(map, pt, value)); // get occupancy value at location
  // ROS_INFO("%d", value);                                                              // display value
  // pt.x = pt.y = -2;                                                                   // set cartesian coordinates
  // ROS_INFO("set_occupancy_wrt_origin: %d", set_occupancy_wrt_origin(map, pt, value)); // set occupancy value at location
  // ROS_INFO("get_occupancy_wrt_origin: %d", get_occupancy_wrt_origin(map, pt, value)); // get occupancy value at location
  // ROS_INFO("%d", value);                                                              // display value

  // Test set horizontal/vertical occupancy using cartesian coordinates (wrt origin)
  pt.x = atof(argv[1]); 
  pt.y = atof(argv[2]);                                                                                             // set cartesian coordinates, wrt cell(0,0)
  ROS_INFO("set_horziontal_occupancy_wrt_origin: %d", set_horziontal_occupancy_wrt_origin(map, pt, OCCUPIED)); // draw horizontal line passing through pt
  ROS_INFO("set_vertical_occupancy_wrt_origin: %d", set_vertical_occupancy_wrt_origin(map, pt, OCCUPIED));     // draw vertical line passing through pt
  // pt.x = pt.y = -2;                                                                                             // set cartesian coordinates, wrt cell(0,0)
  // ROS_INFO("set_horziontal_occupancy_wrt_origin: %d", set_horziontal_occupancy_wrt_origin(map, pt, OCCUPIED)); // draw horizontal line passing through pt
  // ROS_INFO("set_vertical_occupancy_wrt_origin: %d", set_vertical_occupancy_wrt_origin(map, pt, OCCUPIED));     // draw vertical line passing through pt

  map_pub.publish(map);
  ros::spin();

  return 0;
}