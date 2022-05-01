/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: 
      Save a region of the interest specified by the RViz.
    Usage: 
      roslaunch ros_examples example_occupancy_grid_map_roi.launch
    Note: 
      modify find_packages(... geometry_msgs nav_msgs ...)
*/

#include <costmap_2d/costmap_2d.h>       // Costmap
#include <geometry_msgs/Point.h>         // Point
#include <geometry_msgs/PointStamped.h>  // RViz clicked_point
#include <nav_msgs/OccupancyGrid.h>      // OGM
#include <ros/ros.h>                     // ROS related stuff

using namespace std;
using namespace geometry_msgs;

costmap_2d::Costmap2D costmap;  // Costmap
vector<Point> roi;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  costmap.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x,
                    msg->info.origin.position.y);
  copy(msg->data.begin(), msg->data.end(), costmap.getCharMap());

  printf("Origin(%3.1f,%3.1f) SizeCells(%d,%d)->SizeMeters(%3.1f,%3.1f)\r\n", costmap.getOriginX(),
         costmap.getOriginY(), costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), costmap.getSizeInMetersX(),
         costmap.getSizeInMetersY());
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // Get the occupancy value of the current cell
  if (costmap.getCharMap() != nullptr)
  {
    // Clear the roi for Re-initialization
    if (roi.size() >= 4)
    {
      roi.clear();
    }

    // Add the current point to the roi
    roi.push_back(msg->point);
    unsigned int mx, my;
    costmap.worldToMap(msg->point.x, msg->point.y, mx, my);
    ROS_DEBUG("(%3.1f, %3.1f)->(%d,%d): %d \t Points: %d", msg->point.x, msg->point.y, mx, my, costmap.getCost(mx, my),
              roi.size());

    // Update the ROI when four points are recieved.
    if (roi.size() == 4)
    {
      // Calculate the bounding box of the first four points
      double minx = DBL_MAX, miny = DBL_MAX, maxx = DBL_MIN, maxy = DBL_MIN;
      for (int i = 0; i < roi.size(); i++)
      {
        if (roi[i].x < minx)
          minx = roi[i].x;
        if (roi[i].x > maxx)
          maxx = roi[i].x;
        if (roi[i].y < miny)
          miny = roi[i].y;
        if (roi[i].y > maxy)
          maxy = roi[i].y;
      }
      double width = maxx - minx;
      double height = maxy - miny;
      double orig_x = minx;  // Lower left corner
      double orig_y = miny;
      ROS_DEBUG("orig_x: %3.1f orig_y: %3.1f width: %3.1f height: %3.1f", orig_x, orig_y, width, height);
      costmap_2d::Costmap2D win;
      if (win.copyCostmapWindow(costmap, orig_x, orig_y, width, height))
      {
        if (win.saveMap("/home/office/delme/roi.pgm"))
          ROS_DEBUG("Successfully saved map.");
      }
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_occupancy_grid_map_roi");
  ros::NodeHandle nh;

  //* Set the log level of the node to display messages
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();
  //*/

  ros::Subscriber subMap = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
  ros::Subscriber subRViz = nh.subscribe("/clicked_point", 1, rvizCallBack);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}