/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:

    Notes:
        The constructor of the costmap has been changed.
        Pass the tf2_buffer object instead of the listener object.
        Log: W26

*/

#include <costmap_2d/cost_values.h>           // Definations of LETHAL, FREE, UNKNOWN
#include <costmap_2d/costmap_2d.h>            // ROS costmap_2d, copyMapRegion
#include <costmap_2d/costmap_2d_publisher.h>  // Costmap publisher node
#include <costmap_2d/costmap_2d_ros.h>        // Costmap2dROS (complete handling)
#include <costmap_2d/costmap_math.h>          // distanceToLine(), intersects
#include <costmap_2d/footprint.h>             // Footprint math
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/static_layer.h>    // Static layer
#include <costmap_2d/testing_helper.h>  // Printing, counting, copy, layers
// #include <geometry_msgs/Pose.h>         // Map pose
// #include <nav_msgs/GetMap.h>            // Map Service
// #include <nav_msgs/OccupancyGrid.h>     // Map message
// #include <ros/ros.h>                    // ROS related stuff
#include <tf2_ros/buffer.h>              // TF2 Buffer
#include <tf2_ros/transform_listener.h>  // TF2 Listener

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_costmap_2d_basic");
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tfBuffer(ros::Duration(10));        // TF buffer object
  tf2_ros::TransformListener li(tfBuffer, nh, true);  // Subscribe /tf to fill tf buffer

  // Create a 2d costma object
  costmap_2d::Costmap2D m1(10, 10, 0.25, 0, 0, costmap_2d::FREE_SPACE);

  // Create a ROS 2D Costmap object with layers, footprints, tf and configurations
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("cm", tfBuffer);  // costmap object
  costmap->start();  // By default, it is in started mode.

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    costmap->updateMap();

    // Get the robot base frame name
    cout << "costmap->getBaseFrameID(): " << costmap->getBaseFrameID() << endl;
    // Get the global (map) frame name
    cout << "costmap->getGlobalFrameID(): " << costmap->getGlobalFrameID() << endl;
    // Get the namespace of the current costmap
    cout << "costmap->getName(): " << costmap->getName() << endl;
    // Get the list of pointers for each costmap layer
    costmap->getLayeredCostmap()->getPlugins();
    // Get the pointer to the master layer which recieve update from each other layer
    cout << costmap->getCostmap()->getSizeInCellsX() << ", " << costmap->getCostmap()->getSizeInCellsY() << endl;
    // Get the pointer to the master costmap
    costmap_2d::Costmap2D* cm = costmap->getCostmap();
    // The default value for grid cell
    cout << "cm->getDefaultValue(): " << to_string(cm->getDefaultValue()) << endl;
    // Get the origin associated with the map
    cout << "cm->getOriginX(): " << cm->getOriginX() << endl;
    cout << "cm->getOriginY(): " << cm->getOriginY() << endl;
    cout << "cm->getResolution(): " << cm->getResolution() << endl;
    cout << "cm->getSizeInCellsX(): " << cm->getSizeInCellsX() << endl;
    cout << "cm->getSizeInCellsY(): " << cm->getSizeInCellsY() << endl;
    cout << "cm->getSizeInMetersX(): " << cm->getSizeInMetersX() << endl;
    cout << "cm->getSizeInMetersY(): " << cm->getSizeInMetersY() << endl;

    // dest.copyCostmapWindow(src, x,y, width,height)
    // Get a portion of the costmap specified by the window (meters)
    cout << "m1.copyCostmapWindow(*cm, 0, 0, 10, 10): " << m1.copyCostmapWindow(*cm, 0, 0, 10, 10) << endl;
    // Save the result for visulization
    cout << "m1.saveMap(/home/office/delme/temp_map_m1.pgm): " << m1.saveMap("/home/office/delme/temp_map_m1.pgm")
         << endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}