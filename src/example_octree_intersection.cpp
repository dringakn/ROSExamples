/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Octree intersection example.
    Note: The octree can be visulized either by using octovis (by saving) or by
          publishing a message and visulizing it through RViz.
*/
#include <octomap/octomap.h>           // Octomap API
#include <octomap_msgs/Octomap.h>      // Octomap messages
#include <octomap_msgs/conversions.h>  // Message conversions utilities
#include <ros/ros.h>                   // ROS functionality

using namespace std;
using namespace octomap;

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t "
         << node->getOccupancy() << endl;
  } else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_octree_intersection");
  ros::NodeHandle nh;
  OcTree ot(0.1);  // Empty octree of with 0.1 m resolution
  ot.setClampingThresMin(0.01);
  ot.setClampingThresMax(0.99);
  // Add an empty cube (empty voxels)
  for (float x = -2; x < 2; x += 0.02f) {
    for (float y = -2; y < 2; y += 0.02f) {
      for (float z = -2; z < 2; z += 0.02f) {
        point3d pt(x, y, z);
        ot.updateNode(pt, false, false);
        // ot.updateNode(x, y, z, false, false);
      }
    }
  }

  // Add an inner small solid cube (occupied voxels)
  for (float x = -1; x < 1; x += 0.01f) {
    for (float y = -1; y < 1; y += 0.01f) {
      for (float z = -1; z < 1; z += 0.01f) {
        point3d pt(x, y, z);
        ot.updateNode(pt, true, false);
      }
    }
  }

  point3d origin(-1, -1, -0.5);  // ray tracing origin.
  point3d direction;             // direction vector.
  point3d endpoint;              // ray end point.

  for (float z = 0; z <= 0.25; z += 0.125) {
    direction = point3d(1, 1, z);
    cout << endl;
    cout << "casting ray from " << origin << " in the " << direction
         << " direction" << endl;
    bool success = ot.castRay(origin, direction, endpoint);

    if (success) {
      cout << "ray hit cell with center " << endpoint << endl;

      point3d intersection;
      success =
          ot.getRayIntersection(origin, direction, endpoint, intersection);
      if (success) cout << "entrance point is " << intersection << endl;
    }
  }

  ros::Publisher pub;
  pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1, true);
  octomap_msgs::Octomap msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  //   msg.resolution = 0.1; // Automatically filled
  //   msg.id = "OcTree";  // OcTree, ColorOcTree, Automatically filled

  if (!octomap_msgs::binaryMapToMsg<OcTree>(ot, msg))
    ROS_INFO("Error generating message from the tree.");
  pub.publish(msg);

  point3d query(0., 0., 0.);
  OcTreeNode* result = ot.search(query);
  print_query_info(query, result);

  query = point3d(-1., -1., -1.);
  result = ot.search(query);
  print_query_info(query, result);

  query = point3d(1., 1., 1.);
  result = ot.search(query);
  print_query_info(query, result);

  ros::spin();
  return 0;
}