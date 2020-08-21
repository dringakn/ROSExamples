#include <bits/stdc++.h>                     // C++ related stuff
#include <octomap/octomap.h>                 // Octree related stuff
#include <octomap_msgs/conversions.h>        //ROS Octomap message
#include <ros/ros.h>                         // ROS related stuff
#include <tf/tf.h>                           //tf
#include <visualization_msgs/MarkerArray.h>  // Octree

using namespace std;

int main(int argc, char* argv[]) {
  std::mt19937 rng;
  std::uniform_int_distribution<int> distUni(-10, 10);

  // Initialize the node and create node handle
  ros::init(argc, argv, "example_octomap_create");
  ros::NodeHandle nh;

  // Create the octree
  //   octomap::OcTree octree(0.1);
  octomap::ColorOcTree octree(0.1);
  bool occupied = true;
  ROS_INFO("Creating random octree...");
  int n = (argc > 1) ? atoi(argv[1]) : 1000;
  for (int i = 0; i < n; i++) {
    octomap::point3d pt(distUni(rng), distUni(rng), distUni(rng));
    octree.updateNode(pt, (distUni(rng) % 2 == 0));
  }
  ROS_INFO("Octree created.");

  // Save the octree
  ROS_INFO("Saving octomap to /tmp/sampleOcTree.bt ...");
  if (octree.writeBinary("/tmp/sampleOcTree.bt"))
    ROS_INFO("Saving done.");
  else
    ROS_INFO("Saving failed.");

  // Load the octre
  ROS_INFO("Loading octomap from /tmp/sampleOcTree.bt ...");
  if (octree.readBinary("/tmp/sampleOcTree.bt"))
    ROS_INFO("Octree successfully loaded from /tmp/sampleOcTree.bt");
  else
    ROS_INFO("Could not load octree from /tmp/sampleOcTree.bt");

  // Search a point in the tree
  octomap::point3d queryPt(0, 0, 0);
  ROS_INFO("Querying Octree ...");
  cout << queryPt << endl;
  octomap::OcTreeNode* result = octree.search(0, 0, 0);  // NULL
  if (result != NULL)
    ROS_INFO("Occupancy: %5.2f", result->getOccupancy());
  else
    ROS_INFO("Quering Octree failed.");

  // Visulize the octree, create Octomap publisher
  ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
  // Create Octomap message
  octomap_msgs::Octomap msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time(0);
  octomap_msgs::binaryMapToMsg(octree, msg);
  //   msg.id = "OcTree"; // OcTree, ColorOcTree

  // Perodic code
  ros::Rate rate(1);
  while (ros::ok()) {
    // Publish octomap
    pub.publish(msg);
    // Yield control to ROS
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}