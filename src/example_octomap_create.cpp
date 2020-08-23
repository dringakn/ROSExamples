#include <bits/stdc++.h>                     // C++ related stuff
#include <octomap/octomap.h>                 // Octree related stuff
#include <octomap_msgs/conversions.h>        //ROS Octomap message
#include <ros/ros.h>                         // ROS related stuff
#include <tf/tf.h>                           //tf
#include <visualization_msgs/MarkerArray.h>  // Octree

using namespace std;
typedef octomap::point3d Point3D;

int main(int argc, char* argv[]) {
  // Initialize the node and create node handle
  ros::init(argc, argv, "example_octomap_create");
  ros::NodeHandle nh;

  // Get the parameters from server
  float resolution;
  if (!nh.param<float>("resolution", resolution, 0.1))
    nh.setParam("resolution", 0.1);
  int n;
  if (!nh.param<int>("samples", n, 1000)) nh.setParam("samples", 1000);
  int minmax;
  if (!nh.param<int>("min_max", minmax, 10)) nh.setParam("min_max", 10);
  std::mt19937 rng;
  std::uniform_int_distribution<int> distUni(-minmax, minmax);

  // Create the ot
  //   octomap::OcTree ot(resolution);
  octomap::ColorOcTree ot(resolution);
  bool occupied = true;
  ROS_INFO("Creating random ot...");
  for (int i = 0; i < n; i++) {
    Point3D pt(distUni(rng), distUni(rng), distUni(rng));
    ot.updateNode(pt, (distUni(rng) % 2 == 0));
  }
  ROS_INFO("Octree created.");

  // Save the ot
  ROS_INFO("Saving octomap to /tmp/sampleOcTree.bt ...");
  if (ot.writeBinary("/tmp/sampleOcTree.bt"))
    ROS_INFO("Saving done.");
  else
    ROS_INFO("Saving failed.");

  // Load the octre
  ROS_INFO("Loading octomap from /tmp/sampleOcTree.bt ...");
  if (ot.readBinary("/tmp/sampleOcTree.bt"))
    ROS_INFO("Octree successfully loaded from /tmp/sampleOcTree.bt");
  else
    ROS_INFO("Could not load ot from /tmp/sampleOcTree.bt");

  // Search a point in the tree
  Point3D queryPt(0, 0, 0);
  ROS_INFO("Querying Octree ...");
  cout << queryPt << endl;
  octomap::OcTreeNode* result = ot.search(0, 0, 0);  // NULL
  if (result != NULL)
    ROS_INFO("Occupancy: %5.2f", result->getOccupancy());
  else
    ROS_INFO("Quering Octree failed.");

  // Set properties
  Point3D tempMax(50, 50, 50);
  ot.setBBXMax(tempMax);
  Point3D tempMin(-50, -50, -50);
  ot.setBBXMin(tempMin);

  // Octree stats
  Point3D bound = ot.getBBXBounds();
  cout << "Bounding Box:" << bound << endl;
  Point3D center = ot.getBBXCenter();
  cout << "Bounding Box Center:" << center << endl;
  Point3D bbxMax = ot.getBBXMax();
  cout << "Bounding Box Max:" << bbxMax << endl;
  Point3D bbxMin = ot.getBBXMin();
  cout << "Bounding Box Min:" << bbxMin << endl;
  double clampThreshMax = ot.getClampingThresMax();
  cout << "Clamping Threshold Max:" << clampThreshMax << endl;
  double clampThreshMin = ot.getClampingThresMin();
  cout << "Clamping Threshold Min:" << clampThreshMin << endl;
  double x, y, z;
  ot.getMetricMax(x, y, z);
  cout << "Metric Max:" << x << ',' << y << ',' << z << endl;
  ot.getMetricMin(x, y, z);
  cout << "Metric Min:" << x << ',' << y << ',' << z << endl;
  resolution = ot.getResolution();
  cout << "Resolution:" << resolution << endl;
  int depth = ot.getTreeDepth();
  cout << "Depth:" << depth << endl;
  double pHit = ot.getProbHit();
  cout << "pHit:" << pHit << endl;
  double pMiss = ot.getProbMiss();
  cout << "pMiss:" << pMiss << endl;
  int leafs = ot.getNumLeafNodes();
  cout << "Leafs:" << leafs << endl;
  int nodes = ot.calcNumNodes();
  cout << "Nodes:" << nodes << endl;
  double volume = ot.volume();
  cout << "Volume:" << volume << endl;
  int memoryUsage = ot.memoryUsage();
  cout << "Memory Usage:" << memoryUsage << endl;
  double thresh = ot.getOccupancyThres();
  int memoryUsageNode = ot.memoryUsageNode();
  cout << "Memory Usage Node:" << memoryUsageNode << endl;
  int memoryFullGrid = ot.memoryFullGrid();
  cout << "Memory Full Grid:" << memoryFullGrid << endl;
  cout << "Occupancy Threshold:" << thresh << endl;
  Point3D direction(x, y, z), termVoxel, intersection;
  cout << "Origin:" << center << endl;
  cout << "Direction:" << direction << endl;
  // castRay, computeRay, insertRay, getRayIntersection, computeRayKeys,
  // insertPointCloudRays
  if (ot.getRayIntersection(center, direction, termVoxel, intersection))
    cout << "Termination Voxel: " << termVoxel << endl
         << "Intersection:" << intersection << endl;
  else
    cout << "Ray intersection failed." << endl;

  // Visulize the ot, create Octomap publisher
  ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
  // Create Octomap message
  octomap_msgs::Octomap msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time(0);
  octomap_msgs::binaryMapToMsg(ot, msg);
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