/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <bits/stdc++.h>               // C++ Related stuff
#include <octomap/octomap.h>           // Octree
#include <octomap_msgs/conversions.h>  // Octomap messages
#include <ros/ros.h>                   // ROS Related stuff

using namespace std;
typedef octomap::point3d Point3D;
typedef octomath::Vector3 Vector3;
typedef octomath::Quaternion Quaternion;
typedef octomath::Pose6D Pose6D;

int main(int argc, char* argv[]) {
  // Initialize node
  ros::init(argc, argv, "example_octomap_load");
  ros::NodeHandle nh("~");

  // Get pre saved octree map file (with path+ext)
  std::string file;
  nh.param<std::string>(
      "map_file", file,
      "/home/office/cmu_ws/src/ros_examples/map/sample_map3.bt");

  // Create octree
  octomap::OcTree ot(file);
  // ot.setResolution(0.125);  // ???
  // ot.prune();

  // Get a list of unknown voxels at coarse resolution.
  // 0.05 * 2^(16-11) = 1.6m, visulize the difference in points steps.
  // list<Vector3> list;
  double res;
  nh.param<double>("sample_resolution", res, 1);
  int depth_n = floor(16 - (log(res / ot.getResolution()) / log(2)));
  octomap::point3d_list list;
  double xmin, ymin, zmin, xmax, ymax, zmax;
  ot.getMetricMin(xmin, ymin, zmin);
  ot.getMetricMax(xmax, ymax, zmax);
  Point3D bbxMin = Point3D(xmin, ymin, zmin);
  Point3D bbxMax = Point3D(xmax, ymax, zmax);
  ot.getUnknownLeafCenters(list, bbxMin, bbxMax, depth_n);
  for (auto&& pt : list) cout << pt << endl;
  cout << "Unknown points: " << list.size() << endl;

  // To visulize the result, let's create another tree
  octomap::OcTree uk(res);
  for (auto&& pt : list) uk.updateNode(pt, true);
  ros::Publisher p1 = nh.advertise<octomap_msgs::Octomap>("/unknown", 1, true);
  octomap_msgs::Octomap m1;
  octomap_msgs::binaryMapToMsg(uk, m1);
  m1.header.frame_id = "map";
  m1.header.stamp = ros::Time(0);
  p1.publish(m1);
  ros::spinOnce();

  // Show octree stats
  Point3D bound = ot.getBBXBounds();
  cout << "Bounding Box:" << bound << endl;
  Point3D center = ot.getBBXCenter();
  cout << "Bounding Box Center:" << center << endl;
  bbxMax = ot.getBBXMax();
  cout << "Bounding Box Max:" << bbxMax << endl;
  bbxMin = ot.getBBXMin();
  cout << "Bounding Box Min:" << bbxMin << endl;
  double clampThreshMax = ot.getClampingThresMax();
  cout << "Clamping Threshold Max:" << clampThreshMax << endl;
  double clampThreshMin = ot.getClampingThresMin();
  cout << "Clamping Threshold Min:" << clampThreshMin << endl;
  // Limit: |MetricMax-MetricMin|
  ot.getMetricMax(xmax, ymax, zmax);
  cout << "Metric Max:" << xmax << ',' << ymax << ',' << zmax << endl;
  ot.getMetricMin(xmin, ymin, zmin);
  cout << "Metric Min:" << xmin << ',' << ymin << ',' << zmin << endl;
  ot.getMetricSize(xmax, ymax, zmax);
  cout << "Metric Size:" << xmax << ',' << ymax << ',' << zmax << endl;
  double resolution = ot.getResolution();
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
  double volume = ot.volume();  // length x width x height
  cout << "Volume:" << volume << endl;
  int memoryUsage = ot.memoryUsage();  // Binary octree
  cout << "Memory Usage:" << memoryUsage << endl;
  double thresh = ot.getOccupancyThres();
  int memoryUsageNode = ot.memoryUsageNode();  // Node size 16 Bytes = 2 x 8
  cout << "Memory Usage Node:" << memoryUsageNode << endl;
  int memoryFullGrid = ot.memoryFullGrid();  // Including metadata (Bytes)
  cout << "Memory Full Grid:" << memoryFullGrid << endl;
  cout << "Occupancy Threshold:" << thresh << endl;

  // Create octree message publisher
  ros::Publisher pub;
  std::string topic;
  nh.param<std::string>("topic", topic, "/octomap");
  pub = nh.advertise<octomap_msgs::Octomap>(topic, 1, true);

  // Create octree ROS message
  std::string frame;
  nh.param<std::string>("frame", frame, "map");
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(ot, msg);
  msg.header.frame_id = frame;
  msg.header.stamp = ros::Time(0);
  pub.publish(msg);

  // Perodic ROS code
  ros::Rate rate(1);
  while (ros::ok()) {
    // Yield control to ROS
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}