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
  ros::NodeHandle nh;

  // Get pre saved octree map file (with path+ext)
  std::string file;
  if (argc > 1) {
    file = std::string(argv[1]);
  } else {
    file =
        std::string("/home/office/cmu_ws/src/ros_examples/map/sample_map2.bt");
  }

  // Create octree
  octomap::OcTree ot(file);
  // ot.setResolution(0.125);  // ???

  // Show octree stats
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
  // Limit: |MetricMax-MetricMin|
  double x, y, z;
  ot.getMetricMax(x, y, z);
  cout << "Metric Max:" << x << ',' << y << ',' << z << endl;
  ot.getMetricMin(x, y, z);
  cout << "Metric Min:" << x << ',' << y << ',' << z << endl;
  ot.getMetricSize(x, y, z);
  cout << "Metric Size:" << x << ',' << y << ',' << z << endl;
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
  pub = nh.advertise<octomap_msgs::Octomap>((argc > 2) ? argv[2] : "/octomap",
                                            1, true);

  // Create octree ROS message
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(ot, msg);
  msg.header.frame_id = (argc > 3) ? argv[3] : "map";
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