#include <bits/stdc++.h>                     // C++ related stuff
#include <octomap/octomap.h>                 // Octree related stuff
#include <octomap_msgs/conversions.h>        //ROS Octomap message
#include <ros/ros.h>                         // ROS related stuff
#include <tf/tf.h>                           //tf
#include <visualization_msgs/MarkerArray.h>  // Octree

using namespace std;
typedef octomap::point3d Point3D;

namespace std {
template <typename _CharT, typename _Traits>
inline basic_ostream<_CharT, _Traits>& tab(
    basic_ostream<_CharT, _Traits>& __os) {
  return __os.put(__os.widen('\t'));
}
}  // namespace std

int main(int argc, char* argv[]) {
  cout.precision(5);
  cout.setf(cout.showpos);

  // Initialize the node and create node handle
  ros::init(argc, argv, "example_octomap_create");
  ros::NodeHandle nh("~");

  // Get the parameters from server
  float resolution;
  if (!nh.param<float>("resolution", resolution, 0.1))
    nh.setParam("resolution", resolution);
  int n;
  if (!nh.param<int>("samples", n, 10)) nh.setParam("samples", n);
  int minmax;
  if (!nh.param<int>("min_max", minmax, 10)) nh.setParam("min_max", minmax);
  std::mt19937 rng;
  std::uniform_int_distribution<int> distUni(-minmax, minmax);

  // Create the ot
  // octomap::OcTree ot(resolution);
  octomap::ColorOcTree ot(resolution);
  bool occupied = true;
  ROS_INFO("Creating random ot...");
  for (int i = 0; i < n; i++) {
    Point3D pt(distUni(rng), distUni(rng), distUni(rng));
    ot.updateNode(pt, (distUni(rng) % 2 == 0));
  }
  ROS_INFO("Octree created.");
  ROS_INFO("Pruning octree for compression...");
  ot.prune();
  ROS_INFO("Pruning completed.");

  // Save the ot. To visulize: octovis /tmp/sampleOcTree.bt
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
  // Search node using point3d, key or xyz. Returns a node or null
  Point3D queryPt(0, 0, 0);
  ROS_INFO("Querying Octree ...");
  cout << queryPt << endl;
  octomap::OcTreeNode* result = ot.search(0, 0, 0);  // NULL
  if (result != NULL)
    ROS_INFO("Occupancy: %5.2f", result->getOccupancy());
  else
    ROS_INFO("Quering Octree failed.");

  // Set properties
  const int limit = 20;
  Point3D tempMax(limit, limit, limit);
  ot.setBBXMax(tempMax);
  Point3D tempMin(-limit, -limit, -limit);
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
  // Limit: |MetricMax-MetricMin|
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

  // Get octree iterators
  ROS_INFO("Octree iterator...");
  for (auto it = ot.begin(); it != ot.end(); ++it) {
    cout << it.getCoordinate() << '\t' << it.getDepth() << '\t' << '\t'
         << it.getSize() << '\t' << it.getX() << ',' << it.getY() << ','
         << it.getZ() << endl;
  }
  ROS_INFO("Octree begin_tree() iterator...");
  for (auto it = ot.begin_tree(); it != ot.end_tree(); ++it) {
    cout << it.getCoordinate() << '\t' << it.getDepth() << '\t' << '\t'
         << it.getSize() << '\t' << it.getX() << ',' << it.getY() << ','
         << it.getZ() << endl;
  }
  ROS_INFO("Octree begin_leafs() iterator...");
  for (auto it = ot.begin_leafs(); it != ot.end_leafs(); ++it) {
    cout << it.getCoordinate() << '\t' << it.getDepth() << '\t' << '\t'
         << it.getSize() << '\t' << it.getX() << ',' << it.getY() << ','
         << it.getZ() << endl;
  }
  ROS_INFO("Octree begin_leafs_bbx(min,max) iterator...");
  for (auto it = ot.begin_leafs_bbx(bbxMin, bbxMax); it != ot.end_leafs_bbx();
       ++it) {
    cout << it.getCoordinate() << '\t' << it.getDepth() << '\t' << '\t'
         << it.getSize() << '\t' << it.getX() << ',' << it.getY() << ','
         << it.getZ() << '\t' << it->getOccupancy() << '\t' << it->getValue()
         << '\t' << it->getLogOdds() << '\t' << it->getAverageChildColor()
         << endl;
  }

  // Multi-level resolution
  // VoxelLength = res × 2^(16-depth)
  ROS_INFO("Multi-level leaf nodes Centers[depth][voxelLength] -> LogOdds:");
  for (int i = 0; i <= 16; i++)
    for (auto it = ot.begin_leafs(i); it != ot.end_leafs(); ++it)
      cout << it.getCoordinate() << '[' << it.getDepth() << ']' << '['
           << it.getSize() << ']' << " -> " << it->getValue() << endl;
  // while (ros::ok) {
  //   ros::spinOnce();
  //   ros::Duration(0.1).sleep();
  // }

  // Check if the specified node is occupied:
  // ot.isNodeOccupied(ot.search(x,y,z)), watch for NULL, considering the
  // threshold parameters.
  // A node is collapsible if all children exist, don't have children of their
  // own and have the same occupancy value
  /**
   * expandNode(...)
   * Expands a node (reverse of pruning): All children are created and
   * their occupancy probability is set to the node's value.
   *
   * You need to verify that this is indeed a pruned node (i.e. not a
   * leaf at the lowest level)
   *
   */

  // OccupancyOcTree is for 3d OccupancyMapping

  // Maximum tree depth is 16 -> with res of 0.01 values have to be +/-327.68m

  // coordToKey(...) converts a coordinate to a Key
  // To access leafnode coordinates and it's logodd values (probability)
  // For multi-resolution queries use maxdepth parameters
  ROS_INFO("Accessing leaf nodes Centers[depth] -> LogOdds:");
  for (auto it = ot.begin_leafs(1); it != ot.end_leafs(); ++it) {
    cout << it.getCoordinate() << '[' << it.getDepth() << ']' << " -> "
         << it->getValue() << endl;
  }

  // Create a Pose6d (Vector3, Quaternion).
  ROS_INFO("Creating Vector, Quaternion, Pose6D");
  octomath::Vector3 v1(0, 0, 0), v2(5, 5, 5);
  cout << v1 << "\t" << v2 << endl;
  octomath::Quaternion q1(0, 0, 0), q2(M_PI_4, 0, 0);  // Tait-bryian, 1-2-3
  cout << q1 << "\t" << q2 << endl;
  octomath::Pose6D p1(v1, q1);
  octomath::Pose6D p2(5, 5, 5, M_PI_4, 0, 0);
  cout << p1 << "\t" << p2 << endl;
  cout << "p1.distance(p2): " << p1.distance(p2) << endl;
  cout << "v1.dot(v2): " << v1.dot(v2) << endl;
  cout << "v1.cross(v2): " << v1.cross(v2) << endl;
  cout << "v1.norm(): " << v1.norm() << endl;
  cout << "Angle to v2: " << v1.angleTo(v2) << endl;
  cout << "v1.distXY(v2): " << v1.distanceXY(v2) << endl;
  cout << "q1.toEuler(): " << q1.toEuler() << endl;
  vector<double> mat;
  q1.toRotMatrix(mat);
  cout << "q1.toRotMatrix(): " << mat.size() << endl;

  // Create a pointcloud. A colleciton of 3D points.
  octomap::Pointcloud scan;
  scan.push_back(5, 5, 5);
  scan.push_back(5, -5, 5);
  scan.push_back(-5, -5, 5);
  scan.push_back(-5, 5, 5);
  scan.push_back(5, 5, -5);
  scan.push_back(5, -5, -5);
  scan.push_back(-5, -5, -5);
  scan.push_back(-5, 5, -5);
  ROS_INFO("Pointcloud");
  for (auto&& pt : scan) cout << pt << endl;
  ROS_INFO("Transform Pointcloud p2");
  scan.transform(p2);
  for (auto&& pt : scan) cout << pt << endl;
  ROS_INFO("Transform Pointcloud p2^-1");
  scan.transform(p2.inv());
  for (auto&& pt : scan) cout << pt << endl;
  // scan.rotate(r,p,y);
  // scan.transformAbsolute(pose); // absolute transform
  scan.calcBBX(bbxMin, bbxMax);  // calculate bounding box
  cout << "Pointcloud BBX (min) (max):" << bbxMin << "\t" << bbxMax << endl;

  // Delete the key structure, Number of nodes in the tree
  cout << "OcTree Size, Before Delete: " << ot.size() << endl;
  ot.clear();
  cout << "OcTree Size, After Delete: " << ot.size() << endl;

  // We could use the castRay to determine if there is something in between two
  // nodes.
  // On a prebuilt map, use castRay() for localization, i.e. to check from a
  // certain position how likely is to observe a obstacle.
  // ot.castRay(origin, directon, end, ignore, maxrange);
  // computeRayXXXX(...), returns all the node traversed by the ray.
  // computeRayKeys(...) is faster compared to the computeRay(...)
  // Returns false, if one of the coordinate is out of range.
  // Instead of using updateNode(pt, occ|free) use insertScan() to update the
  // free and update nodes with occupancy values.
  // ot.insertPointCloud(scan, origin,-1,false,false);
  // ot.insertPointCloudRays(scan, origin, -1, false);
  // ot.insertRay(origin, end, -1, false);
  // insert rays emiitting from v1 to each scan point.
  ot.insertPointCloud(scan, v1, -1, false, true);
  ot.writeBinary("/tmp/sampleOcTree.bt");

  // sample scan
  octomap::Pointcloud scan2;
  scan.subSampleRandom(3, scan2);  // random subsample
  ROS_INFO("scan.subSampleRandom(3, scan2): ");
  for (auto&& pt : scan2) cout << pt << endl;
  ROS_INFO("scan2.minDist(9): ");  // remove any point lesser then thresh
  scan2.minDist(9);
  for (auto&& pt : scan2) cout << pt << endl;

  // ScanGraph is used to optimize pose graphs.
  // TODO: ScanGraph Example.
  // octomap::ScanEdge
  // octomap::ScanGraph
  // octomap::ScanNode
  /**
   * Reads in a ScanGraph from a "plain" ASCII file of the form
   * NODE x y z R P Y
   * x y z
   * x y z
   * x y z
   * NODE x y z R P Y
   * x y z
   *
   * Lines starting with the NODE keyword contain the 6D pose of a scan node,
   * all 3D point following until the next NODE keyword (or end of file) are
   * inserted into that scan node as pointcloud in its local coordinate frame
   *
   */

  // getUnknownLeafCenters(...) return centers of leafs that do NOT exist (but
  // could) in a given bounding box
  ROS_INFO("ot.getUnknownLeafCenters(list, bbxMin, bbxMax, 11): ");
  list<octomath::Vector3> list;
  // 15=124427, 14=15343, 13=1600, 12=277, 11=9, 10=0
  // 125000-x=124427
  ot.getUnknownLeafCenters(list, bbxMin, bbxMax, 11);
  for (auto&& pt : list) cout << pt << endl;
  cout << "Unknown points: " << list.size() << endl;

  // Multi-level resolution, VoxelLength = res × 2^(16-depth)
  // 10=6.4, 11=3.2, 12=1.6, 13=0.8, 14=0.4, 15=0.2, 16=0.1
  ROS_INFO("Multi-level leaf nodes Centers[depth][voxelLength] -> LogOdds:");
  for (int i = 1; i <= 11; i++)
    for (auto it = ot.begin_leafs(i); it != ot.end_leafs(); ++it)
      cout << it.getCoordinate() << '[' << it.getDepth() << ']' << '['
           << it.getSize() << ']' << " -> " << it->getValue() << endl;

  // change detection, voxels whose logodds changes
  ot.enableChangeDetection(true);
  ot.isChangeDetectionEnabled();
  ot.numChangesDetected();
  ot.resetChangeDetection();

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