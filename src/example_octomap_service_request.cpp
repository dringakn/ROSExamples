/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <bits/stdc++.h>                    // C++ Related stuff
#include <octomap/octomap.h>                // Octree
#include <octomap_msgs/BoundingBoxQuery.h>  // Clear bbx service
#include <octomap_msgs/GetOctomap.h>        // Binary octomap service
#include <octomap_msgs/conversions.h>       // Octomap messages
#include <ros/ros.h>                        // ROS Related stuff

using namespace std;
typedef octomap::point3d Point3D;
typedef octomath::Vector3 Vector3;
typedef octomath::Quaternion Quaternion;
typedef octomath::Pose6D Pose6D;

void showOctreeStats(octomap::OcTree* ot)
{
  // Show octree stats
  Point3D bound = ot->getBBXBounds();
  cout << "Bounding Box:" << bound << endl;
  Point3D center = ot->getBBXCenter();
  cout << "Bounding Box Center:" << center << endl;
  Point3D bbxMax = ot->getBBXMax();
  cout << "Bounding Box Max:" << bbxMax << endl;
  Point3D bbxMin = ot->getBBXMin();
  cout << "Bounding Box Min:" << bbxMin << endl;
  double clampThreshMax = ot->getClampingThresMax();
  cout << "Clamping Threshold Max:" << clampThreshMax << endl;
  double clampThreshMin = ot->getClampingThresMin();
  cout << "Clamping Threshold Min:" << clampThreshMin << endl;
  // Limit: |MetricMax-MetricMin|
  double xmin, ymin, zmin, xmax, ymax, zmax;
  ot->getMetricMax(xmax, ymax, zmax);
  cout << "Metric Max:" << xmax << ',' << ymax << ',' << zmax << endl;
  ot->getMetricMin(xmin, ymin, zmin);
  cout << "Metric Min:" << xmin << ',' << ymin << ',' << zmin << endl;
  ot->getMetricSize(xmax, ymax, zmax);
  cout << "Metric Size:" << xmax << ',' << ymax << ',' << zmax << endl;
  double resolution = ot->getResolution();
  cout << "Resolution:" << resolution << endl;
  int depth = ot->getTreeDepth();
  cout << "Depth:" << depth << endl;
  double pHit = ot->getProbHit();
  cout << "pHit:" << pHit << endl;
  double pMiss = ot->getProbMiss();
  cout << "pMiss:" << pMiss << endl;
  int leafs = ot->getNumLeafNodes();
  cout << "Leafs:" << leafs << endl;
  int nodes = ot->calcNumNodes();
  cout << "Nodes:" << nodes << endl;
  double volume = ot->volume();  // length x width x height
  cout << "Volume:" << volume << endl;
  int memoryUsage = ot->memoryUsage();  // Binary octree
  cout << "Memory Usage:" << memoryUsage << endl;
  double thresh = ot->getOccupancyThres();
  int memoryUsageNode = ot->memoryUsageNode();  // Node size 16 Bytes = 2 x 8
  cout << "Memory Usage Node:" << memoryUsageNode << endl;
  int memoryFullGrid = ot->memoryFullGrid();  // Including metadata (Bytes)
  cout << "Memory Full Grid:" << memoryFullGrid << endl;
  cout << "Occupancy Threshold:" << thresh << endl;
}

int main(int argc, char* argv[])
{
  // Initialize node
  ros::init(argc, argv, "example_octomap_service_request");
  ros::NodeHandle nh("~");

  // Get octree map service topic
  // Create octree message publisher
  ros::Publisher pub;
  std::string topic;
  nh.param<std::string>("topic", topic, "/octomap_binary");
  pub = nh.advertise<octomap_msgs::Octomap>(topic, 1, true);

  // Create a service client to connect to octomap_server on "octomap_binary"
  // client.call(req,res) // actual call to the service
  // client.exists() // true if advertised and available
  // client.getService() // return the name of the service
  // client.isPersistent() // persistent service
  // client.isValid() // handle for persistent service
  // client.waitForExistence(ros::Duration(-1)) // blocked wait

  ros::ServiceClient client = nh.serviceClient<octomap_msgs::GetOctomap>(topic);

  while (!client.waitForExistence(ros::Duration(1)))
  {
    ROS_INFO("Waiting for the octomap service server...");
  }
  ROS_INFO("Octomap service server connected.");

  // Create request message (no members)
  octomap_msgs::GetOctomapRequest req;
  // Create response message
  octomap_msgs::GetOctomapResponse res;
  // call the service
  if (client.exists())
  {
    if (client.call(req, res))
    {
      // Create octree
      octomap::OcTree* ot;
      ot = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(res.map));
      showOctreeStats(ot);
    }
    else
    {
      ROS_INFO("GetMap Service call failed.");
    }
  }
  else
  {
    ROS_INFO("Service doesn't advertised and available.");
  }

  // Perodic ROS code
  ros::Rate rate(1);
  while (ros::ok())
  {
    // Yield control to ROS
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}