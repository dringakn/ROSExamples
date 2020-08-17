#include <bits/stdc++.h>                      // Standard C++
#include <geometry_msgs/PointStamped.h>       // Geometry point message
#include <nav_msgs/OccupancyGrid.h>           // OGM
#include <pcl/kdtree/kdtree_flann.h>          // KDTree
#include <pcl/point_cloud.h>                  // PCL
#include <pcl/visualization/cloud_viewer.h>   // PCL Cloud viewer
#include <pcl_conversions/pcl_conversions.h>  // to/from Msg
#include <ros/ros.h>                          // ROS
#include <sensor_msgs/PointCloud2.h>          // Point Cloud
#include <visualization_msgs/Marker.h>        // RViz Marker

using namespace std;
typedef pcl::PointXY Point2D;
typedef pcl::PointXYZ Point3D;

bool mapRecieved = false;                    // OGM recieved
nav_msgs::OccupancyGrid mapData;             // OGM
visualization_msgs::Marker explorationROI;   // Four points polygon as ROI
std::mt19937 rng(0);                         // Random Number Generator
std::uniform_int_distribution<int> distUni;  // Update the min/max on map cb

/**
 * @brief Occupancy Grid Map (OGM) topic callback.
 *
 * @param msg Pointer to the OGM.
 */
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  mapData = *msg;
  // Update the RNG domain
  distUni = std::uniform_int_distribution<int>(
      0, mapData.info.height * mapData.info.width);
  mapRecieved = true;
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) {}

int main(int argc, char* argv[]) {
  // Initialize the node
  ros::init(argc, argv, "example_rrt_exploration2d");
  ros::NodeHandle nh;

  // Node parameters
  float eta, marker_size, init_exploration, rate_hz;
  std::string map_topic;
  std::string ns = ros::this_node::getName();
  ros::param::param<std::string>(ns + "/map_topic", map_topic, "/map");
  ros::param::param<float>(ns + "/eta", eta, 0.5);
  ros::param::param<float>(ns + "/marker_size", marker_size, 1);
  ros::param::param<float>(ns + "/init_exploration", init_exploration, 0);
  ros::param::param<float>(ns + "/rate_hz", rate_hz, 10);

  // Initialize the subscribers
  ros::Subscriber subMap = nh.subscribe(map_topic, 1, mapCallBack);
  ros::Subscriber subRViz = nh.subscribe("/clicked_point", 5, rvizCallBack);

  // Initialize the publishers
  ros::Publisher pubFrontier =  // Newly detected frontier
      nh.advertise<geometry_msgs::PointStamped>(ns + "/frontier", 1);
  ros::Publisher pubRRT =
      nh.advertise<visualization_msgs::Marker>(ns + "/rrt", 1, true);
  ros::Publisher pubPC = nh.advertise<pcl::PCLPointCloud2>("/pc", 1);

  // Set the update rate
  ros::Rate rate(rate_hz);

  // Wait for the map to be available
  ROS_INFO("Waiting for map...");
  while (!mapRecieved) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Map received.");
  ros::spinOnce();

  // Create KDTree
  pcl::KdTreeFLANN<Point3D> kdtree;
  pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>());
  cloud->width = mapData.info.width;
  cloud->height = mapData.info.height;
  cloud->is_dense = true;
  // cloud->points.resize(cloud->width * cloud->height);
  for (int i = 0; i < mapData.info.width; i++) {
    for (int j = 0; j < mapData.info.height; j++) {
      Point3D pt;
      //   pt.x = mapData.data[i * mapData.info.width + j];
      pt.x = i * mapData.info.resolution;
      pt.y = j * mapData.info.resolution;
      cloud->points.push_back(pt);
    }
  }
  //   sensor_msgs::PointCloud2 msg;
  //   pcl_conversions::fromPCL(*cloud, msg);
  //   msg.header.stamp = ros::Time::now();
  //   msg.header.frame_id = "/map";
  //   pubPC.publish(*cloud);
  //   ros::spinOnce();
  pcl::visualization::CloudViewer viewer(std::string("Simple Cloud Viewer"));
  viewer.showCloud(cloud);

  // Check for the initialization
  if (init_exploration > 0) {
  }

  ROS_INFO("Waiting for start location ...");
  while (init_exploration < 0) {
    // pubRRT.publish(explorationROI);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Start location recieved.");

  // Start processing
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}