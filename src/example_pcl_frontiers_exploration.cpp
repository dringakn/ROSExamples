#include <bits/stdc++.h>                      // Standard C++
#include <geometry_msgs/PointStamped.h>       // Geometry point message
#include <nav_msgs/OccupancyGrid.h>           // OGM
#include <pcl/common/distances.h>             // Euclidean distance
#include <pcl/kdtree/kdtree_flann.h>          // KDTree
#include <pcl/point_cloud.h>                  // PCL
#include <pcl/visualization/cloud_viewer.h>   // PCL Cloud viewer
#include <pcl_conversions/pcl_conversions.h>  // to/from Msg
#include <ros/ros.h>                          // ROS
#include <sensor_msgs/PointCloud2.h>          // Point Cloud
#include <visualization_msgs/Marker.h>        // RViz Marker

using namespace std;
typedef pcl::PointXYZI Point3D;
typedef pcl::PointCloud<Point3D> PointCloud;

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
  // Update the RNG domain: 721x779 = 561659
  distUni = std::uniform_int_distribution<int>(
      0, mapData.info.height * mapData.info.width);
  mapRecieved = true;
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) {}

double calculateEntropy(const vector<int>& vec) {
  unordered_map<int, unsigned> counts;

  for (uint32_t value : vec) {
    ++counts[value];
  }

  double sum = 0.0;
  const double n = vec.size();

  // Entropy: -1 * Sum ( p(x) * log(p(x)) )
  for (auto it = counts.begin(); it != counts.end(); ++it) {
    double p = it->second / n;
    sum += p * log2(p);
  }

  return -1.0 * sum;
}

int main(int argc, char* argv[]) {
  // Initialize the node
  ros::init(argc, argv, "example_pcl_frontiers_exploration");
  ros::NodeHandle nh;

  // Node parameters
  float radius, entropy, marker_size, rate_hz;
  std::string map_topic;
  std::string ns = ros::this_node::getName();
  ros::param::param<std::string>(ns + "/map_topic", map_topic, "/map");
  ros::param::param<float>(ns + "/radius", radius, 0.25);
  ros::param::param<float>(ns + "/entropy", entropy, 0.1);
  ros::param::param<float>(ns + "/marker_size", marker_size, 1);
  ros::param::param<float>(ns + "/rate_hz", rate_hz, 10);

  // Initialize the subscribers
  ros::Subscriber subMap = nh.subscribe(map_topic, 1, mapCallBack);
  ros::Subscriber subRViz = nh.subscribe("/clicked_point", 5, rvizCallBack);

  // Initialize the publishers
  ros::Publisher pubFrontier =  // Newly detected frontier
      nh.advertise<geometry_msgs::PointStamped>(ns + "/frontier", 1);
  ros::Publisher pubRRT = nh.advertise<visualization_msgs::Marker>(
      ns + "/frontier_markers", 1, true);
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

  // Create Point Cloud from OGM
  ROS_INFO("Creating Point Cloud...");
  pcl::KdTreeFLANN<Point3D> kdtree;
  PointCloud::Ptr cloud(new PointCloud());
  Point3D pt;
  pt.z = 0;  // Same z for the OGM
  for (int y = 0; y < mapData.info.height; y++) {
    for (int x = 0; x < mapData.info.width; x++) {
      pt.intensity = mapData.data[y * mapData.info.width + x];
      // intensity: -1=Unknown , 0=Free, 100=Obstacle
      if (pt.intensity != 100) {
        pt.x = x * mapData.info.resolution;
        pt.y = y * mapData.info.resolution;
        cloud->points.push_back(pt);
      }
    }
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
  cloud->header.frame_id = mapData.header.frame_id;
  ROS_INFO("Point cloud constructed.");

  // Create KDTree
  ROS_INFO("Creating KDTree...");
  kdtree.setInputCloud(cloud);
  ROS_INFO("KDTree Created.");
  pcl::visualization::CloudViewer viewerPC("GridMap");
  viewerPC.showCloud(cloud);

  // Entropy filtering on the point cloud.
  ROS_INFO("Filtering Frontiers...");
  PointCloud::Ptr filtered(new PointCloud());
  vector<int> indices;
  vector<float> sqr_dist;
  vector<int> data;
  double sum = 0;
  for (auto&& pt : cloud->points) {
    indices.clear();
    sqr_dist.clear();
    if (kdtree.radiusSearch(pt, radius, indices, sqr_dist) > 0) {
      data.clear();
      for (int i = 0; i < indices.size(); i++)
        data.push_back((int)(cloud->points[indices[i]].intensity));

      //   const double n = data.size();
      //   int t1 = -1, t2 = 0;
      //   double c1 = count(data.begin(), data.end(), t1) / n;
      //   double c2 = count(data.begin(), data.end(), t2) / n;
      //   double inf = -1 * (c1 * log2(c1) + c2 * log2(c2));
      double inf = calculateEntropy(data);

      if (inf > entropy && pt.intensity == -1) {
        filtered->points.push_back(pt);
        sum += inf;
      }
    }
  }
  sum /= filtered->points.size();  // calculate it before further processing

  // Remove the free space
  //   for (auto it = filtered->points.begin(); it != filtered->points.end();
  //   it++) {
  //     if (it->intensity == 0) {
  //       // Notice the iterator is decremented before erase
  //       filtered->points.erase(it--);
  //     }
  //   }

  filtered->height = 1;
  filtered->width = filtered->points.size();
  filtered->is_dense = false;
  filtered->header.frame_id = cloud->header.frame_id;
  ROS_INFO("Frontiers Filtered.");
  ROS_INFO("Average Entropy: %5.3f", sum);
  ROS_INFO("Frontiers: %5d", filtered->points.size());

  // View output
  viewerPC.showCloud(filtered);

  // Start processing
  while (ros::ok()) {
    if (viewerPC.wasStopped(100))  // Check if the window(s) is closed
      ros::shutdown();             // Shutdown the node

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}