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

/**
 * @brief Nearest point in the list in term of Euclidean distance. It assumes
 * at-least a single point is in the list. If the minimum distance is not
 * specified the maximum is assumed (1e+37).
 *
 * @param vertices List of 3D Points
 * @param pt Search point
 * @param min Minimum distance for search
 * @return Point3D
 */
Point3D nearestNeighbor(vector<Point3D>& vertices, Point3D& pt,
                        float min = FLT_MAX) {
  Point3D res;
  float dist;
  for (auto&& v : vertices) {
    dist = pcl::euclideanDistance(v, pt);
    if (dist <= FLT_EPSILON)  // 1e-5
      return v;
    else if (dist < min) {
      dist = min;
      res = v;
    }
  }
  return res;
}

/**
 * @brief Get a new point considering tree growth parameter.
 *
 * @param ptNear Nearest Point
 * @param ptSample Sample Point
 * @param eta Growth rate parameter [0,1]
 * @return Point3D
 */
Point3D Steer(Point3D& ptNear, Point3D& ptSample, float eta = 0.5) {
  if (pcl::euclideanDistance(ptNear, ptSample) <= eta) {
    return ptSample;
  } else {
    float dx = ptSample.x - ptNear.x;
    float dy = ptSample.y - ptNear.y;
    Point3D res;
    if (dx == 0) {
      res.x = ptNear.x;
      res.y = ptNear.y + eta;
    } else {
      float m = dy / dx;
      res.x = ((dx) ? 1 : -1) * sqrt(pow(eta, 2) / (1 + pow(m, 2))) + ptNear.x;
      res.y = m * (res.x - ptNear.x) + ptNear.y;
    }
    return res;
  }
}

/**
 * @brief
 *
 * @param ptNear
 * @param ptSteer
 * @param cloud
 * @return int
 */
// int obstacleFree(Point3D& ptNear, Point3D& ptSteer, PointCloud::Ptr cloud) {
//   Point3D xi = ptNear;
//   float rez = float(mapsub.info.resolution) * .2;
//   float stepz = int(ceil(Norm(ptSteer, ptNear)) / rez);
//   int obstacle = 0, unknown = 0, result = 0;

//   for (int c = 0; c < stepz; c++) {
//     xi = Steer(xi, ptSteer, rez);
//     if (gridValue(mapsub, xi) == 100) obstacle = 1;
//     if (gridValue(mapsub, xi) == -1) {
//       unknown = 1;
//       break;
//     }
//   }

//   ptSteer = xi;
//   if (unknown == 1) result = -1;
//   if (obstacle == 1) result = 0;
//   if (obstacle != 1 && unknown != 1) result = 1;
//   return result;
// }

double entropy(const vector<int>& vec) {
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
  cout.precision(5);
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

  // Create KDTree from OGM
  pcl::KdTreeFLANN<Point3D> kdtree;
  PointCloud::Ptr cloud(new PointCloud());
  Point3D pt;
  pt.z = 0;  // Same z for the OGM
  for (int y = 0; y < mapData.info.height; y++) {
    for (int x = 0; x < mapData.info.width; x++) {
      pt.intensity = mapData.data[y * mapData.info.width + x];
      // cout << x << ',' << y << ',' << pt.intensity << endl;
      // intensity: -1=Unknown , 0=Free, 100=Obstacle
      // Total:   561659
      // Unknown: 413348 (73.59%)
      // Free:    137739 (24.52%)
      // Obstacle:10572  (01.88%)
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
  // cout << cloud->height << endl << cloud->width << endl;
  kdtree.setInputCloud(cloud);
  //   sensor_msgs::PointCloud2 msg;
  //   pcl_conversions::fromPCL(*cloud, msg);
  //   msg.header.stamp = ros::Time::now();
  //   msg.header.frame_id = "/map";
  //   pubPC.publish(*cloud);
  //   ros::spinOnce();
  pcl::visualization::CloudViewer viewer(std::string("Simple Cloud Viewer"));
  // viewer.showCloud(cloud);

  // Entropy filtering on the point cloud.
  PointCloud::Ptr filtered(new PointCloud());
  vector<int> indices;
  vector<float> sqr_dist;
  vector<int> data;
  for (auto&& pt : cloud->points) {
    indices.clear();
    sqr_dist.clear();
    if (kdtree.radiusSearch(pt, 0.25, indices, sqr_dist) > 0) {
      data.clear();
      for (int i = 0; i < indices.size(); i++)
        data.push_back((int)(cloud->points[indices[i]].intensity));
      double inf = entropy(data);
      if (inf > 0.1) {
        filtered->points.push_back(pt);
        // cout << inf << '\t' << data.size() << endl;
      }
    }
  }
  filtered->height = 1;
  filtered->width = filtered->points.size();
  filtered->is_dense = false;
  filtered->header.frame_id = cloud->header.frame_id;
  viewer.showCloud(filtered);

  // vector<int> sample(cloud->points.size());
  // for (int i = 0; i < cloud->points.size(); i++) sample[i] = i;
  // shuffle(sample.begin(), sample.end(), rng);
  // copy(sample.begin(), sample.end(), ostream_iterator<int>(cout, "\n"));

  // vector<Point3D> frontiers, nodes;
  // // vector.back() return the last element in the vector.
  // // vector.popback() removes the last element in the vector and decrease
  // size.
  // // vector.erase(pos|range) removes the element or range and update size.

  // Check for the initialization
  if (init_exploration > 0) {
    //   // Get a random free cell as starting point and remove it from sample
    //   list for (int idx = 0; idx < sample.size(); ++idx) {
    //     if (cloud->points[sample[idx]].intensity == 0) {  // Free space
    //       nodes.push_back(cloud->points[sample[idx]]);  // Add it to the node
    //       list sample.erase(sample.begin() + idx);  // Remove the sample
    //       number break;                               // Exit for loop
    //     }
    //   }
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
    if (viewer.wasStopped(50))  // Check if the window is closed
      ros::shutdown();          // Shutdown the node

    // int s = sample.back();                // Get a random sample
    // Point3D ptSample = cloud->points[s];  // Sample point
    // Point3D ptNear;                       // Nearest point in nodes list
    // ptNear = nearestNeighbor(nodes, ptSample);
    // Point3D ptSteer;  // New point considering tree growth rate
    // ptSteer = Steer(ptNear, ptSample, 0.5);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}