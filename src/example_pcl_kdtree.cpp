/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <bits/stdc++.h>              // Standard C++
#include <pcl/common/distances.h>     // Distances
#include <pcl/kdtree/kdtree_flann.h>  // KDTree
#include <pcl/point_cloud.h>          //PCL
#include <ros/ros.h>                  // ROS

using namespace std;
typedef pcl::PointXYZ Point3D;

void linearSearchNN(pcl::PointCloud<Point3D>::Ptr cloud, Point3D& key) {
  double dist, min = DBL_MAX;
  Point3D res;

  for (auto&& pt : cloud->points) {
    dist = pcl::euclideanDistance(key, pt);
    if (dist < min) {
      min = dist;
      res = pt;
    }
  }
  cout << "linearSearchNN:" << min << "->" << res << endl;
}

int main(int argc, char* argv[]) {
  //   ros::init(argc, argv, "example_pcl_kdtree");
  //   ros::NodeHandle nh;

  std::cout.precision(5);

  // Profiling
  auto start = chrono::high_resolution_clock::now();
  auto stop = chrono::high_resolution_clock::now();

  // Random Number Generator
  std::mt19937 rng(0);
  std::uniform_int_distribution<int> distUni(-1e6, 1e6);

  // Create Point cloud
  pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>());
  cloud->height = 1;
  cloud->width = 1e7;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate random points
  start = chrono::high_resolution_clock::now();
  for (auto&& pt : cloud->points) {
    pt.x = distUni(rng);
    pt.y = distUni(rng);
    pt.z = distUni(rng);
  }
  stop = chrono::high_resolution_clock::now();
  cout << "Random Points Generation Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Create a KDTree
  start = chrono::high_resolution_clock::now();
  pcl::KdTreeFLANN<Point3D> tree(true);
  tree.setInputCloud(cloud);
  stop = chrono::high_resolution_clock::now();
  cout << "KDTree Creation Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Create a random search point
  Point3D pt;
  pt.x = distUni(rng);
  pt.y = distUni(rng);
  pt.z = distUni(rng);
  cout << "pt:" << pt << endl;

  // Nearest Neighbour Search
  start = chrono::high_resolution_clock::now();
  int K = 10;
  vector<int> idx(K);     // Index of nearest neighbours
  vector<float> dist(K);  // Distance of nearest neigbours
  int found = tree.nearestKSearch(pt, K, idx, dist);
  if (found > 0) {
    for (int i = 0; i < idx.size(); i++) {
      cout << i << ":" << idx[i] << '\t' << sqrt(dist[i]) << '\t'
           << cloud->points[idx[i]] << endl;
    }
  }
  stop = chrono::high_resolution_clock::now();
  cout << "KDTree nearestKSearch Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  linearSearchNN(cloud, pt);
  stop = chrono::high_resolution_clock::now();
  cout << "linearSearcnNN Elapsed Time: "
       << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  return 0;
}