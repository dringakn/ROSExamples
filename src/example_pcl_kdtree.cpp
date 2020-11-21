/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
      Compare the performance (time) of the KDTree and Linear search in 3D space.
      Ten millions points are uniformly generated between +-1e-6 using Marseinne
      twister random number generator.
      KDTree is very fast interm of search but slow interm of building the tree.
      While the linear search is vice versa.
*/
#include <bits/stdc++.h>              // Standard C++
#include <pcl/common/distances.h>     // Distances
#include <pcl/kdtree/kdtree_flann.h>  // KDTree
#include <pcl/point_cloud.h>          //PCL
#include <ros/ros.h>                  // ROS

using namespace std;
typedef pcl::PointXYZ Point3D;  // PCL 3D Point

void linearSearchNN(pcl::PointCloud<Point3D>::Ptr cloud, Point3D& key)
{
  double dist, min = DBL_MAX;
  Point3D res;

  for (auto&& pt : cloud->points)
  {
    dist = pcl::euclideanDistance(key, pt);
    if (dist < min)
    {
      min = dist;
      res = pt;
    }
  }
  cout << "linearSearchNN:" << min << "->" << res << endl;
}

int main(int argc, char* argv[])
{
  //   ros::init(argc, argv, "example_pcl_kdtree");
  //   ros::NodeHandle nh;

  std::cout.precision(5);

  // C++ standard library for time profiling
  auto start = chrono::high_resolution_clock::now();
  auto stop = chrono::high_resolution_clock::now();

  // Mersenne twister random number generator with uniform distributation
  std::mt19937 mtrng(0);                                  // MT RNG
  std::uniform_int_distribution<int> distUni(-1e6, 1e6);  // UNI-Distributation with min/max

  // Create Point cloud (unstructured) with ten millions points
  pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>());
  start = chrono::high_resolution_clock::now();
  cloud->height = 1;
  cloud->width = 1e7;
  cloud->points.resize(cloud->width * cloud->height);
  for (auto&& pt : cloud->points)
  {
    // Generate uniformly distributed random points
    pt.x = distUni(mtrng);
    pt.y = distUni(mtrng);
    pt.z = distUni(mtrng);
  }
  stop = chrono::high_resolution_clock::now();
  cout << "Number of points: " << cloud->points.size() << endl;
  cout << "UniDist(" << distUni.min() << "," << distUni.max() << "), RNG:MersenneTwister19937" << endl;
  cout << "Random Points Generation Elapsed Time: " << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Create a KDTree
  start = chrono::high_resolution_clock::now();
  pcl::KdTreeFLANN<Point3D> tree(true);
  tree.setInputCloud(cloud);
  stop = chrono::high_resolution_clock::now();
  cout << "KDTree Creation Elapsed Time: " << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  // Create a random search query point
  Point3D pt;
  pt.x = distUni(mtrng);
  pt.y = distUni(mtrng);
  pt.z = distUni(mtrng);
  cout << "Query Point:" << pt << endl;

  // Nearest Neighbour Search
  int K = 10;
  cout << "Number of nearest neighbours (sq. distance) to search: " << K << endl;
  start = chrono::high_resolution_clock::now();
  vector<int> idx(K);     // Index of nearest neighbours
  vector<float> dist(K);  // Distance of nearest neigbours
  int found = tree.nearestKSearch(pt, K, idx, dist);
  if (found > 0)
  {
    for (int i = 0; i < idx.size(); i++)
    {
      cout << i << ":" << idx[i] << '\t' << sqrt(dist[i]) << '\t' << cloud->points[idx[i]] << endl;
    }
  }
  stop = chrono::high_resolution_clock::now();
  cout << "KDTree nearestKSearch Elapsed Time: " << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  start = chrono::high_resolution_clock::now();
  linearSearchNN(cloud, pt);
  stop = chrono::high_resolution_clock::now();
  cout << "linearSearcnNN Elapsed Time: " << chrono::duration_cast<chrono::microseconds>(stop - start).count()
       << " uSec" << endl;

  return 0;
}