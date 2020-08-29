/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <bits/stdc++.h>                     // Standard C++ libraries
#include <pcl/point_cloud.h>                 // PCL
#include <pcl/visualization/cloud_viewer.h>  // PC Visulizer
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>  //ROS

using namespace std;

// Use Intensity to store data.
typedef pcl::PointXYZRGB Point3D;             // Points types
typedef pcl::PointCloud<Point3D> PointCloud;  // PointCloud

/**
  Visulizer Shortcut Keys:

          p, P   : switch to a point-based representation
          w, W   : switch to a wireframe-based representation (where available)
          s, S   : switch to a surface-based representation (where available)

          j, J   : take a .PNG snapshot of the current window view
          c, C   : display current camera/window parameters
          f, F   : fly to point mode

          e, E   : exit the interactor
          q, Q   : stop and call VTK's TerminateApp

           +/-   : increment/decrement overall point size
     +/- [+ ALT] : zoom in/out

          g, G   : display scale grid (on/off)
          u, U   : display lookup table (on/off)

    o, O         : switch between perspective/parallel projection (default =
 perspective) r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} ->
 center_{x, y, z}] CTRL + s, S  : save camera parameters CTRL + r, R  : restore
 camera parameters

    ALT + s, S   : turn stereo mode on/off
    ALT + f, F   : switch between maximized window mode and original size

          l, L           : list all available geometric and color handlers for
 the current actor map ALT + 0..9 [+ CTRL]  : switch between different geometric
 handlers (where available) 0..9 [+ CTRL]  : switch between different color
 handlers (where available)

    SHIFT + left click   : select a point (start with -use_point_picking)

          x, X   : toggle rubber band selection mode for left mouse button
 * */

int main(int argc, char* argv[]) {
  // Initialize node
  ros::init(argc, argv, "example_pcl_cloud_viewer");
  ros::NodeHandle nh;

  std::mt19937 rng(0);  // Mersenne Twister RNG
  std::uniform_real_distribution<float> dist(-10, 10);

  // Point cloud
  PointCloud::Ptr cloud(new PointCloud());             // Create a pointer to PC
  cloud->height = 100;                                 // Organized point cloud
  cloud->width = 100;                                  // Cloud width
  cloud->is_dense = true;                              // No points are invalid
  cloud->points.resize(cloud->height * cloud->width);  // Adust size

  for (auto&& pt : cloud->points) {
    // Randomly generaate a 3D point [-10, 10]
    pt.x = dist(rng);
    pt.y = dist(rng);
    pt.z = dist(rng);
    // pt.intensity = 64;
    pt.g = 255;
    // uint32_t rgb = (static_cast<uint32_t>(255) << 16 |  // R G B
    //                 static_cast<uint32_t>(15) << 8 |
    //                 static_cast<uint32_t>(15));
    // pt.rgb = *reinterpret_cast<float*>(&rgb);
  }

  // Point cloud visulizer
  // Method 1
  // pcl::visualization::CloudViewer viewer("PC Viewer");
  // viewer.showCloud(cloud);  // Display the point cloud

  // Method 2
  pcl::visualization::PCLVisualizer visulizer("PC Visulizer");
  visulizer.setBackgroundColor(0, 0, 0);
  visulizer.addCoordinateSystem(10);
  visulizer.addPointCloud<Point3D>(cloud);
  Point3D center;
  visulizer.addSphere<Point3D>(center, 1);
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  visulizer.addPlane(coeffs, 2, 2, 0);
  visulizer.initCameraParameters();

  // Perodic code
  ros::Rate loop_rate(10);  // Node update rate
  while (ros::ok()) {
    visulizer.spinOnce(1, true);
    if (visulizer.wasStopped())  // Check if the window is closed
      ros::shutdown();           // close ros
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}