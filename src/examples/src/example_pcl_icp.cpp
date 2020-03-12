/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        The following program create a 3D map using inertial and
        3D point cloud date.
        PointCloud2 message holds a collection of nD points, as a binary blob.
        Height and Width descirbes 2D structure of the point cloud. If the cloud
        is unordered, height is 1 and width is the length of the point cloud.
        The fields describes the channels and their layout in the binary data
   blob.
        point_step describes the length of a point in bytes.
        row_step describes the length of a row in bytes.
        is_dense is set to  true if there are no invalid points.
        data.size(): Actual point data, size is (row_step*height)
        #This message holds the description of one point entry in the
        #PointCloud2 message format.
            uint8 INT8 = 1
            uint8 UINT8 = 2
            uint8 INT16 = 3
            uint8 UINT16 = 4
            uint8 INT32 = 5
            uint8 UINT32 = 6
            uint8 FLOAT32 = 7
            uint8 FLOAT64 = 8
            string name # Name of field
            uint32 offset # Offset from start of point struct
            uint8 datatype # Datatype enumeration see above
            uint32 count # How many elements in field
        PointField examples:
            “x”, 0, 7, 1
            “y”, 4, 7, 1
            “z”, 8, 7, 1
            “rgba”, 12, 6, 1
            “normal_x”, 16, 8, 1
            “normal_y”, 20, 8, 1
            “normal_z”, 24, 8, 1
            “fpfh”, 32, 7, 33
    concatenate Fields of two PCs:
     The fields of both PCs are merged together, e.g. if one contains XYZ and
     the other contains the RGB, then the concated one contains the XYZRGB.
    concatenate Points of two PCs:
     The points of the second PC is appended at the end of first PC.

    Notes:
        include_directory (... include ${Eigen_INCLUDE_DIRS} ...)
        include_directory (... include ${PCL_INCLUDE_DIRS} ...)
        find_package(PCL 1.7 REQUIRED)
        find_package(catkin REQUIRED COMPONENTS
        roscpp sensor_msgs geometry_msgs tf pcl_ros nav_msgs random_numbers)
*/
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>

using namespace std;

int main(int argc, char *argv[]) {
  random_numbers::RandomNumberGenerator rng;
  pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);

  int n = (argc >= 1) ? atoi(argv[1]) : 5;
  double limit = (argc >= 2) ? atof(argv[2]) : 1.0;
  double dx = (argc >= 3) ? atof(argv[3]) : 1.0;

  in->width = n;
  in->height = 1;
  in->is_dense = true;
  in->points.resize(in->height * in->width);

  cout << "Input cloud:" << endl;
  for (size_t i = 0; i < in->points.size(); i++) {
    in->points[i].x = rng.uniformReal(-limit, limit);
    in->points[i].y = rng.uniformReal(-limit, limit);
    in->points[i].z = rng.uniformReal(-limit, limit);
    cout << in->points[i] << endl;
  }

  *out = *in;
  //   out = in; // This doesn't works!!!
  cout << "Transformed cloud:" << endl;
  for (size_t i = 0; i < out->points.size(); i++) {
    out->points[i].x += dx;
    cout << out->points[i] << endl;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.setInputSource(in);
  icp.setInputTarget(out);
  icp.setMaximumIterations(10); // Number of RANSAC iterations
  icp.setTransformationEpsilon(1e-6);
  //icp.setMaxCorrespondenceDistance(1);
  icp.align(Final);
  cout << "Converged:" << icp.hasConverged() << endl
       << "Score:" << icp.getFitnessScore() << endl;
  cout << "Transformation" << endl << icp.getFinalTransformation() << endl;

  return 0;
}