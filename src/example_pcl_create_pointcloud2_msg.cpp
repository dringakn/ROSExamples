/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <pcl_conversions/pcl_conversions.h>  // to/from PCL
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud<Point3D> PointCloud;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_pcl_create_pointcloud2_msg");
  ros::NodeHandle nh;

  // In case of pcl::PointCloud<T>, no need to convert it to a
  // sensor_msgs::PointCloud2. The serialization takes care of the
  // sensor_msgs::PointCloud2 message.
  ros::Publisher pub = nh.advertise<PointCloud>("/pointcloud", 1, true);

  // Create and fill the point cloud message
  PointCloud pc;
  pc.header.frame_id = "map";
  // pcl_conversions::toPCL(ros::Time(0), pc.header.stamp);
  pc.header.stamp = ros::Time(0).toNSec() / 1000UL;
  pc.height = 1;        // 1 If unordered, otherwise image
  pc.is_dense = false;  // True if no invalid points
  pc.width = 8;         // Number o points
  pc.push_back(Point3D(1, 1, 1));
  pc.push_back(Point3D(1, -1, 1));
  pc.push_back(Point3D(-1, -1, 1));
  pc.push_back(Point3D(-1, 1, 1));
  pc.push_back(Point3D(1, 1, -1));
  pc.push_back(Point3D(1, -1, -1));
  pc.push_back(Point3D(-1, -1, -1));
  pc.push_back(Point3D(-1, 1, -1));

  pub.publish(pc);
  ros::spin();
  return 0;
}