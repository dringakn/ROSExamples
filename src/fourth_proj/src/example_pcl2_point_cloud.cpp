#include "pcl/io/ply_io.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub =
      nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 1, true);
  pcl::PCLPointCloud2 *pc = new pcl::PCLPointCloud2;

  if (pcl::io::loadPLYFile("/home/ahmad/catkin_ws/src/examples/data/LUMS1.ply",
                           *pc) == -1) {
    ROS_ERROR("Couldn't read file LUMS1.ply");
    return -1;
  } else {
    ROS_INFO("Loaded %dx%d points from LUMS1.ply", pc->width, pc->height);
    sensor_msgs::PointCloud2 msg;
    pcl_conversions::fromPCL(*pc, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";
    pub.publish(msg);
  }
  ros::spin();
  return 0;
}