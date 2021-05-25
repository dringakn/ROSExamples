/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *    Notes: Add pcl_ros library in CMakeLists
 **/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_pcl_load_ply_file");
  ros::NodeHandle nh("~");

  std::string map_file_name;
  if (!nh.param<std::string>("map_file_name", map_file_name, "/home/workshop/LUMS1.ply"))
    nh.setParam("map_file_name", map_file_name);
  ROS_INFO("Map file: %s", map_file_name.c_str());

  ros::Publisher pub =
      nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 1000, true);
 
  pcl::PCLPointCloud2* pc = new pcl::PCLPointCloud2;
 
  if (pcl::io::loadPLYFile(map_file_name.c_str(), *pc) == -1) {
    ROS_INFO("Pointcloud file (%s) not loaded", map_file_name.c_str());
    return -1;
  }else{
    ROS_INFO("Loaded %dx%d points from %s", pc->width, pc->height, map_file_name.c_str());
    sensor_msgs::PointCloud2 msg;
    pcl_conversions::fromPCL(*pc, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub.publish(msg);
  }

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}