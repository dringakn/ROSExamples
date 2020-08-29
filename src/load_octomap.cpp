/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "load_octomap");
  ros::NodeHandle nh("~");
  std::string file_name;
  nh.param<std::string>("map_file", file_name, "");
  octomap::OcTree ot(file_name);
  octomap_msgs::Octomap msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time(0);
  octomap_msgs::binaryMapToMsg(ot, msg);
  ros::Publisher pubOcto;
  pubOcto = nh.advertise<octomap_msgs::Octomap>("/octomap", 1, true);
  pubOcto.publish(msg);
  ros::spin();
  return 0;
}