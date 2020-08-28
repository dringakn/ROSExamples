#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

using namespace std;
typedef octomap_msgs::Octomap Octomap;
typedef octomap::point3d Point3D;
typedef octomath::Vector3 Vector3;
typedef octomath::Quaternion Quaternion;
typedef octomath::Pose6D Pose6D;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

ros::Publisher pubFrontiersPC;
octomap::OcTree* ot;
octomap::OcTree frontiers(1);

void octomap_callback(const Octomap::ConstPtr& msg) {
  ot = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
  frontiers.setResolution(ot->getResolution());
}

void frontiers_callback(const PointCloud::ConstPtr& msg) {
  for (auto&& pt : msg->points) {
    octomap::point3d p(pt.x, pt.y, pt.z);
    frontiers.updateNode(p, true, true);
  }
  octomap_msgs::Octomap m;
  m.header.frame_id = msg->header.frame_id;
  m.header.stamp = ros::Time(0);
  octomap_msgs::binaryMapToMsg(frontiers, m);
  pubFrontiersPC.publish(m);
  ROS_INFO_STREAM(ros::this_node::getName()
                  << " frontiers: " << frontiers.getNumLeafNodes());
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "frontiers_filter");
  ros::NodeHandle nh("~");

  string frontiers_topic, octomap_topic;
  if (!nh.param<string>("octomap_topic", octomap_topic, "/octomap"))
    nh.setParam("octomap_topic", octomap_topic);
  if (!nh.param<string>("frontiers_topic", frontiers_topic, "/frontiers_pc"))
    nh.setParam("frontiers_topic", frontiers_topic);

  double height, info_rad;
  if (!nh.param<double>("height", height, 10)) nh.setParam("height", height);
  if (!nh.param<double>("info_rad", info_rad, 10))
    nh.setParam("info_rad", info_rad);

  ROS_INFO("Octomap Topic: %s", octomap_topic.c_str());
  ROS_INFO("Frontiers Topic: %s", frontiers_topic.c_str());
  ROS_INFO("Height: %f", height);
  ROS_INFO("Info Rad.: %f", info_rad);

  ros::Subscriber subOctree, subFrontiers;
  subOctree = nh.subscribe<Octomap>(octomap_topic, 1, octomap_callback);
  subFrontiers =
      nh.subscribe<PointCloud>(frontiers_topic, 1, frontiers_callback);

  pubFrontiersPC = nh.advertise<Octomap>("/frontiers_filtered", 1, true);

  ros::spin();
  return 0;
}