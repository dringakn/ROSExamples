/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

using namespace std;
typedef octomap::point3d Point3D;
typedef octomath::Vector3 Vector3;
typedef octomath::Quaternion Quaternion;
typedef octomath::Pose6D Pose6D;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

double sample_res;
double minx, miny, minz, maxx, maxy, maxz;

ros::Publisher pubFrontiersPC;
ros::Publisher pubFrontiersOT;

void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg) {
  octomap::OcTree* ot =
      dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
  int depth = floor(16 - (log(sample_res / ot->getResolution()) / log(2)));
  sample_res = ot->getResolution() * pow(2, 16 - depth);

  octomap::point3d max(maxx, maxy, maxz), min(minx, miny, minz);
  octomap::point3d_list list;
  ot->getUnknownLeafCenters(list, min, max, depth);

  octomap::OcTree frontiers(sample_res);
  frontiers.setBBXMax(max);
  frontiers.setBBXMin(min);
  frontiers.useBBXLimit(true);

  PointCloud msgPCL;
  msgPCL.header.frame_id = msg->header.frame_id;
  msgPCL.header.stamp = msg->header.stamp.toNSec() / 1000UL;
  msgPCL.height = 1;
  msgPCL.width = list.size();
  msgPCL.is_dense = false;

  for (auto&& pt : list) {
    frontiers.updateNode(pt, true);
    msgPCL.push_back(Point(pt.x(), pt.y(), pt.z()));
  }
  ROS_INFO_STREAM(ros::this_node::getName() << " frontiers: " << list.size());

  octomap_msgs::Octomap msgOT;
  msgOT.header = msg->header;
  octomap_msgs::binaryMapToMsg(frontiers, msgOT);
  pubFrontiersOT.publish(msgOT);
  pubFrontiersPC.publish(msgPCL);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "frontiers_detector");
  ros::NodeHandle nh("~");

  string topic;
  if (!nh.param<string>("octomap_topic", topic, "/octomap"))
    nh.setParam("octomap_topic", topic);
  if (!nh.param<double>("sample_res", sample_res, 1))
    nh.setParam("sample_res", sample_res);
  if (!nh.param<double>("minx", minx, -10)) nh.setParam("minx", minx);
  if (!nh.param<double>("miny", miny, -10)) nh.setParam("miny", miny);
  if (!nh.param<double>("minz", minz, -10)) nh.setParam("minz", minz);
  if (!nh.param<double>("maxx", maxx, 10)) nh.setParam("maxx", maxx);
  if (!nh.param<double>("maxy", maxy, 10)) nh.setParam("maxy", maxy);
  if (!nh.param<double>("maxz", maxz, 10)) nh.setParam("maxz", maxz);

  ROS_INFO("Octomap Topic: %s", topic.c_str());
  ROS_INFO("Sample Res.: %f", sample_res);
  ROS_INFO("min: %f, %f, %f", minx, miny, minz);
  ROS_INFO("max: %f, %f, %f", maxx, maxy, maxz);

  ros::Subscriber subOctree;
  subOctree = nh.subscribe<octomap_msgs::Octomap>(topic, 1, octomap_callback);
  pubFrontiersOT =
      nh.advertise<octomap_msgs::Octomap>("/frontiers_ot", 1, true);
  pubFrontiersPC = nh.advertise<PointCloud>("/frontiers_pc", 1, true);

  ros::spin();

  return 0;
}