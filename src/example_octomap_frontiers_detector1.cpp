#include <geometry_msgs/Point.h>
#include <octomap/octomap.h>
#include <octomap_frontiers/GetFrontiers.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

ros::ServiceClient client;

bool getFrontiers(octomap_frontiers::GetFrontiers::Request& req,
                  octomap_frontiers::GetFrontiers::Response& res) {
  if (client.exists()) {
    octomap_msgs::GetOctomapRequest reqMap;
    octomap_msgs::GetOctomapResponse resMap;
    if (client.call(reqMap, resMap)) {
      ros::WallTime startTime = ros::WallTime::now();
      octomap::OcTree* ot = dynamic_cast<octomap::OcTree*>(
          octomap_msgs::binaryMsgToMap(resMap.map));
      int depth =
          floor(16 - (log(req.sample_res / ot->getResolution()) / log(2)));
      octomap::point3d_list list;
      octomap::point3d max(req.max.x, req.max.y, req.max.z);
      octomap::point3d min(req.min.x, req.min.y, req.min.z);
      ot->getUnknownLeafCenters(list, min, max, depth);
      // if (list.size() == 0) return false;
      geometry_msgs::Point p;
      for (auto&& pt : list) {
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        res.frontiers.push_back(p);
      }
      res.header.frame_id = resMap.map.header.frame_id;
      res.header.stamp = ros::Time(0);
      ros::WallTime endTime = ros::WallTime::now();
      double execution_time = (endTime - startTime).toNSec() * 1e-6;
      ROS_INFO_STREAM(ros::this_node::getName()
                      << ", detected frontiers: " << list.size()
                      << ", Exectution time (ms): " << execution_time);

      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "frontiers_detector");
  ros::NodeHandle nh("~");

  std::string topic;
  if (!nh.param<std::string>("topic", topic, "/octomap_binary"))
    nh.setParam("topic", topic);
  ROS_INFO("Octomap Service Topic: %s", topic.c_str());

  client = nh.serviceClient<octomap_msgs::GetOctomap>(topic);

  ros::ServiceServer server =
      nh.advertiseService("get_frontiers", getFrontiers);

  ros::spin();

  return 0;
}