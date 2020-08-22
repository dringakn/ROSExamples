#include <bits/stdc++.h>               // C++ Related stuff
#include <octomap/octomap.h>           // Octree
#include <octomap_msgs/conversions.h>  // Octomap messages
#include <ros/ros.h>                   // ROS Related stuff

using namespace std;

int main(int argc, char* argv[]) {
  // Initialize node
  ros::init(argc, argv, "example_octomap_load");
  ros::NodeHandle nh;

  // Get pre saved octree map file (with path+ext)
  std::string file;
  if (argc > 1) {
    file = std::string(argv[1]);
  } else {
    file =
        std::string("/home/office/cmu_ws/src/ros_examples/map/sample_map2.bt");
  }

  // Create octree
  octomap::OcTree ot(file);

  // Create octree message publisher
  ros::Publisher pub;
  pub = nh.advertise<octomap_msgs::Octomap>((argc > 2) ? argv[2] : "/octomap",
                                            1, true);

  // Create octree ROS message
  octomap_msgs::Octomap msg;
  octomap_msgs::binaryMapToMsg(ot, msg);
  msg.header.frame_id = (argc > 3) ? argv[3] : "map";
  msg.header.stamp = ros::Time(0);
  pub.publish(msg);

  // Perodic ROS code
  ros::Rate rate(1);
  while (ros::ok()) {
    // Yield control to ROS
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}