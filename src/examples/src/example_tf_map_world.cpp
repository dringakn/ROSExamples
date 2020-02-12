/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
// Launch RViz and click "2D Pose Estimate" or "2D Nav Goal" button
// It shall publish a PoseWithCovarianceStamp or PoseStamped message res.
// on the "initialpose" and "move_base_simple/goal" respectively.
// Both of the topics are subscribed by a callback routine.
// The callback routine uses boost library to pass on an extra argument.
// The node assumes that another node is publisheing the transformation
// between world and map frame. If it's not the case, use static transform
// publisher as follows:
// rosrun tf static_transform_publisher x y z yaw pitch roll parent child t_ms
//
// use view_frames to view the tf graph (evince frames.pdf)
// or rosrun tf tf_monitor world map
// or rosrun tf tf_echo world map
// or rostopic echo /tf

#include <geometry_msgs/PoseStamped.h>               // Transformation pose
#include <geometry_msgs/PoseWithCovarianceStamped.h> // Transformation pose
#include <nav_msgs/MapMetaData.h>                    // OGM meta data
#include <nav_msgs/OccupancyGrid.h>                  // Occupancy grid map
#include <ros/ros.h>
#include <tf/transform_broadcaster.h> // Transform broadcaster
#include <tf/transform_listener.h>    // Transform listener

nav_msgs::MapMetaData map_info;

void map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  map_info = msg->info;
  ROS_INFO("Map: %d x %d @ %0.3f m/px", map_info.width, map_info.height,
           map_info.resolution);
}

bool frame2map(const tf::TransformListener &li, geometry_msgs::PointStamped &pt,
               geometry_msgs::PointStamped &pt_m) {
  // target frame, source frame
  if (li.canTransform("map", pt.header.frame_id, ros::Time(0))) {
    li.transformPoint("map", pt, pt_m);
    pt_m.point.x = ceil(pt_m.point.x / map_info.resolution);
    pt_m.point.y = ceil(pt_m.point.y / map_info.resolution);
    pt_m.point.z = ceil(pt_m.point.z / map_info.resolution);
    std::cout << pt_m << std::endl;
    return true;
  } else {
    ROS_INFO("Transform is not available");
    return false;
  }
}

void initpose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                 const tf::TransformListener &li) {
  geometry_msgs::PointStamped pt, pt_m;
  pt.header = msg->header;
  pt.header.stamp = ros::Time();
  pt.point = msg->pose.pose.position;
  frame2map(li, pt, pt_m);
}

void goalpose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg,
                 const tf::TransformListener &li) {
  geometry_msgs::PointStamped pt, pt_m;
  pt.header = msg->header;
  pt.header.stamp = ros::Time();
  pt.point = msg->pose.position;
  frame2map(li, pt, pt_m);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_tf_map_world");
  ros::NodeHandle nh;
  tf::TransformListener li(ros::Duration(10));

  ros::Subscriber initPose_sub =
      nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
          "initialpose", 1, boost::bind(&initpose_cb, _1, boost::ref(li)));

  ros::Subscriber goalPose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal", 1,
      boost::bind(&goalpose_cb, _1, boost::ref(li)));

  ros::Subscriber map_sub =
      nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, map_cb);

  ros::spin();
  return 0;
}
