/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/

// rosrun map_server map_server map.yaml
// or
// rosrun examples example_occupancy_grid_map 10 10 0.5

// Above node publishes a latched map topic and a static_map service.
// We can write a node which can subscribe to the "map" topic or call
// the "static_map" service to obtain the map.
//
// Rviz can be used to specify the start and target point.
// The RViz interface can be used to publish following topics:
// 		/clicked_point [geometry_msgs/PointStamped]
// 		/initialpose [geometry_msgs/PoseWithCovarianceStamped]
// 		/move_base_simple/goal [geometry_msgs/PoseStamped]
//
// Modify find_packages(... geometry_msgs nav_msgs ...)

/*
    Comparision: Map (1000x1000 @ 0.5m/px)
                ExploredCells PathSteps
    Dijkstra    237346        462
    AStar       107489        458

*/

#include "AStar.h"

AStar g;
ros::Publisher path_pub;

bool frame2map(const tf::TransformListener &li, geometry_msgs::PointStamped &pt,
               geometry_msgs::PointStamped &pt_m) {
  // target frame, source frame
  if (li.canTransform("map", pt.header.frame_id, ros::Time(0))) {
    li.transformPoint("map", pt, pt_m);
    std::cout << pt_m.header.frame_id << ": " << pt_m.point.x << ","
              << pt_m.point.y << "," << pt_m.point.z << std::endl;
    return true;
  } else {
    ROS_INFO("Transform is not available");
    return false;
  }
}

void map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Map: %d x %d @ %0.3f m/px", msg->info.width, msg->info.height,
           msg->info.resolution);
  g.setMap(msg);
}

void initpose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                 const tf::TransformListener &li) {
  geometry_msgs::PointStamped pt, pt_m;
  pt.header = msg->header;
  pt.header.stamp = ros::Time();
  pt.point = msg->pose.pose.position;
  if (frame2map(li, pt, pt_m)) g.setStartPoint(pt_m);
}

void goalpose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg,
                 const tf::TransformListener &li) {
  geometry_msgs::PointStamped pt, pt_m;
  pt.header = msg->header;
  pt.header.stamp = ros::Time();
  pt.point = msg->pose.position;
  if (frame2map(li, pt, pt_m)) {
    g.setGoalPoint(pt_m);
    int pathSteps = g.shortestPath();
    if (pathSteps != 0) {
      cout << "Path Found, Steps:" << pathSteps << endl;
      path_pub.publish(g.getPath());
    } else {
      cout << "Path Not Found" << endl;
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "astar");
  ros::NodeHandle nh;
  tf::TransformListener li(ros::Duration(10));

  path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);

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