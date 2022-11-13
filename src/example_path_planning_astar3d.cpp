/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
      Test the AStar3D algorithm.
      For testing an UFO map is required.
      If there doesn't exist any, Use the example_ufomap_load_pcd_file to load and publish one.

    Usage:
      rosrun ros_examples example_ufomap_load_pcd_file
      The "example_ufomap_load_pcd_file" node loads a map from a file and publishes it
      on a latched topic and a transform between world->map frame. We can write
      a node which can subscribe to the "ufomap" topic.


    Rviz can be used to specify the start and target point.
    The RViz interface can be used to publish following topics:
        /clicked_point [geometry_msgs/PointStamped]
        /initialpose [geometry_msgs/PoseWithCovarianceStamped]
        /move_base_simple/goal [geometry_msgs/PoseStamped]

    Modify find_packages(... geometry_msgs nav_msgs ufomap_msgs ufomap_ros ...)

*/

#include "AStar3D.hpp"
#include <tf/transform_listener.h>          // Robot transform listener

unsigned int search_depth = 2;
AStar3D<unsigned long int, double> g(false, true, true, search_depth); // 0=0.25, 1=0.5, 2=1, 3=2
ros::Publisher path_pub;

bool frame2map(const tf::TransformListener &li, geometry_msgs::PointStamped &pt,
               geometry_msgs::PointStamped &pt_m)
{
  // target frame, source frame
  if (li.canTransform("map", pt.header.frame_id, ros::Time(0)))
  {
    li.transformPoint("map", pt, pt_m);
    std::cout << pt_m.header.frame_id << ": " << pt.point.x << "," << pt.point.y << "," << pt.point.z << " -> " << pt_m.point.x << "," << pt_m.point.y << "," << pt_m.point.z << std::endl;
    return true;
  }
  else
  {
    ROS_INFO("Transform is not available");
    return false;
  }
}

void map_cb(const ufomap_msgs::UFOMapStamped::ConstPtr &msg)
{
  // Convert ROS message to UFOMap
  if (ufomap_msgs::msgToUfo(msg->map, g.map))
  {
    ROS_INFO("Recieved Resolution: %5.2f",msg->map.info.resolution);
    ROS_INFO("GetResolution: %5.2f",g.map.getResolution());

    // g.printUFOPoint(msg->map.info., "Half Size: ");
    // Testing
    // geometry_msgs::PointStamped start, goal;
    // start.point.x = -90;
    // start.point.y = -90;
    // start.point.z = -90;
    // g.setStartPoint(start);
    // goal.point.x = 90;
    // goal.point.y = 90;
    // goal.point.z = 90;
    // g.setGoalPoint(goal);

    // int pathSteps = g.shortestPath();
    // if (pathSteps > 0)
    // {
    //   cout << "Path length:" << pathSteps << endl;
    //   path_pub.publish(g.getPath());
    // }
  }
  else
  {
    ROS_WARN("UFOMap conversion failed");
  }
}

void initpose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg, const tf::TransformListener &li)
{
  geometry_msgs::PointStamped pt, pt_m;
  pt.header = msg->header;
  pt.header.stamp = ros::Time();
  pt.point = msg->pose.pose.position;
  // if (frame2map(li, pt, pt_m))
  pt_m.point = msg->pose.pose.position;
    g.setStartPoint(pt_m);
}

void goalpose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, const tf::TransformListener &li)
{
  geometry_msgs::PointStamped pt, pt_m;
  pt.header = msg->header;
  pt.header.stamp = ros::Time();
  pt.point = msg->pose.position;
  // if (frame2map(li, pt, pt_m))
  pt_m.point = msg->pose.position;
  {
    g.setGoalPoint(pt_m);
    int pathSteps = g.shortestPath();
    if (pathSteps > 0)
    {
      cout << "Path Found, Length:" << pathSteps << endl;
      path_pub.publish(g.getPath());
    }
    else
    {
      cout << "Path 'Not' Found, " << endl;
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "astar");
  ros::NodeHandle nh;
  tf::TransformListener li(ros::Duration(10));

  path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);

  ros::Subscriber initPose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, boost::bind(&initpose_cb, _1, boost::ref(li)));

  ros::Subscriber goalPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, boost::bind(&goalpose_cb, _1, boost::ref(li)));

  ros::Subscriber map_sub = nh.subscribe<ufomap_msgs::UFOMapStamped>("ufomap", 1, map_cb);

  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "Shutting down.." << std::endl;
  return 0;
}