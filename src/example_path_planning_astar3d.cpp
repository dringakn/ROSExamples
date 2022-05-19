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

#include "AStar3D.h"

AStar3D g(false, true, false, 0);
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

void map_cb(ufomap_msgs::UFOMapStamped::ConstPtr const &msg)
{
  // Convert ROS message to UFOMap
  if (ufomap_msgs::msgToUfo(msg->map, g.map))
  {
    // Conversion was successful
    ROS_INFO("UFOMap conversion successful");
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
  if (frame2map(li, pt, pt_m))
    g.setStartPoint(pt_m);
}

void goalpose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, const tf::TransformListener &li)
{
  geometry_msgs::PointStamped pt, pt_m;
  pt.header = msg->header;
  pt.header.stamp = ros::Time();
  pt.point = msg->pose.position;
  if (frame2map(li, pt, pt_m))
  {
    g.setGoalPoint(pt_m);
    int pathSteps = g.shortestPath();
    if (pathSteps != 0)
    {
      cout << "Path Found, Steps:" << pathSteps << endl;
      path_pub.publish(g.getPath());
    }
    else
    {
      cout << "Path 'Not' Found, Steps:" << pathSteps << endl;
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

  ros::Subscriber map_sub = nh.subscribe<ufomap_msgs::UFOMapStamped>("map", 1, map_cb);

  ros::spin();

  return 0;
}