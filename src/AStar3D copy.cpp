/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Date: 14 June 2022
    Description:
      Node to test AStar3D.

*/

#include "AStar3D.hpp"
#include <dynamic_reconfigure/server.h>          // Dynamic reconfigure
#include <astar3d/AStar3DParamsConfig.h>         // Parameter server configurations
#include <gap_detection/path2geo.h>              // Cart2Geo
#include <astar3d/searchpath.h>                  // SearchPath
#include <geometry_msgs/TransformStamped.h>      // TF2 transform geometry msg
#include <geometry_msgs/PointStamped.h>          // TF2 point geometry msg
#include <tf2_ros/transform_listener.h>          // Robot transform listener
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // geometry msg from tf
#include <tf2_ros/message_filter.h>              // TF2 message filter
#include <message_filters/subscriber.h>          // TF2 message filter subscriber

tf2_ros::Buffer *tfBuffer = NULL;                                             // Transform2 listen buffer
tf2_ros::MessageFilter<geometry_msgs::PointStamped> *tf2_filter;              // TF2 message filter
message_filters::Subscriber<geometry_msgs::PointStamped> *robot_position_sub; // Robot position
geometry_msgs::TransformStamped robot_transform;                              // Robot transform

AStar3D<unsigned long int, double> g(false, true, true, 2);
ros::Publisher path_pub, path_geo_pub;
bool publish_geodatic = false, use_current_as_start = true;
ros::ServiceClient client;

void tf2_callback(const geometry_msgs::PointStampedConstPtr &point_ptr)
{
  geometry_msgs::PointStamped point_out;
  try
  {
    tfBuffer->transform(*point_ptr, point_out, "base_link");
    ROS_INFO("point of frame1 in frame of fram2 Position(x:%f y:%f z:%f)\n", point_out.point.x, point_out.point.y, point_out.point.z);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Failure %s\n", ex.what()); // Print exception which was caught
  }
}

bool get_robot_transform()
{
  bool result = false;
  try
  {
    robot_transform = tfBuffer->lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1));
    // ROS_INFO("%f, %f, %f", robot_transform.transform.translation.x, robot_transform.transform.translation.y, robot_transform.transform.translation.z);
    result = true;
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return result;
}

// Convert Cartesian Path to Geodatic Path wrt. first measurement
bool callback_searchPath(astar3d::searchpath::Request &req, astar3d::searchpath::Response &res)
{
  bool result = false;
  if (use_current_as_start)
  {
    req.start.point.x = robot_transform.transform.translation.x;
    req.start.point.y = robot_transform.transform.translation.y;
    req.start.point.z = robot_transform.transform.translation.z;
  }

  if (g.setStartPoint(req.start))
  {
    if (g.setGoalPoint(req.goal))
    {
      g.shortestPath();
      res.path = g.getPath();
      result = true;
      path_pub.publish(res.path);
      if (publish_geodatic)
      {
        gap_detection::path2geo srv;
        srv.request.cart = res.path;
        if (client.call(srv))
        {
          path_geo_pub.publish(srv.response.geo);
        }
        else
        {
          ROS_ERROR("Failed to call %s service!!!", client.getService().c_str());
        }
      }
    }
  }

  return result;
}

void paramsCallback(astar3d::AStar3DParamsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request [%d]: %d %f %d", level, config.search_depth, config.min_z, config.abort_search);
  g.setSearchDepth(config.search_depth);
  g.setSearchMinZ(config.min_z);
  if (config.abort_search)
  {
    g.abortSearch();
  }
}

void map_cb(const ufomap_msgs::UFOMapStamped::ConstPtr &msg)
{
  get_robot_transform();
  if (ufomap_msgs::msgToUfo(msg->map, g.map))
  {
    ROS_INFO("GetResolution: %5.2f", g.map.getResolution());
    g.map_frame = msg->header.frame_id;
    g.setSearchDepth(3); // Change search depth after recieving map
  }
  else
  {
    ROS_WARN("UFOMap conversion failed");
  }
}

void clickedpoint_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  ROS_INFO("Aborting search[%f,%f,%f]...", msg->point.x, msg->point.y, msg->point.z);
  g.abortSearch();
}

void initpose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  ROS_INFO("Setting start location.");
  geometry_msgs::PointStamped pt;
  pt.header = msg->header;
  pt.point = msg->pose.pose.position;
  if (g.setStartPoint(pt) == false)
  {
  }
}

void goalpose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (use_current_as_start)
  {
    ROS_INFO("Setting start location.");
    geometry_msgs::PointStamped pt;
    pt.header = msg->header;
    pt.point.x = robot_transform.transform.translation.x;
    pt.point.y = robot_transform.transform.translation.y;
    pt.point.z = robot_transform.transform.translation.z;
    if (g.setStartPoint(pt) == false)
    {
    }
  }
  ROS_INFO("Setting goal location.");
  geometry_msgs::PointStamped pt;
  pt.header = msg->header;
  pt.point = msg->pose.position;
  g.setGoalPoint(pt);
  int pathSteps = g.shortestPath();
  std::cout << "Path Found, Length:" << pathSteps << std::endl;
  if (pathSteps > 0)
  {
    path_pub.publish(g.getPath());
    if (publish_geodatic)
    {
      gap_detection::path2geo srv;
      srv.request.cart = g.getPath();
      if (client.call(srv))
      {
        path_geo_pub.publish(srv.response.geo);
      }
      else
      {
        ROS_ERROR("Failed to call %s service!!!", client.getService().c_str());
      }
    }
  }
  else
  {
    cout << "Path not found, " << endl;
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "AStar3D");
  ros::NodeHandle nh("~");

  tfBuffer = new tf2_ros::Buffer(ros::Duration(10), false);
  tf2_ros::TransformListener li(*tfBuffer, nh, true);
  // tf2_filter = new tf2_ros::MessageFilter<geometry_msgs::PointStamped>(robot_position_sub, *tfBuffer, "base_link", 1, 0);
  // robot_position_sub = new message_filters::Subscriber<geometry_msgs::PointStamped>();
  // robot_position_sub->subscribe(nh, "odom", 10);
  // tf2_filter->registerCallback(tf2_callback);

  if (!nh.param<bool>("use_current_as_start", use_current_as_start, true))
    nh.setParam("use_current_as_start", use_current_as_start);
  ROS_WARN("use_current_as_start: %d", use_current_as_start);

  if (!nh.param<bool>("publish_geodatic", publish_geodatic, false))
    nh.setParam("publish_geodatic", publish_geodatic);
  ROS_WARN("publish_geodatic: %d", publish_geodatic);

  dynamic_reconfigure::Server<astar3d::AStar3DParamsConfig> server;
  dynamic_reconfigure::Server<astar3d::AStar3DParamsConfig>::CallbackType cb;
  cb = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(cb);

  path_pub = nh.advertise<nav_msgs::Path>("/path", 1, true);
  path_geo_pub = nh.advertise<nav_msgs::Path>("/path_geo", 1, true);

  ros::Subscriber initPose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &initpose_cb);

  ros::Subscriber goalPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &goalpose_cb);

  ros::Subscriber abortSearch_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &clickedpoint_cb);

  ros::Subscriber map_sub = nh.subscribe<ufomap_msgs::UFOMapStamped>("/ufomap", 1, map_cb);

  client = nh.serviceClient<gap_detection::path2geo>("/path2geo", false);
  if (client.waitForExistence(ros::Duration(10))) // -1 to wait for infinity
    ROS_WARN("GPS conversion service [%s] is available.", client.getService().c_str());
  else
    ROS_ERROR("GPS conversion service [%s] is not available.", client.getService().c_str());

  ros::ServiceServer server_searchpath = nh.advertiseService("/searchpath", callback_searchPath);

  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "Shutting down.." << std::endl;
  return 0;
}