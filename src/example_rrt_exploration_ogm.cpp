/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <bits/stdc++.h>                 // Standard C++
#include <costmap_2d/cost_values.h>      // Constants: NO_INFORMATION/FREE_SPACE/LETHAL_OBSTACLE
#include <costmap_2d/costmap_2d.h>       // 2D Costmap
#include <geometry_msgs/Point.h>         // Point
#include <geometry_msgs/PointStamped.h>  // RViz clicked point
#include <nav_msgs/OccupancyGrid.h>      // OGM
#include <ros/ros.h>                     // ROS
#include <visualization_msgs/Marker.h>   // RViz Marker

using namespace std;
using namespace geometry_msgs;

std::mt19937 rng(0);                              // Random Number Generator
std::uniform_real_distribution<double> distUniX;  // Update the min/max on mapCallback
std::uniform_real_distribution<double> distUniY;  // Update the min/max on mapCallback
costmap_2d::Costmap2D costmap;                    // Costmap
vector<Point> graph, roi;                         // Graph nodes, ROI
geometry_msgs::PointStamped msgFrontier;
visualization_msgs::Marker msgRRT, msgFrontiers;

/**
 * @brief Occupancy Grid Map (OGM) topic callback.
 *        Store the recieved map. Furthermore re-initialize the uniform random
 *        number generator max/min values based upon the map size.
 *
 * @param msg Pointer to the OGM.
 */
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  // Sample Map Size: 721x779 = 561659
  // Unknown [-1]:  413348 (73.59%)
  // Free [0]:      137739 (24.52%)
  // Obstacle [100]:10572  (01.88%)
  costmap.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x,
                    msg->info.origin.position.y);
  copy(msg->data.begin(), msg->data.end(), costmap.getCharMap());

  ROS_DEBUG("O(%3.1f,%3.1f) S(%d,%d)->S(%3.1f,%3.1f)\r\n", costmap.getOriginX(), costmap.getOriginY(),
            costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), costmap.getSizeInMetersX(),
            costmap.getSizeInMetersY());
  distUniX = std::uniform_real_distribution<double>(-costmap.getSizeInMetersX() / 2, costmap.getSizeInMetersX() / 2);
  distUniY = std::uniform_real_distribution<double>(-costmap.getSizeInMetersY() / 2, costmap.getSizeInMetersY() / 2);
}

/**
 * @brief RViz clicked point callback.
 *        Clear the existing frontiers and graph.
 *
 * @param msg Pointer to the clicked point.
 */
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  // Get the occupancy value of the current cell
  if (costmap.getCharMap() != nullptr)
  {
    // Clear the ROI and start point for re-initialization
    if (roi.size() >= 5)
    {
      roi.clear();
      graph.clear();
      msgRRT.points.clear();
      msgFrontiers.points.clear();
    }

    // Add the current point to the ROI
    roi.push_back(msg->point);

    unsigned int mx, my;
    costmap.worldToMap(msg->point.x, msg->point.y, mx, my);
    ROS_INFO("(%3.1f, %3.1f)->(%d,%d): %d", msg->point.x, msg->point.y, mx, my, costmap.getCost(mx, my));

    // Update the ROI when four points are recieved.
    if (roi.size() == 4)
    {
      // Calculate the bounding box of the first four points
      double minx = DBL_MAX, miny = DBL_MAX, maxx = DBL_MIN, maxy = DBL_MIN;
      for (int i = 0; i < 4; i++)
      {
        if (roi[i].x < minx)
          minx = roi[i].x;
        if (roi[i].x > maxx)
          maxx = roi[i].x;
        if (roi[i].y < miny)
          miny = roi[i].y;
        if (roi[i].y > maxy)
          maxy = roi[i].y;
      }
      distUniX = std::uniform_real_distribution<double>(minx, maxx);
      distUniY = std::uniform_real_distribution<double>(miny, maxy);
    }
    else if (roi.size() == 5)
    {
      // Add the fifth point as starting location
      graph.push_back(msg->point);
    }
  }
}

/**
 * @brief Return the 2nd Norm or Euclidean distance between two points.
 *
 * @param v1 First point.
 * @param v2 Second point.
 * @return double Distance between the two points.
 */
inline double euclideanDistance(Point& v1, Point& v2)
{
  return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
}

/**
 * @brief Return the squared Euclidean distance between two points.
 *
 * @param v1 First point.
 * @param v2 Second point.
 * @return double Squared distance between the two points.
 */
inline double euclideanDistanceSq(Point& v1, Point& v2)
{
  return pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2);
}

/**
 * @brief Nearest point in the list in term of Euclidean distance. It assumes
 * at-least a single point is in the list. If the minimum distance is not
 * specified the maximum is assumed (1e+37).
 *
 * @param vertices List of Points
 * @param pt Search point
 * @param min Minimum distance for search
 * @return Point
 */
Point nearestNeighbor(vector<Point>& vertices, Point& pt, double min = DBL_MAX)
{
  Point res;
  float minDist;
  for (auto&& v : vertices)
  {
    minDist = euclideanDistanceSq(v, pt);
    if (minDist <= FLT_EPSILON)  // FLT_EPSILON or 1e-3
      return v;
    else if (minDist < min)
    {
      min = minDist;
      res = v;
    }
  }
  return res;
}

/**
 * @brief Check if a point closer then the threshold exist in the list
 *
 * @param vertices List of points for checking
 * @param pt Query point
 * @param distSq Squared distance threshold
 * @return true If a point in the list exist with a distance less than threshold
 * @return false otherwise
 */
bool nearestExist(vector<Point>& vertices, Point& pt, double distSq = DBL_EPSILON)
{
  for (auto&& v : vertices)
    if (euclideanDistanceSq(v, pt) <= distSq)
      return true;
  return false;
}

/**
 * @brief Get a new point considering tree growth parameter (eta).
 *  eta is used for two purposes.
 *  If the distance between the two points is less than eta then the
 *  end point is returned. Otherwise linearly interpolate a new point between
 *  the start and end point using eta parameter. If eta=0, start point
 *  shall be returned, if eta=1, end point shall be returned, if eta<0
 *  then a point beyond start point is returend, anf if eta>1 then a
 *  point beyond end point is returned.
 *
 * @param ptStart Start Point
 * @param ptEnd End Point
 * @param dist distance between the points
 * @param eta Growth rate parameter [-1,1]
 * @return Point
 */
Point linearInterpolate(Point& ptStart, Point& ptEnd, float& dist, float eta = 0.5)
{
  dist = euclideanDistance(ptStart, ptEnd);
  if (dist <= eta)
  {
    return ptEnd;
  }
  else
  {
    Point res;
    res.x = (1 - eta) * ptStart.x + eta * ptEnd.x;
    res.y = (1 - eta) * ptStart.y + eta * ptEnd.y;
    return res;
  }
}

/**
 * @brief Determine, weather there is a obtacle/free/unknown between the two points on the map.
 *  If the start/end points are outside the map bounds, 100[Obstacle] is returened.
 *  If there is some obstacle or frontier between the start and end point, the result contains
 *  the world coordinate of the grid cell, otherwise the end point.
 * @param ptStart Start point
 * @param ptEnd End point
 * @param ptResult Result point
 * @return unsigned char 255[Unknown]/100[Obstacle]/0[Free]
 */
unsigned char rayTracing(Point& ptStart, Point& ptEnd, Point& ptResult)
{
  unsigned char cost = 100;
  unsigned int x0, y0, x1, y1;
  if (costmap.worldToMap(ptStart.x, ptStart.y, x0, y0) == false ||
      costmap.worldToMap(ptEnd.x, ptEnd.y, x1, y1) == false)
  {
    ptResult.x = DBL_MAX;
    ptResult.y = DBL_MAX;
    return cost;
  }

  int dx = abs(static_cast<int>(x1 - x0));
  int dy = -abs(static_cast<int>(y1 - y0));
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int e = dx + dy, e2;

  while (true)
  {
    cost = costmap.getCost(x0, y0);
    if (cost == 100 || cost == 255)
    {
      // Obstacle or Unknown cell
      costmap.mapToWorld(x0, y0, ptResult.x, ptResult.y);
      return cost;
    }
    else
    {
      // Free cell
      e2 = 2 * e;
      if (e2 >= dy)
      {
        if (x0 == x1)
          break;
        e += dy;
        x0 += sx;
      }  // dy
      if (e2 <= dx)
      {
        if (y0 == y1)
          break;
        e += dx;
        y0 += sy;
      }  // dx
    }
  }  // while
  costmap.mapToWorld(x0, y0, ptResult.x, ptResult.y);
  return cost;
}

int main(int argc, char* argv[])
{
  cout.precision(5);

  // Initialize the node
  ros::init(argc, argv, "example_rrt_exploration_ogm");
  ros::NodeHandle nh;

  //*
  // Set the log level of the node to display messages
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();
  //*/

  // Node parameters
  float eta, marker_size, rate_hz, min_dist;
  std::string map_topic;
  std::string ns = ros::this_node::getName();
  ros::param::param<std::string>(ns + "/map_topic", map_topic, "/map");
  ros::param::param<float>(ns + "/eta", eta, 0.5);
  ros::param::param<float>(ns + "/min_dist", min_dist, 1.0);
  ros::param::param<float>(ns + "/marker_size", marker_size, 1);
  ros::param::param<float>(ns + "/rate_hz", rate_hz, 10);
  ros::Rate rate(rate_hz);  // Set the update rate

  // Initialize the subscribers
  ros::Subscriber subMap = nh.subscribe(map_topic, 1, mapCallBack);
  ros::Subscriber subRViz = nh.subscribe("/clicked_point", 1, rvizCallBack);

  // Initialize the publishers
  ros::Publisher pubRRT = nh.advertise<visualization_msgs::Marker>(ns + "/rrt", 1, true);
  ros::Publisher pubFrontiers = nh.advertise<visualization_msgs::Marker>(ns + "/frontiers", 1, true);
  ros::Publisher pubFrontier = nh.advertise<geometry_msgs::PointStamped>(ns + "/frontier", 1);

  // Prepare Frontier message
  msgFrontier.header.frame_id = "map";
  msgFrontier.header.stamp = ros::Time::now();

  // Prepare RRT graph message
  msgRRT.header.frame_id = "map";
  msgRRT.header.stamp = ros::Time::now();
  msgRRT.ns = "Nodes";
  msgRRT.id = 0;
  msgRRT.type = visualization_msgs::Marker::LINE_LIST;
  msgRRT.action = visualization_msgs::Marker::ADD;
  msgRRT.color.b = msgRRT.color.a = 1;
  msgRRT.pose.orientation.w = 1;
  msgRRT.scale.x = marker_size;
  msgRRT.lifetime = ros::Duration(-1);

  // Prepare frontiers message
  msgFrontiers.header.frame_id = "map";
  msgFrontiers.header.stamp = ros::Time::now();
  msgFrontiers.ns = "Frontiers";
  msgFrontiers.id = 0;
  msgFrontiers.type = visualization_msgs::Marker::POINTS;
  msgFrontiers.action = visualization_msgs::Marker::ADD;
  msgFrontiers.color.g = msgFrontiers.color.a = 1;
  msgFrontiers.pose.orientation.w = 1;
  msgFrontiers.scale.x = msgFrontiers.scale.y = 3 * marker_size;
  msgFrontiers.lifetime = ros::Duration(-1);

  // Wait for the map to be available
  ROS_INFO("Waiting for map...");
  while (costmap.getCharMap() == nullptr)
  {
    ros::spinOnce();
    ros::Duration(0.2).sleep();
  }
  ROS_INFO("Map received.");

  // Wait for the start location
  ROS_INFO("Waiting for start location and ROI...");
  while (roi.size() < 5)
  {
    ros::spinOnce();
    ros::Duration(0.2).sleep();
  }
  ROS_INFO("Start location and ROI recieved.");

  // Intermediate variables
  Point ptSample, ptStart, ptEnd, ptResult;
  unsigned char mapValue;
  float ptDist;
  // geometry_msgs::Point pt;

  while (ros::ok())
  {
    if (roi.size() >= 5)
    {
      // Generate a random sample point
      ptSample.x = distUniX(rng);
      ptSample.y = distUniY(rng);
      ROS_DEBUG_STREAM_THROTTLE(1, "Sample Point: " << ptSample.x << "," << ptSample.y);

      // Find the nearest node in the graph to the sample
      ptStart = nearestNeighbor(graph, ptSample);
      ROS_DEBUG_STREAM_THROTTLE(1, "Start Point: " << ptStart.x << "," << ptStart.y);

      // Get a scaled point in the sample direction (nearest->sample)
      ptEnd = linearInterpolate(ptStart, ptSample, ptDist, eta);
      ROS_DEBUG_STREAM_THROTTLE(1, "End Point: " << ptEnd.x << "," << ptEnd.y << ", Dist:" << ptDist);

      // Discard if the points are too close
      if (ptDist > costmap.getResolution())
      {
        mapValue = rayTracing(ptStart, ptEnd, ptResult);
        ROS_DEBUG_STREAM_THROTTLE(1, "Ray Tracing: " << (int)mapValue << "->" << ptStart.x << "," << ptStart.y);

        if (mapValue == 255)
        {
          // Unknown cell between start and end point
          // graph.push_back(ptResult); // ?
          // Add frontier by checking close-by frontier
          if (!nearestExist(msgFrontiers.points, ptResult, min_dist))
          {
            msgFrontier.header.stamp = ros::Time::now();
            msgFrontier.point = ptResult;
            pubFrontier.publish(msgFrontier);
            msgFrontiers.points.push_back(ptResult);
            pubFrontiers.publish(msgFrontiers);
          }
        }
        else if (mapValue != 100)
        {
          // No obstacle in between start and end point
          graph.push_back(ptResult);
          msgRRT.header.stamp = ros::Time::now();
          msgRRT.points.push_back(ptStart);
          msgRRT.points.push_back(ptResult);
          pubRRT.publish(msgRRT);
        }
      }
      ROS_DEBUG_STREAM_THROTTLE(1, "Graph Nodes: " << graph.size() << "\t Frontiers:" << msgFrontiers.points.size());
      ROS_DEBUG_STREAM_THROTTLE(1, "");
    }
    ros::spinOnce();
    rate.sleep();
  }  // while

  return 0;
}