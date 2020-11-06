/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Rapidly Exploring Random Tree.
    Note: Mersenne Twister RNG has better properties.
    In RViz use the shortcut key (G/g) to reset the graph.
*/

#include <bits/stdc++.h>                 // Standard C++
#include <geometry_msgs/PointStamped.h>  // Geometry point message
#include <geometry_msgs/PoseStamped.h>   // RViz Goal message
#include <ros/ros.h>                     // ROS
#include <visualization_msgs/Marker.h>   // RViz Marker
using namespace std;

struct Point2D
{
  Point2D()
  {
    x = y = 0;
  }
  Point2D(double X, double Y) : x(X), y(Y)
  {
  }
  double x, y;
};
std::mt19937 rng(0);                              // Random Number Generator
std::uniform_real_distribution<double> distUniX;  // Update the min/max on mapCallback
std::uniform_real_distribution<double> distUniY;  // Update the min/max on mapCallback
vector<Point2D> graph;                            // Graph nodes
geometry_msgs::PointStamped msgSample;
visualization_msgs::Marker msgRRT;

void clickedPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  Point2D pt(msg->pose.position.x, msg->pose.position.y);
  if (pt.x >= distUniX.min() && pt.x <= distUniX.max() && pt.y >= distUniY.min() && pt.y <= distUniY.max())
  {
    graph.clear();
    msgRRT.points.clear();
    graph.push_back(pt);
  }
}

/**
 * @brief Return the squared Euclidean distance between two 2D points.
 *
 * @param v1 First 2D point.
 * @param v2 Second 2D point.
 * @return double Squared distance between the two points.
 */
inline double euclideanDistanceSq(Point2D& v1, Point2D& v2)
{
  return pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2);
}

/**
 * @brief Nearest point in the list in term of Euclidean distance. It assumes
 * at-least a single point is in the list. If the minimum distance is not
 * specified the maximum is assumed (1e+37).
 *
 * @param vertices List of 2D Points
 * @param pt Search point
 * @param min Minimum distance for search
 * @return Point2D
 */
Point2D nearestNeighbor(vector<Point2D>& vertices, Point2D& pt, double min = DBL_MAX)
{
  Point2D res;
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
 * @brief Get a new point considering tree growth parameter (growth_rate).
 *  growth_rate is used for two purposes.
 *  If the distance between the two points is less than growth_rate then the
 *  end point is returned. Otherwise linearly interpolate a new point between
 *  the start and end point using growth_rate parameter. If growth_rate=0, start point
 *  shall be returned, if growth_rate=1, end point shall be returned, if growth_rate<0
 *  then a point beyond start point is returend, anf if growth_rate>1 then a
 *  point beyond end point is returned.
 *
 * @param ptStart Nearest Point
 * @param ptEnd Sample Point
 * @param growth_rate Growth rate parameter [-1,1]
 * @return Point2D
 */
Point2D linearInterpolate(Point2D& ptStart, Point2D& ptEnd, float growth_rate = 0.5)
{
  if (euclideanDistanceSq(ptStart, ptEnd) <= growth_rate)
  {
    return ptEnd;
  }
  else
  {
    Point2D res;
    res.x = (1 - growth_rate) * ptStart.x + growth_rate * ptEnd.x;
    res.y = (1 - growth_rate) * ptStart.y + growth_rate * ptEnd.y;
    return res;
  }
}

int main(int argc, char* argv[])
{
  cout.precision(5);
  // Initialize the node
  ros::init(argc, argv, "example_rrt");
  ros::NodeHandle nh;

  distUniX = std::uniform_real_distribution<double>(-100, 100);
  distUniY = std::uniform_real_distribution<double>(-100, 100);
  ros::Subscriber subRViz = nh.subscribe("/move_base_simple/goal", 1, clickedPointCallback);

  // Node parameters
  float growth_rate, marker_size, rate_hz;
  std::string ns = ros::this_node::getName();
  ros::param::param<float>(ns + "/growth_rate", growth_rate, 0.5);
  ros::param::param<float>(ns + "/marker_size", marker_size, 1);
  ros::param::param<float>(ns + "/rate_hz", rate_hz, 10);

  // Initialize the publishers
  ros::Publisher pubSample =  // Newly detected frontier
      nh.advertise<geometry_msgs::PointStamped>(ns + "/Sample", 1);
  ros::Publisher pubRRT = nh.advertise<visualization_msgs::Marker>(ns + "/rrt", 1, true);

  msgSample.header.frame_id = "map";
  msgSample.header.stamp = ros::Time::now();

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

  // Set the update rate
  ros::Rate rate(rate_hz);

  // graph.push_back(Point2D(0, 0)); // Set the start point
  ROS_INFO("Waiting for start location ...");
  while (graph.size() == 0)
  {
    ROS_INFO("Waiting for start point ...");
    ros::spinOnce();
    ros::Duration(0.2).sleep();
  }
  ROS_INFO("Start location recieved.");

  Point2D ptSample, ptStart, ptEnd;
  int status;
  while (ros::ok())
  {
    // Generate a random sample point
    ptSample.x = distUniX(rng);
    ptSample.y = distUniY(rng);
    cout << "Sample Point: " << ptSample.x << "," << ptSample.y << endl;
    // Show the sample point
    msgSample.header.stamp = ros::Time::now();
    msgSample.point.x = ptSample.x;
    msgSample.point.y = ptSample.y;
    pubSample.publish(msgSample);

    // Find the nearest point in the graph to the sample
    ptStart = nearestNeighbor(graph, ptSample);
    cout << "Start Point: " << ptStart.x << "," << ptStart.y << endl;
    // Get a scaled point in the sample direction (nearest->sample)
    ptEnd = linearInterpolate(ptStart, ptSample, growth_rate);
    cout << "End Point: " << ptEnd.x << "," << ptEnd.y << endl;

    // Show the tree
    geometry_msgs::Point pt;
    pt.x = ptStart.x;
    pt.y = ptStart.y;
    msgRRT.header.stamp = ros::Time::now();
    msgRRT.points.push_back(pt);
    pt.x = ptEnd.x;
    pt.y = ptEnd.y;
    msgRRT.points.push_back(pt);
    pubRRT.publish(msgRRT);

    graph.push_back(ptEnd);
    cout << "Graph size: " << graph.size() << endl;
    cout << "---" << endl;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}