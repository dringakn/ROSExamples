/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <actionlib/client/simple_action_client.h>  // Action Client
#include <bits/stdc++.h>                            // Standard C++
#include <costmap_2d/cost_values.h>                 // Constants: NO_INFORMATION/FREE_SPACE/LETHAL_OBSTACLE
#include <costmap_2d/costmap_2d.h>                  // 2D Costmap
#include <geometry_msgs/Point.h>                    // Point
#include <geometry_msgs/PointStamped.h>             // RViz clicked point
#include <geometry_msgs/Pose.h>                     // Robot pose provided by move_base
#include <move_base_msgs/MoveBaseAction.h>          // Move base action
#include <move_base_msgs/MoveBaseFeedback.h>        // Move base feedback
#include <move_base_msgs/MoveBaseResult.h>          // Move base result
#include <nav_msgs/GetPlan.h>                       // Planning service
#include <nav_msgs/OccupancyGrid.h>                 // OGM
#include <ros/ros.h>                                // ROS
#include <sensor_msgs/PointCloud2.h>                // Frontiers
#include <visualization_msgs/Marker.h>              // RViz Marker

using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::mt19937 rng(0);                              // Random Number Generator
std::uniform_real_distribution<double> distUniX;  // Update the min/max on mapCallback
std::uniform_real_distribution<double> distUniY;  // Update the min/max on mapCallback
costmap_2d::Costmap2D costmap;                    // Costmap
vector<Point> graph, roi;                         // Graph nodes, ROI
geometry_msgs::PointStamped msgFrontier;          // Recent detected frontier
visualization_msgs::Marker msgRRT, msgFrontiers;  // Graph and Frontiers
sensor_msgs::PointCloud2 msgCost;                 // Frontiers cost message
Point robPos;                                     // Current robot position
MoveBaseClient* client;                           // Move base client
ros::ServiceClient client_pl;                     // Planning service
ros::Publisher pubFrontierCost;                   // Frontiers cost
move_base_msgs::MoveBaseGoal goal;                // Current goal
const unsigned char UNKNOWN = 255;                // Unknown cost value
const unsigned char OBSTACLE = 100;               // Obstacle cost value
const unsigned char OBST_THRES = 100;             // Obstacle Threshold
std::string frame("map");                         // frame_id

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
 * @param distSq Distance threshold
 * @return true If a point in the list exist with a distance less than threshold
 * @return false otherwise
 */
bool nearestFrontierExist(vector<Point>& vertices, Point& pt, double dist = DBL_EPSILON)
{
  dist = pow(dist, 2);
  for (auto&& v : vertices)
    if (euclideanDistanceSq(v, pt) <= dist)
      return true;
  return false;
}

unsigned int countNearestFrontiers(vector<Point>& vertices, Point& pt, double dist)
{
  unsigned int count = 0;
  dist = pow(dist, 2);
  for (auto&& v : vertices)
    if (euclideanDistanceSq(v, pt) <= dist)
      count++;
  return count;
}

Point selectFrontier(vector<Point>& vertices, Point& pt)
{
  double distCtr = 0, frontierCtr = 0;
  vector<Point> cost(vertices.size());
  for (unsigned int idx = 0; idx < vertices.size(); idx++)
  {
    cost[idx].x = euclideanDistance(vertices[idx], pt);
    distCtr += cost[idx].x;
    cost[idx].y = countNearestFrontiers(vertices, vertices[idx], 5);
    frontierCtr += cost[idx].y;
  }
  double minCost = DBL_MAX;
  Point result;
  msgCost.header.stamp = ros::Time::now();
  msgCost.width = vertices.size();
  msgCost.row_step = msgCost.point_step * msgCost.width;
  msgCost.data.resize(msgCost.row_step);

  for (unsigned int idx = 0; idx < vertices.size(); idx++)
  {
    // select one with maximum nearest frontiers with minimum distance
    cost[idx].x /= distCtr;
    cost[idx].y /= frontierCtr;
    cost[idx].z = cost[idx].x / cost[idx].y;
    if (cost[idx].z < minCost)
    {
      minCost = cost[idx].z;
      result = vertices[idx];
    }
  }

  if (pubFrontierCost != nullptr)
    if (pubFrontierCost.getNumSubscribers())
    {
      float x, y, z = 0, f1, f2, f3;
      for (unsigned int idx = 0; idx < vertices.size(); idx++)
      {
        x = vertices[idx].x;
        y = vertices[idx].y;
        f1 = cost[idx].x;
        f2 = cost[idx].x;
        f3 = cost[idx].x;
        memcpy(&msgCost.data[idx * msgCost.point_step + msgCost.fields[0].offset], &x, sizeof(float));
        memcpy(&msgCost.data[idx * msgCost.point_step + msgCost.fields[1].offset], &y, sizeof(float));
        memcpy(&msgCost.data[idx * msgCost.point_step + msgCost.fields[2].offset], &z, sizeof(float));
        memcpy(&msgCost.data[idx * msgCost.point_step + msgCost.fields[3].offset], &f1, sizeof(float));
        memcpy(&msgCost.data[idx * msgCost.point_step + msgCost.fields[4].offset], &f2, sizeof(float));
        memcpy(&msgCost.data[idx * msgCost.point_step + msgCost.fields[5].offset], &f3, sizeof(float));
      }
      pubFrontierCost.publish(msgCost);
    }

  return result;
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
  unsigned char cost = OBSTACLE;
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
    if (cost >= OBST_THRES || cost == UNKNOWN)
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

/**
 * @brief Check if there is obstacle in the neighbouring cells of the frontier
 *
 * @param pt The query location.
 * @param delta The padding around location.
 * @return true If there is no obstacle in the neighbourhood.
 * @return false Otherwise.
 */
bool checkFrontierProximity(Point& pt, unsigned char delta = 2)
{
  unsigned int mx, my;
  if (costmap.worldToMap(pt.x, pt.y, mx, my))
  {
    vector<costmap_2d::MapLocation> p1, p2;
    p1.push_back({ mx - delta, my - delta });
    p1.push_back({ mx - delta, my + delta });
    p1.push_back({ mx + delta, my + delta });
    p1.push_back({ mx + delta, my - delta });
    costmap.polygonOutlineCells(p1, p2);
    // ROS_INFO("polygonOutlineCells(...): %d", p2.size());
    int idx = 0;
    for (auto&& c : p2)
    {
      //   cout << idx++ << ": " << c.x << "," << c.y << endl;
      if (costmap.getCost(c.x, c.y) == OBSTACLE)
      {
        return false;
      }
    }
    return true;
  }
  return false;
}

/**
 * @brief Remove a frontier from the list
 *
 * @param vertices List of frontiers
 * @param pt  Frontier to be removed
 * @return true If the frontier exist and removed.
 * @return false Otherwise
 */
bool removeFrontier(vector<Point>& vertices, Point pt)
{
  auto result = std::find(vertices.begin(), vertices.end(), pt);
  if (result != vertices.end())
  {
    vertices.erase(result);
    return true;
  }
  return false;
}

/**
 * @brief Check if the plan exist from the start to goal location.
 *  The client works only, if the move_base is currently not in active state.
 *  Therefore, calling it during frontier detection will most likely fail. It
 *  can be used before sending the new goal location. The start and goal locations
 *  are assumed with zero orientation and in the map frame.
 *
 * @param client Movebase client
 * @param start Start location
 * @param goal Goal location
 * @return true If the path exist between start and goal.
 * @return false Otherwise.
 */
bool checkPlan(ros::ServiceClient& client, Point& start, Point& goal)
{
  // Create a request message
  nav_msgs::GetPlanRequest req;
  req.start.header.frame_id = req.goal.header.frame_id = frame;
  req.start.pose.orientation.w = req.goal.pose.orientation.w = 1;
  req.tolerance = 0.0;
  req.start.pose.position = start;
  req.goal.pose.position = goal;
  // Create a response message and call
  nav_msgs::GetPlanResponse res;
  if (client.exists())
  {
    if (client.call(req, res))
    {
      return (res.plan.poses.size() > 0) ? true : false;
    }
    else
    {
      // Planner declined or plan failed
      return false;
    }
  }
  else
  {
    // Service is not available
    return false;
  }
}

/**
 * @brief Remove the invalid frontiers from the list.
 *
 */
void filterFrontiers(void)
{
  if (costmap.getCharMap() == nullptr)
    return;

  unsigned int mx, my;
  unsigned char cost = 0;
  // Mutating for loop
  for (auto it = msgFrontiers.points.begin(); it != msgFrontiers.points.end();)
  {
    if (costmap.worldToMap(it->x, it->y, mx, my))
    {
      cost = costmap.getCost(mx, my);
      if (cost != UNKNOWN)
      {
        // The cell is not frontier any more
        it = msgFrontiers.points.erase(it);
        ROS_DEBUG("Removing (%3.1f, %3.1f)->(%d,%d): %d", it->x, it->y, mx, my, cost);
      }
      else
      {
        // Note the increment is not inside the loop statement
        ++it;
      }
    }
    else
    {
      // The cell is not within map bound
      it = msgFrontiers.points.erase(it);
      ROS_DEBUG("Removing (%3.1f, %3.1f)->(%d,%d): %d", it->x, it->y, mx, my, cost);
    }
  }
  // Check existing frontier goal state
  if (client != nullptr)
    if (client->isServerConnected())
      if (!client->getState().isDone())
      {
        // If move_base is currently running
        if (costmap.worldToMap(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, mx, my))
        {
          cost = costmap.getCost(mx, my);
          if (cost != UNKNOWN || cost >= OBST_THRES)
          {
            // If the frontier has been updated
            client->cancelAllGoals();
          }
        }
      }
}

void done_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Action Finished.");
  std::cout << "State: " << state.toString() << " [" << state.state_ << "]" << std::endl;
  std::cout << "Status: " << state.getText() << std::endl;
  std::cout << "result: " << *result << std::endl;
  //   std::cout << "isDone: " << state.isDone() << std::endl;
  switch (state.state_)
  {
    case 5:  // Aborted
    case 6:  // Succeed
      // Aborted assumes the plan doesn't exist.
      // Succeed assumes the current frontier is completed
      removeFrontier(msgFrontiers.points, goal.target_pose.pose.position);
      filterFrontiers();
      client->cancelAllGoals();
      break;

    default:
      break;
  }
}

void active_cb()
{
  ROS_INFO("Action becomes active.");
}

void feedback_cb(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
{
  //   ROS_INFO("Action Feedback recieved.");
  robPos = fb->base_position.pose.position;
}

/**
 * @brief Assign the closest frontier to the robot.
 *
 */
void assignFrontier(void)
{
  if (msgFrontiers.points.size())
  {
    // Send goal to the client if there is a connection between server/client
    if (client->isServerConnected())
    {
      ROS_INFO("Server is connected.");
      // Cancel all goals currently running on the server, if any
      // The goal id is reset: status.id {node_name-goalID-time_stamp}
      // ROS_INFO("Cancelling all previous goals.");
      // client.cancelAllGoals();
      // Send the goal and register the callbacks
      Point goalPos = selectFrontier(msgFrontiers.points, robPos);
      if (checkPlan(client_pl, robPos, goalPos))
      {
        // Create a goal message
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = goalPos;
        client->sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
        ROS_INFO_STREAM("New goal sent.\r\n" << goal);
      }
      else
      {
        removeFrontier(msgFrontiers.points, goalPos);
        ROS_INFO_STREAM("Removing frontier because of invalid plan.\r\n" << goalPos);
      }
    }
    else
    {
      ROS_INFO("Server is not connected.");
    }
  }
}

/**
 * @brief Occupancy Grid Map (OGM) topic callback.
 *        Store the recieved map. Furthermore re-initialize the uniform random
 *        number generator max/min values based upon the map size.
 *
 * @param msg Pointer to the OGM.
 */
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  // Unknown [-1]:  413348 (73.59%)
  // Free [0]:      137739 (24.52%)
  // Obstacle [100]:10572  (01.88%)
  costmap.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x,
                    msg->info.origin.position.y);
  copy(msg->data.begin(), msg->data.end(), costmap.getCharMap());

  ROS_DEBUG("O(%3.1f,%3.1f) S(%d,%d)->S(%3.1f,%3.1f)\r\n", costmap.getOriginX(), costmap.getOriginY(),
            costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), costmap.getSizeInMetersX(),
            costmap.getSizeInMetersY());

  // Filter frontiers array on recieving of new map
  filterFrontiers();
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

int main(int argc, char* argv[])
{
  cout.precision(5);

  // Initialize the node
  ros::init(argc, argv, "example_rrt_exploration_ogm_with_filtering_and_assigner");
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
  pubFrontierCost = nh.advertise<sensor_msgs::PointCloud2>("/frontiers_cost", 1, true);

  // Prepare Frontier message
  msgFrontier.header.frame_id = frame;
  msgFrontier.header.stamp = ros::Time::now();

  // Prepare RRT graph message
  msgRRT.header.frame_id = frame;
  msgRRT.header.stamp = ros::Time::now();
  msgRRT.ns = "Nodes";
  msgRRT.id = 0;
  msgRRT.type = visualization_msgs::Marker::LINE_LIST;
  msgRRT.action = visualization_msgs::Marker::ADD;
  msgRRT.color.b = msgRRT.color.a = 1;
  msgRRT.pose.orientation.w = 1;
  msgRRT.scale.x = marker_size;
  msgRRT.lifetime = ros::Duration(0);

  // Prepare frontiers message
  msgFrontiers.header.frame_id = frame;
  msgFrontiers.header.stamp = ros::Time::now();
  msgFrontiers.ns = "Frontiers";
  msgFrontiers.id = 0;
  msgFrontiers.type = visualization_msgs::Marker::POINTS;
  msgFrontiers.action = visualization_msgs::Marker::ADD;
  msgFrontiers.color.g = msgFrontiers.color.a = 1;
  msgFrontiers.pose.orientation.w = 1;
  msgFrontiers.scale.x = msgFrontiers.scale.y = 3 * marker_size;
  msgFrontiers.lifetime = ros::Duration(0);

  msgCost.header.frame_id = frame;
  msgCost.height = 1;
  msgCost.is_dense = 0;
  msgCost.is_bigendian = false;
  msgCost.point_step = 0;

  sensor_msgs::PointField field;
  field.count = 1;

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 0;
  field.name = "x";
  msgCost.fields.push_back(field);
  msgCost.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset += sizeof(float);
  field.name = "y";
  msgCost.fields.push_back(field);
  msgCost.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset += sizeof(float);
  field.name = "z";
  msgCost.fields.push_back(field);
  msgCost.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset += sizeof(float);
  field.name = "dist";
  msgCost.fields.push_back(field);
  msgCost.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset += sizeof(float);
  field.name = "counts";
  msgCost.fields.push_back(field);
  msgCost.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset += sizeof(float);
  field.name = "cost";
  msgCost.fields.push_back(field);
  msgCost.point_step += sizeof(float);

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

  // Instinate Move base action client object
  client = new MoveBaseClient("move_base", true);

  // Initialize the goal message
  goal.target_pose.header.frame_id = frame;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  // Wait for the server-client to establish connection
  // 0 = Inf timeout, returns false on timeout
  while (!client->waitForServer(ros::Duration(0)))
    ROS_INFO_STREAM(ros::this_node::getName() << " waiting for the server connection...");
  ROS_INFO_STREAM(ros::this_node::getName() << ", Server connection established.");

  // Wait for the service to become active
  client_pl = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  ROS_INFO("Waiting for planning service...");
  while (!client_pl.waitForExistence(ros::Duration(1)))
  {
  }
  ROS_INFO("Planning service available.");

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

        if (mapValue == UNKNOWN)  // Unknown cell between start and end point
        {
          // Checck if there is no other frontier within specified distance.
          if (!nearestFrontierExist(msgFrontiers.points, ptResult, min_dist))
          {
            if (checkFrontierProximity(ptResult, 3))
            {
              msgFrontier.header.stamp = ros::Time::now();
              msgFrontier.point = ptResult;
              pubFrontier.publish(msgFrontier);
              msgFrontiers.points.push_back(ptResult);
              if (pubFrontiers.getNumSubscribers())
                pubFrontiers.publish(msgFrontiers);
            }
            else
            {
            }
          }
        }
        else if (mapValue < OBST_THRES)
        {
          // No obstacle in between start and end point
          graph.push_back(ptResult);
          msgRRT.header.stamp = ros::Time::now();
          msgRRT.points.push_back(ptStart);
          msgRRT.points.push_back(ptResult);
          if (pubRRT.getNumSubscribers())
            pubRRT.publish(msgRRT);
        }
      }
      ROS_DEBUG_STREAM_THROTTLE(1, "Graph Nodes: " << graph.size() << "\t Frontiers:" << msgFrontiers.points.size());
      ROS_DEBUG_STREAM_THROTTLE(1, "");
    }

    if (client->isServerConnected())
    {
      if (client->getState().isDone())
      {
        assignFrontier();
      }
      else
      {
        ROS_DEBUG_THROTTLE(1, "Movebase state: %s", client->getState().toString().c_str());
      }
    }
    else
    {
      ROS_DEBUG_THROTTLE(1, "Movebase server not connected");
    }

    ros::spinOnce();
    rate.sleep();
  }  // while

  return 0;
}