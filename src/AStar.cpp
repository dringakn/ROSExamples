/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *      Given a start point, end point and an OGM (Occupancy Grid Map) find the shortest path using A* algorithm.
 *      The input points are of standard ROS message type geometry_msgs::PointStamped and the OGM is of type
 *      nav_msgs::OccupancyGrid.
 *
 *      The output path is of type nav_msgs::Path.
 **/

#include "AStar.h"

AStar::AStar() {}

AStar::~AStar() {}

/**
 * @brief Set the start point for the path. The point is assumed to be in the map frame.
 *
 * @param s geometry_msgs::PointStamped
 */
void AStar::setStartPoint(geometry_msgs::PointStamped &s)
{
  start.header = s.header;
  start.point.x = floor(s.point.x / map->info.resolution);
  start.point.y = floor(s.point.y / map->info.resolution);
  start.point.z = floor(s.point.z / map->info.resolution);
}

/**
 * @brief set the end point for the path. The point is assumed to be in the map frame.
 *
 * @param g geometry_msgs::PointStamped
 */
void AStar::setGoalPoint(geometry_msgs::PointStamped &g)
{
  goal.header = g.header;
  goal.point.x = floor(g.point.x / map->info.resolution);
  goal.point.y = floor(g.point.y / map->info.resolution);
  goal.point.z = floor(g.point.z / map->info.resolution);
}

/**
 * @brief Store the occupancy grid map memory pointer.
 *
 * @param m const nav_msgs::OccupancyGrid::ConstPt
 */
void AStar::setMap(const nav_msgs::OccupancyGrid::ConstPtr &m) { map = m; }

/**
 * @brief Return the previously calculated path.
 *
 * @return nav_msgs::Path
 */
nav_msgs::Path AStar::getPath() { return path; }

/**
 * @brief Calculate the shortest path. This function is called after specifying the start and goal points.
 *
 * @return int : The cost of the path.
 */
int AStar::shortestPath()
{

  // Read the map parameters from the OGM message.
  int width = map->info.width;
  int height = map->info.height;
  float resolution = map->info.resolution;
  int n = width * height;
  int exploredCells = 0;

  // Data structures for the AStar (A*) algorithm
  // Store the points/nodes/gridcells with the specified sorted order. 
  priority_queue<point, vector<point>, greater<point>> pq;
  vector<float> cost(n, INF);   // Cost value for all the grid cells are set to infinity.
  vector<int> parent(n, -1);    // Set the parent value to each of the grid cell to be unknown (-1)

  // Return if the start point is outside the map
  if (start.point.x < 0 || start.point.x >= width || start.point.y < 0 ||
      start.point.y >= height)
    return 0;

  // Return if the goal point is outside the map
  if (goal.point.x < 0 || goal.point.x >= width || goal.point.y < 0 ||
      goal.point.y >= height)
    return 0;

  int sidx = start.point.x + start.point.y * height;
  cost[sidx] = 0;

  // Add the start point to the pirority queue
  point pt(start.point.x, start.point.y, cost[sidx]);
  pq.push(pt);

  while (!pq.empty())
  {
    // Get the top value from the pirority queue to explore it's neighbour
    point pt = pq.top();
    pq.pop();

    // If the current cell is the goal, stop searching
    if (pt.x == goal.point.x && pt.y == goal.point.y)
      break;

    // Find the neighbouring cells
    for (int r = -1; r <= 1; r++)
      for (int c = -1; c <= 1; c++)
      {

        // Skip if it's center cell
        if (r == 0 && c == 0)
          continue;

        // Skip if it's beyond map boundary
        int x = c + pt.x;
        int y = r + pt.y;
        if (x < 0 || y < 0 || x >= width || y >= height)
          continue;

        // Skip if it's a non-reachable cell, 0=free, -1=unknown, 100=occupied
        int nidx = x + y * height;
        if (map->data[nidx] == 100)
          continue;

        // Add the cell to the queue if new cost is smaller than prior cost
        int pidx = pt.x + pt.y * height;
        // Weighted cost in all direction, Diagonal are most costlier
        // The difference between Dijkstra and AStar is only the second term.
        // float ncost = pt.cost + sqrt(r * r + c * c) ;
        float ncost = pt.cost + sqrt(r * r + c * c) + sqrt(pow(x - goal.point.x, 2) + pow(y - goal.point.y, 2));
        if (ncost < cost[nidx])
        {
          // Update the cell cost, save it's parent, and add it to the queue
          cost[nidx] = ncost;
          point pt_n(x, y, ncost);
          parent[nidx] = pidx;
          pq.push(pt_n);
          exploredCells++;
        }
      }

  } // While the queue is not empty

  // the path is found if it's cost has been changed
  int gidx = goal.point.x + goal.point.y * height;
  if (cost[gidx] < INF)
  {
    path.header.seq = path.header.seq + 1;
    path.header.stamp = ros::Time();
    path.header.frame_id = map->header.frame_id;
    path.poses.clear(); // reset previous poses
    int idx = gidx, steps = 0;
    while (idx != -1)
    {
      geometry_msgs::PoseStamped pos;
      pos.header = path.header;

      // Covert current grid cell index to caresian coordinate
      pos.pose.position.x = (idx % width) * resolution;
      pos.pose.position.y = (idx / height) * resolution;
      pos.pose.position.z = 0;
      path.poses.push_back(pos);
      idx = parent[idx];
      steps++;
    }

    cout << "Explored Cells:" << exploredCells << endl;
    return steps;
  }
  else
  {
    return 0;
  }
}
