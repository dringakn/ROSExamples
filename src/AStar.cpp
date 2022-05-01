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

AStar::AStar() : gcost{
                     {INF, 1, INF}, // Initialize the cost matrix, Manhatten style move
                     {1, 0, 1},
                     {INF, 1, INF}}
{
}

AStar::~AStar() {}

/**
 * @brief Set the start point for the path. The point is assumed to be in the map frame. The
 *        input point is converted to grid cell location.
 *
 * @param s geometry_msgs::PointStamped
 */
void AStar::setStartPoint(geometry_msgs::PointStamped &s)
{
  start.header = s.header;
  start.point.x = floor(s.point.x / map->info.resolution);
  start.point.y = floor(s.point.y / map->info.resolution);
  start.point.z = floor(s.point.z / map->info.resolution);
  // std::cout << start.point << std::endl;
}

/**
 * @brief Set the end point for the path. The point is assumed to be in the map frame.The
 *        input point is converted to grid cell location.
 *
 * @param g geometry_msgs::PointStamped
 */
void AStar::setGoalPoint(geometry_msgs::PointStamped &g)
{
  goal.header = g.header;
  goal.point.x = floor(g.point.x / map->info.resolution);
  goal.point.y = floor(g.point.y / map->info.resolution);
  goal.point.z = floor(g.point.z / map->info.resolution);
  // std::cout << goal.point << std::endl;
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
  priority_queue<point, vector<point>, greater<point>> pq; // smallest element on the top.
  vector<float> cost(n, INF);                              // Cost value for all the grid cells are set to infinity.
  vector<int> parent(n, -1);                               // Set the parent value to each of the grid cell to be unknown (-1)

  // Return if the start point is outside the map
  if ((start.point.x < 0) || (start.point.x >= width) || (start.point.y < 0) || (start.point.y >= height))
  {
    ROS_INFO("AStar start point is outside the map.");
    return 0;
  }

  // Return if the goal point is outside the map
  if ((goal.point.x < 0) || (goal.point.x >= width) || (goal.point.y < 0) || (goal.point.y >= height))
  {
    ROS_INFO("AStar goal point is outside the map.");
    return 0;
  }

  // start point 1D array index = col_index + (row_index x total_columns)
  int sidx = start.point.x + (start.point.y * width);
  cost[sidx] = 0;

  // Add the start point to the pirority queue (col, row)
  point pt(start.point.x, start.point.y, cost[sidx]);
  pq.push(pt);

  while (!pq.empty())
  {
    // Get the top value from the pirority queue to explore it's neighbour
    point pt = pq.top();
    pq.pop();

    // If the current cell is the goal, stop searching
    if ((pt.x == goal.point.x) && (pt.y == goal.point.y))
      break;

    /*
      Find the neighbouring cells (3x3): r=row=y-axis, c=col=x-axis
      Generating all the 8 successor of this cell

          NW[0]   N[1]     NE[2]
            \      |      /
              \    |    /
                \  |  /
          W[3]----Cell[4]----E[5]
                /  |  \
              /    |    \
            /      |      \
          SW[6]   S[7]     SE[8]

      Cell--> Popped Cell (r,   c) --> index:4
      N --> North       (r-1, c)   --> index:1
      S --> South       (r+1, c)   --> index:7
      E --> East        (r,   c+1) --> index:5
      W --> West        (r,   c-1) --> index:3
      NE--> North-East  (r-1, c+1) --> index:2
      NW--> North-West  (r-1, c-1) --> index:0
      SE--> South-East  (r+1, c+1) --> index:8
      SW--> South-West  (r+1, c-1) --> index:6
    */
    for (int r = -1; r <= 1; r++)   // y-axis
      for (int c = -1; c <= 1; c++) // x-axis
      {

        // Skip if it's center cell
        if ((r == 0) && (c == 0))
          continue;

        // Skip if it's beyond map boundary
        int y = r + pt.y;
        int x = c + pt.x;
        if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
          continue;

        // Skip if it's a non-reachable cell, 0=free, -1=unknown, 100=occupied
        int nidx = x + y * width;
        if ((map->data[nidx] == 100) || (map->data[nidx] == -1))
          continue;

        // Weighted cost in all direction, Diagonal are most costlier
        // new cost = previous cost + motioncost (Euclidean distance) + heuristic
        // motioncast: cost to move from current to the neighbour; sqrt(r * r + c * c) or gcost(r,c)
        // heuristic: approximate cost to move from neighbour to the target
        // Note: The Euclidean distance for diagonal is more than the longitudinal/lateral
        //       <<< A 3x3 matrix can also be used for custom cost >>>
        //       The difference between Dijkstra and AStar is only the second term (heuristic).
        float ncost = pt.cost + gcost[r + 1][c + 1] + sqrt(pow(x - goal.point.x, 2) + pow(y - goal.point.y, 2));
        int pidx = pt.x + pt.y * width;
        if (ncost < cost[nidx])         // Add the cell to the queue if new cost is smaller than prior cost
        {
          // Update the cell cost, save it's parent, and add it to the queue
          cost[nidx] = ncost;      // Update the cost value
          point pt_n(x, y, ncost); // Create the node
          parent[nidx] = pidx;     // Set the explored cell parent (current cell)
          pq.push(pt_n);           // Store the node (minimum value will automatically come to top)
          exploredCells++;         // Increment the explored cells counter
        }
      }

  } // While the queue is not empty

  
  int gidx = goal.point.x + goal.point.y * width;
  if (cost[gidx] < INF) // the path is found if it's cost has been changed
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
      pos.pose.position.y = (idx / width) * resolution;
      pos.pose.position.z = 0;
      path.poses.push_back(pos);
      idx = parent[idx];
      steps++;
      // std::cout << idx << " -> " << pos.pose.position.x << ", " << pos.pose.position.y << std::endl;
    }

    cout << "Explored Cells:" << exploredCells << endl;
    return steps;
  }
  else
  {
    return 0;
  }
}
