/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *      Given a start point, end point and an UFO (Unknown, Free, Occupied) map find the shortest path using A* algorithm.
 *      The input points are of standard ROS message type geometry_msgs::PointStamped and the UFO is of type
 *      ufo::map::OccupancyMap.
 *
 *      The output path is of type nav_msgs::Path.
 **/
#include <cstddef>                                   // nullptr
#include <geometry_msgs/Point.h>                     // Start/Goal Point
#include <geometry_msgs/Pose.h>                      // Map origin
#include <geometry_msgs/PoseStamped.h>               // Goal point: move_base/goal
#include <geometry_msgs/PoseWithCovarianceStamped.h> // Start point: /initialpose
#include <iostream>                                  // cout, cin
#include <ufo/map/occupancy_map.h>                   // ufo map
#include <ufo/map/occupancy_map_color.h>             // ufo color map
#include <ufomap_msgs/UFOMapMetaData.h>              // ufo map meta data
#include <ufomap_msgs/UFOMapStamped.h>               // UFOMap ROS msg
#include <ufomap_msgs/conversions.h>                 // To convert between UFO and ROS
#include <ufomap_ros/conversions.h>                  // To convert between UFO and ROS
#include <nav_msgs/Path.h>                           // Resultant path
#include <queue>                                     // priority_queue
#include <random_numbers/random_numbers.h>           // random number
#include <ros/ros.h>                                 // ros functionality
#include <tf/transform_listener.h>                   // Transform listener

using namespace std;

#define INF FLT_MAX
/**
 * @brief Intermediate data structure to store voxel cell or node information.
 *        The members includes x(int), y(int), z(int), cost(float). Overloaded with
 *        ">" and "<" operators to compare cost value.
 *
 */

template <typename Key, typename Priority>
class MyNode
{
public:
    Key key;
    Priority priority;
    Priority g;
    ufo::geometry::Point pos;

    MyNode()
    {
        this->key = 0;
        this->g = this->priority = INF;
    }

    MyNode(Key k, Priority p)
    {
        this->key = k, this->priority = p;
        this->g = INF;
    }

    friend bool operator<(const MyNode &lhs, const MyNode &rhs)
    {
        return lhs.priority > rhs.priority;
    }

    friend bool operator>(const MyNode &lhs, const MyNode &rhs)
    {
        return lhs.priority < rhs.priority;
    }

    friend std::ostream &operator<<(std::ostream &os, const MyNode &n)
    {
        os << n.key << ": " << n.priority;
        return os;
    }
};

template <typename Key, typename Priority>
class MyQueue
{
public:                                      // Change to protected
    std::unordered_map<Key, size_t> id;      // HeapID of the Key
    std::vector<MyNode<Key, Priority>> heap; // Heap for sorted nodes
    std::unordered_map<Key, bool> visited;   // Table for visited node lookup
    std::unordered_map<Key, Key> parent;     // Table to store Parent ID

public:
    MyQueue() {}
    void clear() { heap.clear(), id.clear(), visited.clear(), parent.clear(); }
    bool empty() const { return heap.empty(); }
    std::size_t size() const { return heap.size(); }
    const MyNode<Key, Priority> &top() const { return heap.front(); }
    bool isVisited(Key key) { return visited.find(key) != visited.end(); }

    void push(MyNode<Key, Priority> &node)
    {
        size_t n = heap.size();
        id[node.key] = n;
        heap.push_back(node);
        shiftUp(n);
    }

    MyNode<Key, Priority> pop()
    {
        if (size() == 0)
            return MyNode<Key, Priority>();

        MyNode<Key, Priority> ret = std::move(*heap.begin());
        id.erase(ret.key);
        if (size() > 1)
            *heap.begin() = std::move(*(heap.end() - 1));

        heap.pop_back();
        shiftDown(0);

        return ret;
    }

    int pushOrUpdate(MyNode<Key, Priority> &node)
    {
        // Add if doesn't exist
        if (id.find(node.key) == id.end())
        {
            push(node);
            return 1;
        }

        // Update if node g-cost is smaller than existing node g-cost
        size_t heappos = id[node.key];
        if (node.g < heap[heappos].g)
        {
            heap[heappos].g = node.g;     // Update g-cost
            heap[heappos].pos = node.pos; // Update position
            if (node.priority > heap[heappos].priority)
            {
                heap[heappos].priority = node.priority; // Update priority, f-cost
                shiftUp(heappos);
            }
            else if (node.priority < heap[heappos].priority)
            {
                heap[heappos].priority = node.priority; // Update priority, f-cost
                shiftDown(heappos);
            }
            return 2;
        }
        else
        {
            return 0;
        }
    }

    bool set_priority(const Key &key, const Priority &new_priority)
    {
        bool result = false;
        if (id.find(key) == id.end())
            return result;

        size_t heappos = id[key];
        Priority &priority = heap[heappos].priority;
        if (new_priority > priority)
        {
            priority = new_priority;
            shiftUp(heappos);
            result = true;
        }
        else if (new_priority < priority)
        {
            priority = new_priority;
            shiftDown(heappos);
            result = true;
        }
        return result;
    }

private:
    void shiftUp(size_t heappos)
    {
        size_t len = heap.size();
        if (len < 2 || heappos <= 0)
            return;

        size_t parent = (heappos - 1) >> 1;
        if (!(heap[heappos] > heap[parent]))
            return;

        MyNode<Key, Priority> val = std::move(heap[heappos]);
        do
        {
            heap[heappos] = std::move(heap[parent]);
            id[heap[heappos].key] = heappos;
            heappos = parent;
            if (heappos <= 0)
                break;
            parent = (parent - 1) >> 1;
        } while (val > heap[parent]);

        heap[heappos] = std::move(val);
        id[heap[heappos].key] = heappos;
    }

    void shiftDown(size_t heappos)
    {
        size_t len = heap.size();
        size_t child = (heappos << 1) + 1;
        if (len < 2 || child >= len)
            return;

        if (child + 1 < len && heap[child + 1] > heap[child])
            ++child;

        if (!(heap[child] > heap[heappos]))
            return;

        MyNode<Key, Priority> val = std::move(heap[heappos]);
        do
        {
            heap[heappos] = std::move(heap[child]);
            id[heap[heappos].key] = heappos;
            heappos = child;
            child = (child << 1) + 1;
            if (child >= len)
                break;

            if (child + 1 < len && heap[child + 1] > heap[child])
                ++child;

        } while (heap[child] > val);

        heap[heappos] = std::move(val);
        id[heap[heappos].key] = heappos;
    }
};

template <typename Key, typename Priority>
class AStar3D
{
private:
    nav_msgs::Path path;                                      // Resultant path
    MyNode<Key, Priority> start;                              // Starting location
    MyNode<Key, Priority> goal;                               // Goal location
    ufo::map::DepthType min_depth;                            // UFO depth level for search
    bool occupied_space, free_space, unknown_space, contains; // UFO search flags
    double search_distance;                                   // Nearest neighbourhood distance [m]
    MyQueue<Key, Priority> queue;                             // Customized priority queue, smallest element on the top.

public:
    ufo::map::OccupancyMap map; // The UFO map

    bool findNearestVoxel(ufo::map::Point3 position, MyNode<Key, Priority> *result)
    {
        bool success = false; // No neighbour found.
        double dist = INF, temp;
        MyNode<Key, Priority> pt;
        pt.pos = position;
        for (const auto &n : this->findNeighbours(pt))
        {
            temp = position.distance(n.second);
            if (temp < dist)
            {
                dist = temp;
                result->key = n.first;
                result->pos = n.second;
                success = true;
            }
        }
        return success;
    }

    std::vector<std::pair<Key, ufo::map::Point3>> findNeighbours(MyNode<Key, Priority> position)
    {
        std::vector<std::pair<Key, ufo::map::Point3>> result;
        double res = map.getNodeSize(this->min_depth);
        for (double x = position.pos.x() - this->search_distance; x <= position.pos.x() + this->search_distance; x += res)
            for (double y = position.pos.y() - this->search_distance; y <= position.pos.y() + this->search_distance; y += res)
                for (double z = position.pos.z() - this->search_distance; z <= position.pos.z() + this->search_distance; z += res)
                {
                    ufo::map::Point3 pt(x, y, z);
                    ufo::map::Code code = map.toCode(pt, this->min_depth);
                    Key key = code.getCode();
                    if (key == position.key)
                        continue;

                    if (this->free_space && map.isFree(code))
                        result.push_back(make_pair(key, pt));
                    else if (this->unknown_space && map.isUnknown(code))
                        result.push_back(make_pair(key, pt));
                }

        return result;
    }

    void printUFOPoint(ufo::map::Point3 &pt, std::string prefix = "")
    {
        ROS_INFO("%s[%+5.2f, %+5.2f, %+5.2f]", prefix.c_str(), pt.x(), pt.y(), pt.z());
    }

    void printNode(MyNode<Key, Priority> &pt, std::string prefix = "")
    {
        ROS_INFO("%s %015lu[%+5.2f, %+5.2f, %+5.2f] -> %+5.2f [%5.2f]", prefix.c_str(), pt.key, pt.pos.x(), pt.pos.y(), pt.pos.z(), pt.priority, pt.g);
    }

    AStar3D(bool occupied_space, bool free_space, bool unknown_space, bool contains, ufo::map::DepthType min_depth, double nn_distance) : map(1) // Constructor (occ, free, unknown, depth)
    {
        this->contains = contains;             // Bounding box with contains
        this->search_distance = nn_distance;   // Nearest neighbourhood distance [m]
        this->occupied_space = occupied_space; // Flag to enable/disable to include occupied voxels in the search.
        this->free_space = free_space;         // Flag to enable/disable to include free voxels in the search.
        this->unknown_space = unknown_space;   // Flag to enable/disable to include unknown voxels in the search.
        this->min_depth = min_depth;           // UFO depth level for search
        std::cout << "Occ, Free, Unknown, Depth <-> " << this->occupied_space << "," << this->free_space << "," << this->unknown_space << "," << this->min_depth << std::endl;
    }

    virtual ~AStar3D() {} // Destructor

    /**
     * @brief Set the start point for the path. The point is assumed to be in the map frame.
     *        The input point is converted to voxel cell location.
     *
     * @param pt geometry_msgs::PointStamped
     * @return true if a nearby voxel is found and set as starting point.
     * @return false otherwise
     */
    bool setStartPoint(geometry_msgs::PointStamped &pt) // Set starting locatoin
    {
        ufo::map::Point3 in(pt.point.x, pt.point.y, pt.point.z);
        MyNode<Key, Priority> out;
        bool result = false;
        if (findNearestVoxel(in, &out))
        {
            this->printUFOPoint(in, "Start point: ");
            this->printNode(out, "Nearest voxel to start point: ");
            this->start = out;
            result = true;
        }
        return result;
    }

    /**
     * @brief Set the goal point for the path. The point is assumed to be in the map frame.
     *        The input point is converted to voxel cell location.
     *
     * @param pt geometry_msgs::PointStamped
     * @return true if a nearby voxel is found and set as goal point.
     * @return false otherwise
     */
    bool setGoalPoint(geometry_msgs::PointStamped &pt) // Set goal location
    {
        ufo::map::Point3 in(pt.point.x, pt.point.y, pt.point.z);
        MyNode<Key, Priority> out;
        bool result = false;
        if (findNearestVoxel(in, &out))
        {
            this->printUFOPoint(in, "Goal point: ");
            this->printNode(out, "Nearest voxel to goal point: ");
            this->goal = out;
            result = true;
        }
        return result;
    }

    /**
     * @brief Return the previously calculated path.
     *
     * @return nav_msgs::Path
     */
    nav_msgs::Path getPath() // Get the previous calculated path
    {
        path.header.stamp = ros::Time();
        return path;
    }

    size_t reconstuctPath(Key &current)
    {
        path.header.seq = path.header.seq + 1;
        path.header.stamp = ros::Time();
        path.header.frame_id = "map"; // map->header.frame_id;
        path.poses.clear();           // reset previous poses
        geometry_msgs::PoseStamped pos;
        pos.header = path.header;
        Key predecessor = current;
        do
        {
            ufo::map::Point3 pt = map.toCoord(ufo::map::Code(predecessor, this->min_depth));
            pos.pose.position.x = pt.x();
            pos.pose.position.y = pt.y();
            pos.pose.position.z = pt.z();
            path.poses.push_back(pos);
            predecessor = queue.parent[predecessor];
        } while (predecessor != 0);

        return path.poses.size();
    }

    /**
     * @brief Calculate the shortest path. This function is called after specifying the start and goal points.
     *
     * @return int : The cost of the path.
     */
    int shortestPath() // Search path
    {

        // Read the map parameters from the UFOMap.
        int visitedNodes = 0;
        int discardedNodes = 0;
        int neighbourNodes = 0;
        int updatedNodes = 0;
        int unchangedNodes = 0;
        int ctr = 0;
        MyNode<Key, Priority> current, neighbour;

        // Data structures for the AStar3D (A*) algorithm
        queue.clear();

        // Return if the start point is invalid.
        if (start.key == 0)
        {
            ROS_INFO("AStar3D start point is not specified.");
            return -1;
        }

        // Return if the goal point is invalid.
        if (goal.key == 0)
        {
            ROS_INFO("AStar3D goal point is not specified.");
            return -1;
        }

        ROS_INFO("Searching goal using AStar3D...");

        // Add the start the point to the queue to start the process
        start.g = 0;
        start.priority = start.g + start.pos.distance(goal.pos);
        queue.push(start);

        while (!queue.empty())
        {
            // Get the top value from the pirority queue to explore it's neighbour
            current = queue.pop();

            // If the current cell is the goal, stop searching
            if (current.key == goal.key)
            {
                // this->printNode(current, "GOAL            : ");
                ROS_INFO("<<<FOUND>>> Iterations:%d, Visited:%d, Discarded:%d, NeighboursFound:%d, CostUpdated:%d, CostUnchanged:%d", ctr, visitedNodes, discardedNodes, neighbourNodes, updatedNodes, unchangedNodes);
                return this->reconstuctPath(current.key);
            }

            queue.visited[current.key] = true; // Add to the visited queue
            visitedNodes++;

            // Find the nearest voxel by Expansion
            for (const auto &n : this->findNeighbours(current))
            {
                ctr++;
                neighbour.key = n.first;

                if (queue.isVisited(neighbour.key))
                {
                    discardedNodes++;
                    continue; // Neighbour is the current node or already visited
                }

                neighbour.pos = n.second;
                neighbour.g = current.g + current.pos.distance(neighbour.pos);       // Movement cost
                neighbour.priority = neighbour.g + neighbour.pos.distance(goal.pos); // Heuristic cost
                int ret = queue.pushOrUpdate(neighbour);
                if (ret == 1)
                {
                    queue.parent[neighbour.key] = current.key; // Set parent Parent
                    // this->printNode(neighbour, "QUEUED          : ");
                    neighbourNodes++;
                }
                else if (ret == 2)
                {
                    queue.parent[neighbour.key] = current.key; // Set parent Parent
                    // this->printNode(neighbour, "COST UPDATED    : ");
                    updatedNodes++;
                }
                else
                {
                    // this->printNode(neighbour, "COST NOT UPDATED: ");
                    unchangedNodes++;
                }
            }
            ROS_INFO_THROTTLE(10, "<<<SEARCHING>>> Iterations:%d, Visited:%d, Discarded:%d, NeighboursFound:%d, CostUpdated:%d, CostUnchanged:%d", ctr, visitedNodes, discardedNodes, neighbourNodes, updatedNodes, unchangedNodes);
        }
        ROS_INFO("<<<NOT FOUND>>> Iterations:%d, Visited:%d, Discarded:%d, NeighboursFound:%d, CostUpdated:%d, CostUnchanged:%d", ctr, visitedNodes, discardedNodes, neighbourNodes, updatedNodes, unchangedNodes);
        return -1;
    }
};