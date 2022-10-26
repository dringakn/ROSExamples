/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Date: 14 June 2022
 *    Description:
 *      Given a start point, end point and an UFO (Unknown, Free, Occupied) map find the shortest path using A* algorithm.
 *      The input points are of standard ROS message type geometry_msgs::PointStamped and the UFO is of type
 *      ufo::map::OccupancyMap.
 *
 *      The output path is of type nav_msgs::Path.
 **/
#include <bits/stdc++.h>                             // C++ functionality
#include <ros/ros.h>                                 // ros functionality
#include <geometry_msgs/PointStamped.h>              // Goal point: move_base/goal
#include <geometry_msgs/PoseWithCovarianceStamped.h> // Start point: /initialpose
#include <nav_msgs/Path.h>                           // Resultant path
#include <ufo/map/occupancy_map.h>                   // ufo map
#include <ufo/map/occupancy_map_color.h>             // ufo color map
#include <ufomap_msgs/UFOMapMetaData.h>              // ufo map meta data
#include <ufomap_msgs/UFOMapStamped.h>               // UFOMap ROS msg
#include <ufomap_msgs/conversions.h>                 // To convert between UFO and ROS
#include <ufomap_ros/conversions.h>                  // To convert between UFO and ROS

using namespace std;
#define INF FLT_MAX

/**
 * @brief Intermediate data structure to store voxel cell or node information.
 *        Overloaded with ">" and "<" operators to compare cost value.
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

/**
 * @brief Datastructure designed for the AStar calculations.
 */
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

    /**
     * @brief To Check if the the Node is already visited.
     *
     * @param key NodeID
     * @return true if already visited
     * @return false otherwise
     */
    bool isVisited(Key key) { return visited.find(key) != visited.end(); }

    /**
     * @brief Add the node to the data.
     *
     * @param node
     */
    void push(MyNode<Key, Priority> &node)
    {
        size_t n = heap.size();
        id[node.key] = n;
        heap.push_back(node);
        shiftUp(n);
    }

    /**
     * @brief Remove the next node to be processed
     *
     * @return MyNode<Key, Priority>
     */
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

    /**
     * @brief Add or update the node in the data.
     *
     * @param node the node to be processed.
     * @return int 1 if inserted, 2 if updated, 0 otherwise.
     */
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

/**
 * @brief The 3D AStar path planning object.
 */
template <typename Key, typename Priority>
class AStar3D
{
private:
    nav_msgs::Path path;
    MyNode<Key, Priority> start;
    MyNode<Key, Priority> goal;
    ufo::map::DepthType search_depth;
    bool occupied_space, free_space, unknown_space, contains, abort;
    double search_distance;
    double min_z;
    MyQueue<Key, Priority> queue;

public:
    /**
     * @brief The UFO map.
     *
     */
    ufo::map::OccupancyMap map;
    std::string map_frame;

    /**
     * @brief Set the Minimum Z value for search
     *
     * @param depth new minimum search value [-Inf to +Inf]
     */
    void setSearchMinZ(double minZ)
    {
        this->min_z = minZ;
    }

    /**
     * @brief Set the Search Depth object
     *
     * @param depth new depth value [0-21]
     */
    void setSearchDepth(ufo::map::DepthType depth)
    {
        this->search_depth = depth;
        this->search_distance = map.getNodeSize(this->search_depth);
        ROS_INFO("Occ[%d], Free[%d], Unknown[%d], Depth[%d][%f]", this->occupied_space, this->free_space, this->unknown_space, this->search_depth, this->search_distance);
    }

    /**
     * @brief Abort the search if it's in prgoress.
     */
    void abortSearch()
    {
        this->abort = true;
    }

    /**
     * @brief Find the nearest neighbour of the specified node.
     *
     * @param position input location
     * @return std::vector<std::pair<Key, ufo::map::Point3>> list of neighbours
     */
    std::vector<std::pair<Key, ufo::map::Point3>> findNeighbours(MyNode<Key, Priority> position)
    {
        std::vector<std::pair<Key, ufo::map::Point3>> result;
        double res = map.getNodeSize(this->search_depth);
        for (double x = position.pos.x() - this->search_distance; x <= position.pos.x() + this->search_distance; x += res)
            for (double y = position.pos.y() - this->search_distance; y <= position.pos.y() + this->search_distance; y += res)
                for (double z = position.pos.z() - this->search_distance; z <= position.pos.z() + this->search_distance; z += res)
                {
                    if (z <= this->min_z)
                        continue;

                    ufo::map::Point3 pt(x, y, z);
                    ufo::map::Code code = map.toCode(pt, this->search_depth);
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

    /**
     * @brief Get the Nearest Node object
     *
     * @param position input location
     * @param result output node
     * @return true if a nearest point is found
     * @return false otherwise
     */
    bool getNearestNode(ufo::map::Point3 position, MyNode<Key, Priority> *result)
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

    /**
     * @brief print the UFO location.
     *
     * @param pt UFO map locatoin
     * @param prefix Prefix to be printed
     */
    void printUFOPoint(ufo::map::Point3 &pt, std::string prefix = "")
    {
        ROS_INFO("%s[%+5.2f, %+5.2f, %+5.2f]", prefix.c_str(), pt.x(), pt.y(), pt.z());
    }

    /**
     * @brief print the Node
     *
     * @param pt Input node
     * @param prefix Prefix to be printed
     */
    void printNode(MyNode<Key, Priority> &pt, std::string prefix = "")
    {
        ROS_INFO("%s %015lu[%+5.2f, %+5.2f, %+5.2f]", prefix.c_str(), pt.key, pt.pos.x(), pt.pos.y(), pt.pos.z());
    }

    /**
     * @brief Construct a new AStar3D object
     *
     * @param occupied_space enable/disable occupied voxel to be searched.
     * @param free_space enable/disable free voxel to be searched.
     * @param unknown_space enable/disable unknown voxel to be searched.
     * @param depth search depth
     */
    AStar3D(bool occupied_space, bool free_space, bool unknown_space, ufo::map::DepthType depth) : map(1)
    {
        this->abort = false;
        this->contains = false;
        this->occupied_space = occupied_space;
        this->free_space = free_space;
        this->unknown_space = unknown_space;
        this->min_z = 0;
        this->map_frame = "map";
        this->setSearchDepth(depth);
        ROS_INFO("Occ[%d], Free[%d], Unknown[%d], Depth[%d][%f]", this->occupied_space, this->free_space, this->unknown_space, this->search_depth, this->search_distance);
    }

    virtual ~AStar3D() {}

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
        if (getNearestNode(in, &out))
        {
            if (out.pos.z() > this->min_z)
            {
                this->printUFOPoint(in, "Start point: ");
                this->printNode(out, "Nearest voxel to start point: ");
                this->start = out;
                result = true;
            }
            else
            {
                this->printNode(out, "Failed to set start voxel, it's below minimum z.");
            }
        }
        else
        {
            this->printUFOPoint(in, "Failed to set start point: ");
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
        if (getNearestNode(in, &out))
        {
            if (out.pos.z() > this->min_z)
            {
                this->printUFOPoint(in, "Goal point: ");
                this->printNode(out, "Nearest voxel to goal point: ");
                this->goal = out;
                result = true;
            }
            else
            {
                this->printNode(out, "Failed to set goal voxel, it's below minimum z.");
            }
        }
        else
        {
            this->printUFOPoint(in, "Failed to set goal point: ");
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
        path.header.seq = path.header.seq + 1;
        path.header.stamp = ros::Time();
        path.header.frame_id = this->map_frame;
        return path;
    }

    /**
     * @brief Reconstruct the path
     *
     * @param current current node
     * @return size_t number of nodes to be traversed
     */
    size_t reconstuctPath(Key &current)
    {
        path.header.seq = path.header.seq + 1;
        path.header.stamp = ros::Time();
        path.header.frame_id = this->map_frame;
        path.poses.clear();
        geometry_msgs::PoseStamped pos;
        pos.header = path.header;
        pos.pose.orientation.x = pos.pose.orientation.y = pos.pose.orientation.z = 0;
        pos.pose.orientation.w = 1;
        Key predecessor = current;
        do
        {
            ufo::map::Point3 pt = map.toCoord(ufo::map::Code(predecessor, this->search_depth));
            pos.pose.position.x = pt.x();
            pos.pose.position.y = pt.y();
            pos.pose.position.z = pt.z();
            path.poses.push_back(pos);
            predecessor = queue.parent[predecessor];
        } while (predecessor != 0);

        // Reverse the path: start to end.
        std::reverse(path.poses.begin(), path.poses.end());

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
            current = queue.pop();

            if (current.key == goal.key)
            {
                ROS_INFO("<<<FOUND>>> Iterations:%d, Visited:%d, Discarded:%d, NeighboursFound:%d, CostUpdated:%d, CostUnchanged:%d", ctr, visitedNodes, discardedNodes, neighbourNodes, updatedNodes, unchangedNodes);
                return this->reconstuctPath(current.key);
            }

            if (this->abort)
            {
                this->abort = false;
                ROS_INFO("<<<ABORT>>> Iterations:%d, Visited:%d, Discarded:%d, NeighboursFound:%d, CostUpdated:%d, CostUnchanged:%d", ctr, visitedNodes, discardedNodes, neighbourNodes, updatedNodes, unchangedNodes);
                return this->reconstuctPath(current.key);
            }

            queue.visited[current.key] = true;
            visitedNodes++;

            for (const auto &n : this->findNeighbours(current))
            {
                ctr++;
                neighbour.key = n.first;

                if (queue.isVisited(neighbour.key))
                {
                    discardedNodes++;
                    continue;
                }

                neighbour.pos = n.second;
                neighbour.g = current.g + current.pos.distance(neighbour.pos);       // Movement cost
                neighbour.priority = neighbour.g + neighbour.pos.distance(goal.pos); // Heuristic cost
                int ret = queue.pushOrUpdate(neighbour);
                if (ret == 1)
                {
                    queue.parent[neighbour.key] = current.key;
                    neighbourNodes++;
                }
                else if (ret == 2)
                {
                    queue.parent[neighbour.key] = current.key;
                    updatedNodes++;
                }
                else
                {
                    unchangedNodes++;
                }
            }
            ROS_INFO_THROTTLE(10, "<<<SEARCHING>>> Iterations:%d, Visited:%d, Discarded:%d, NeighboursFound:%d, CostUpdated:%d, CostUnchanged:%d", ctr, visitedNodes, discardedNodes, neighbourNodes, updatedNodes, unchangedNodes);
        }
        ROS_INFO("<<<NOT FOUND>>> Iterations:%d, Visited:%d, Discarded:%d, NeighboursFound:%d, CostUpdated:%d, CostUnchanged:%d", ctr, visitedNodes, discardedNodes, neighbourNodes, updatedNodes, unchangedNodes);
        return -1;
    }
};