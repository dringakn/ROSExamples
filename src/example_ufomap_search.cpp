/**
 * Author: Dr. Ing. Ahmad Kamal Nasir
 * Email: dringakn@gmail.com
 * Modified:  30 June 2021
 * Description: UFOMap is an extension to the octomapping.
 *      It provides the explicit handling of unknown, free, and occupied space in a map.
 * Note:
 *      IMPORTANT:
 *      To use UFOMap in your package you need to add at the begining:
 *          - Add the following to your CMakeLists.txt
 *              - find_package(ufomap REQUIRED)
 *              - target_link_libraries(YOUR_NODE ${catkin_LIBRARIES} UFO::Map)
 *              - If your package also requires ufomap_msgs and/or ufomap_ros you can add them as standard catkin components
 *              - find_package(catkin REQUIRED COMPONENTS roscpp ufomap_msgs ufomap_ros)
 *          - Add the following to your package.xml
 *              - <build_depend>ufomap</build_depend>
 *              - <run_depend>ufomap</run_depend>
 *              - If your package also requires ufomap_msgs and/or ufomap_ros you can add them as standard catkin components
 *              - <build_depend>ufomap_msgs</build_depend>
 *              - <build_depend>ufomap_ros</build_depend>
 *              - <run_depend>ufomap_msgs</run_depend>
 *              - <run_depend>ufomap_ros</run_depend>
 *      In order to install:
 *          - mkdir /tmp
 *          - cd /tmp
 *          - git clone https://github.com/UnknownFreeOccupied/ufomap.git
 *          - cd ufomap/ufomap
 *          - mkdir build
 *          - cd build
 *          - cmake ..
 *          - make
 *          - sudo make install
 *          - In the CMakeLists.txt: find_package(ufomap REQUIRED)
 *
 *
 * Requirements:
 *      - sudo apt install libtbb-dev
 *      - sudo apt install python3-catkin-tools python3-osrf-pycommon
 *      - mkdir -p ~/catkin_ws/src
 *      - cd ~/catkin_ws
 *      - catkin init # Init Catkin workspace
 *      - catkin config --extend /opt/ros/noetic  # exchange noetic for your ros distro if necessary
 *      - catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # To enable release mode compiler optimzations
 *      - cd ~/catkin_ws/src
 *      - git clone git@github.com:UnknownFreeOccupied/ufomap.git
 *      - rosdep install --from-paths . --ignore-src -r -y
 *      - catkin build
 *      - source devel/setup.bash
 * Reference:
 *      https://github.com/UnknownFreeOccupied/ufomap/wiki
 * */

#include <ros/ros.h>                   // ROS stuff
#include <ufo/map/occupancy_map.h>     // UFO map
#include <ufomap_msgs/UFOMapStamped.h> // UFOMap ROS msg
#include <ufomap_msgs/conversions.h>   // To convert between UFO and ROS: msgToUfo
#include <ufomap_ros/conversions.h>    // To convert between UFO and ROS
#include <bits/stdc++.h>               // C++ stuff

using namespace std;

/**
 * @brief Get the value of the option flag if it exist.
 *  Example: char * filename = getCmdOption(argv, argv + argc, "-f");
 *
 * @param begin start of the list of string array, e.g. argv
 * @param end end of the list of string array, e.g. argv+argc
 * @param option command line switch, e.g. "-h"
 * @return char*
 */
char *getCmdOption(char **begin, char **end, const std::string &option)
{
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

/**
 * @brief Check if the option flag (switch) exists.
 *  Example: if(cmdOptionExists(argv, argv+argc, "-h")){...}
 *
 * @param begin start of the list of string array, e.g. argv
 * @param end end of the list of string array, e.g. argv+argc
 * @param option command line switch, e.g. "-h"
 * @return true
 * @return false
 */
bool cmdOptionExists(char **begin, char **end, const std::string &option)
{
    return std::find(begin, end, option) != end;
}

void printPoint3(ufo::map::Point3 pt, std::string prefix = "")
{
    cout << prefix << pt.x() << "," << pt.y() << "," << pt.z() << endl;
}

void printPoint(ufo::geometry::Point pt, std::string prefix = "")
{
    cout << prefix << pt.x() << "," << pt.y() << "," << pt.z() << endl;
}

void search_ufomap_whole_tree(ufo::map::OccupancyMap &map)
{
    // map.beginTreeBounding:
    // map.beginNNTree: iterate over specified types of nodes at specified depth near the specified point
    // map.changesBegin

    // map.beginTree: iterate over specified types of nodes at specified depth in complete tree.
    // It iterates over all the inner nodes
    // begin(xyz, occ, free, unkown, contain, depth)
    // beginTree: nodes from specified levels to maxlevels: (occ, free, unknow, contains, level)
    for (auto it = map.beginLeaves(true, false, false, true, 0); it != map.endLeaves(); ++it)
    {
        // it: containsUnknown(), containsFree(), containsOccupied(), getX(), getY(), getZ()
        // it: getDepth(), getSize(), hasChildren(), isPureLeaf(), getOccupancy()
        printPoint3(it.getCenter(), "Pt: ");
    }
}

void ufomap_search_leaves(ufo::map::OccupancyMap &map, bool occ = true, bool free = false, bool unknown = false, bool contains = true, unsigned int level = 0)
{
    // map.beginLeaves(): iterate over specified types of leaf nodes at specified depth
    // beginLeaves: only specified type of voxel at specified level: (occ, free, unknow, contains, level)
    for (auto it = map.beginLeaves(occ, free, unknown, contains, level); it != map.endLeaves(); ++it)
    {
        // it: containsUnknown(), containsFree(), containsOccupied(), getX(), getY(), getZ()
        // it: getDepth(), getSize(), hasChildren(), isPureLeaf(), getOccupancy()
        printPoint3(it.getCenter(), "Pt: ");
    }
}

bool isInCollision(ufo::map::OccupancyMap const &map,
                   ufo::geometry::BoundingVar const &bounding_volume,
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 0)
{
    // Iterate through all leaf nodes that intersects the bounding volume
    for (auto it = map.beginLeaves(bounding_volume, occupied_space, free_space, unknown_space, false, min_depth); it != map.endLeaves(); ++it)
        return true; // Is in collision since a leaf node intersects the bounding volume.
    return false;    // No leaf node intersects the bounding volume.
}

bool isCollisionFreePathExist(ufo::map::OccupancyMap const &map,
                              ufo::map::Point3 position, ufo::map::Point3 goal, double radius,
                              bool occupied_space = true, bool free_space = false, bool unknown_space = false,
                              ufo::map::DepthType min_depth = 0)
{
    // Check if it is possible to move between the robot's current position and goal

    ufo::math::Vector3 direction = goal - position;           // The direction from position to goal
    ufo::math::Vector3 center = position + (direction / 2.0); // The center between position and goal
    double distance = direction.norm();                       // The distance between position and goal
    direction /= distance;                                    // Normalize direction

    // Calculate yaw, pitch and roll for oriented bounding box
    double yaw = -atan2(direction[1], direction[0]);
    double pitch = -asin(direction[2]);
    double roll = 0; // TODO: Fix

    // Create an oriented bounding box between position and goal, with a size radius.
    ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, radius, radius), ufo::math::Quaternion(roll, pitch, yaw));

    if (isInCollision(map, obb, occupied_space, free_space, unknown_space, min_depth)) // Check if the oriented bounding box collides with specified space
    {
        std::cout << "The path between position and goal is not clear" << std::endl;
        return false;
    }
    else
    {
        std::cout << "It is possible to move between position and goal in a straight line" << std::endl;
        return true;
    }
}

bool findNeighbour(ufo::map::OccupancyMap const &map, ufo::map::Point3 position,
                   bool occupied_space = true, bool free_space = false, bool unknown_space = false,
                   ufo::map::DepthType min_depth = 0)
{
    // Find the closest occupied voxel to position at depth
    for (auto it = map.beginNNLeaves(position, occupied_space, free_space, unknown_space, false, min_depth); it != map.endNNLeaves(); ++it)
    {
        std::cout << "The closest occupied space to position is at " << it.getX() << " " << it.getY() << " " << it.getZ() << ", " << it.getDistance() << " meter away" << std::endl;
        // We do not have to break if we want X number of closest voxels. Here we only want the closest, so we break here.
        return true;
    }
    return false; // No neighbour node found.
}

bool findNeighbourBV(ufo::map::OccupancyMap const &map, ufo::geometry::BoundingVar const &bounding_volume, ufo::map::Point3 position,
                     bool occupied_space = true, bool free_space = false, bool unknown_space = false,
                     ufo::map::DepthType min_depth = 0)
{
    // Find the closest occupied voxel to position at depth 0
    for (auto it = map.beginNNLeaves(position, bounding_volume, occupied_space, free_space, unknown_space, false, min_depth); it != map.endNNLeaves(); ++it)
    {
        std::cout << "The closest occupied space to position is at " << it.getX() << " " << it.getY() << " " << it.getZ() << ", " << it.getDistance() << " meter away" << std::endl;
        // We do not have to break if we want X number of closest voxels. Here we only want the closest, so we break here.
        return true;
    }
    return false; // No or Not enough neighbour(s) found.
}

bool findKNeighbour(ufo::map::OccupancyMap const &map, ufo::map::Point3 position, unsigned int K,
                    bool occupied_space = true, bool free_space = false, bool unknown_space = false,
                    ufo::map::DepthType min_depth = 0)
{
    // Find the closest occupied voxel to position at depth 0 (2 cm)
    for (auto it = map.beginNNLeaves(position, occupied_space, free_space, unknown_space, false, min_depth); it != map.endNNLeaves(); ++it)
    {
        std::cout << "Position: " << it.getX() << " " << it.getY() << " " << it.getZ() << "Distance: " << it.getDistance() << " meter ";
        if (it.isOccupied())
        {
            std::cout << "The voxel is occupied" << std::endl;
        }
        else if (it.isUnknown())
        {
            std::cout << "The voxel is unknown" << std::endl;
        }
        if (0 > --K)
        {
            return true; // We have to break if we want K number of closest voxels.
        }
    }
    return false; // No or Not enough neighbour(s) found.
}

void populate_ufomap(ufo::map::OccupancyMap &map)
{
    /*
        Set the occupancy value at specified level (convert the prob to log odds): log(p/(1-p))
    */
    float res = map.getResolution();
    for (float z = -5; z < 5; z += res)
        for (float x = -5; x < 5; x += res)
            for (float y = -5; y < 5; y += res)
                map.updateOccupancy(x, y, z, 1, 1); // xyz, prob=1, depth=0
}

void check_ufomap_location(ufo::map::OccupancyMap &map)
{
    /*
        Check the state of the voxel at specified level.
    */
    map.containsFree(1, 1, 1, 0); // check state: code/xyz, depth=0
    /*
        Get the occupancy value at specified level (convert the logodds to the prob): 1 / 1 + e(-lo)
    */
    float prob = map.getOccupancy(1, 1, 1); // Get value: code/xyz, depth=0
}

void ufomap_stats(ufo::map::OccupancyMap &map)
{
    /*
        Print the UFO map information.
    */

    std::cout << " <<<<<<<<<<<<<<<<< UFOMap Stats >>>>>>>>>>>>>>>>> " << std::endl;
    std::cout << "getClampingThresMax: " << map.getClampingThresMax() << std::endl;
    std::cout << "getClampingThresMin: " << map.getClampingThresMin() << std::endl;
    std::cout << "getFileVersion: " << map.getFileVersion() << std::endl;
    std::cout << "getFreeThres: " << map.getFreeThres() << std::endl;
    std::cout << "getKnownBBX(Center): " << map.getKnownBBX().center.x() << "," << map.getKnownBBX().center.y() << "," << map.getKnownBBX().center.z() << std::endl;
    std::cout << "getKnownBBX(HalfSize): " << map.getKnownBBX().half_size.x() << "," << map.getKnownBBX().half_size.y() << "," << map.getKnownBBX().half_size.z() << std::endl;
    std::cout << "getMin: " << map.getMin().x() << "," << map.getMin().y() << "," << map.getMin().z() << std::endl;
    std::cout << "getMax: " << map.getMax().x() << "," << map.getMax().y() << "," << map.getMax().z() << std::endl;
    std::cout << "getMaxDepthLevels: " << map.getMaxDepthLevels() << std::endl;
    std::cout << "getMinDepthLevels: " << map.getMinDepthLevels() << std::endl;
    std::cout << "getNodeHalfSize: " << map.getNodeHalfSize(0) << std::endl;
    std::cout << "size: " << map.size() << std::endl;
    std::cout << "getNumInnerNodes: " << map.getNumInnerNodes() << std::endl;
    std::cout << "getNumLeafNodes: " << map.getNumLeafNodes() << std::endl;
    std::cout << "getOccupiedThres: " << map.getOccupiedThres() << std::endl;
    std::cout << "getProbHit: " << map.getProbHit() << std::endl;
    std::cout << "getProbMiss: " << map.getProbMiss() << std::endl;
    std::cout << "getResolution: " << map.getResolution() << std::endl;
    std::cout << "getTreeDepthLevels: " << map.getTreeDepthLevels() << std::endl;
    std::cout << "getTreeType: " << map.getTreeType() << std::endl;
    std::cout << "numChangedDetected: " << map.numChangedDetected() << std::endl;
    std::cout << "isChangeDetectionEnabled: " << map.isChangeDetectionEnabled() << std::endl;
    std::cout << "isAutomaticPruningEnabled: " << map.isAutomaticPruningEnabled() << std::endl;
    std::cout << "isMinMaxChangeDetectionEnabled: " << map.isMinMaxChangeDetectionEnabled() << std::endl;
    std::cout << "maxChange: " << map.maxChange().x() << "," << map.maxChange().y() << "," << map.maxChange().z() << std::endl;
    std::cout << "memoryUsage: " << map.memoryUsage() << std::endl;
    std::cout << "memoryUsageInnerNode: " << map.memoryUsageInnerNode() << std::endl;
    std::cout << "memoryUsageLeafNode: " << map.memoryUsageLeafNode() << std::endl;
    std::cout << " ================================================= " << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout.precision(5);                                  // Set the floating point percision for cout
    std::cout.setf(std::cout.showpos | std::cout.showpoint); // Show the leading +- and zero

    ros::init(argc, argv, "example_ufomap_save"); // Initialize the map
    ros::NodeHandle nh;                           // Create the node handle

    /*
        Create a non colored UFO map with minimum resolution of 0.25m and with 16 levels (Max: 21).
        Each level has double the size of voxel. Thus the extreme size of the enviornment can be
        from (-8192m, -8192m, -8192m) to (8192m, 8192m, 8192m).
    */
    float res = 0.25;
    unsigned int nlevels = 16;
    bool occ = cmdOptionExists(argv, argv + argc, "-occ");
    bool free = cmdOptionExists(argv, argv + argc, "-free");
    bool unknown = cmdOptionExists(argv, argv + argc, "-unknown");
    bool contains = cmdOptionExists(argv, argv + argc, "-contains");
    unsigned int neighbours = 1; // atoi(getCmdOption(argv, argv + argc, "-neighbours"));

    ROS_INFO("occ: %d, free: %d, unknown: %d, contains: %d, neighbours: %d", occ, free, unknown, contains, neighbours);

    ufo::map::OccupancyMap map(res, nlevels); // Resolution, levels, prune?, th_occ, th_free, p_hit, p_miss, clamp_th_min, clm_th_max

    // Read map from the topic
    ufomap_msgs::UFOMapStamped::ConstPtr msg = ros::topic::waitForMessage<ufomap_msgs::UFOMapStamped>("ufomap", ros::Duration(10)); // Wait 10Sec for the msg.

    // Convert the msg to map
    if (ufomap_msgs::msgToUfo(msg->map, map))
    {
        ufo::geometry::Sphere sphere(ufo::geometry::Point(10, 10, 10), 10);
        std::cout << "isInCollision? " << isInCollision(map, sphere, occ, free, unknown, 0) << std::endl;

        ufo::geometry::Point position(0.0, 0.0, 0.0); // The robot's current position
        ufo::geometry::Point goal(10.0, 5.0, 0.0);    // The goal, where the robot wants to move
        double radius = 0.5;                        // The robot's size (radius)
        std::cout << "isCollisionFreePathExist? " << isCollisionFreePathExist(map, position, goal, radius, occ, free, unknown, 0) << std::endl;

        std::cout << "findNeighbour? " << findNeighbour(map, position, occ, free, unknown, 0) << std::endl;
        // std::cout << "findKNeighbour? " << findKNeighbour(map, position, 3, occ, free, unknown, 0) << std::endl;
        std::cout << "findNeighbourVB? " << findNeighbourBV(map, sphere, position, occ, free, unknown, 0) << std::endl;

        // ufomap_search_leaves(map, occ, free, unknown, contains, 0);
        // ufomap_search_neighbours(map, ufo::map::Point3(0, 0, 0), neighbours, occ, free, unknown, contains, 0);
    }

    // ufomap_stats(map); // Print stats

    return 0;
}
