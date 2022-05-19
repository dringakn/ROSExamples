/**
 * Author: Dr. Ing. Ahmad Kamal Nasir
 * Email: dringakn@gmail.com
 * Modified:  11 May 2022
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

#include <ros/ros.h>                         // ROS Stuff
#include <bits/stdc++.h>                     // C++ stuff
#include <ufo/map/occupancy_map.h>           // Non-colored map
#include <ufo/map/occupancy_map_color.h>     // Colored map
#include <random_numbers/random_numbers.h>   // Random number
#include <ufomap_msgs/UFOMapStamped.h>       // UFOMap ROS msg
#include <ufomap_msgs/conversions.h>         // To convert between UFO and ROS
#include <ufomap_ros/conversions.h>          // To convert between UFO and ROS
#include <pcl/point_cloud.h>                 // PCL Stuff
#include <pcl/io/ply_io.h>                   // PCL file i/o
#include <pcl/point_types.h>                 // PCL Types
#include <pcl_conversions/pcl_conversions.h> // PCL ROS

using namespace std; // Standard namespace

double resolution = 0.25;
ufo::map::OccupancyMapColor map(0.25); // Resolution, levels, prune?, th_occ, th_free, p_hit, p_miss, clamp_th_min, clm_th_max

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
    std::cout << prefix << pt.x() << "," << pt.y() << "," << pt.z() << std::endl;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "example_ufomap_save"); // Initialize the map
    ros::NodeHandle nh;                           // Create the node handle

    std::cout.precision(5);                                  // Set the floating point percision for cout
    std::cout.setf(std::cout.showpos | std::cout.showpoint); // Show the leading +- and zero

    ufomap_msgs::UFOMapStamped::ConstPtr msg = ros::topic::waitForMessage<ufomap_msgs::UFOMapStamped>("ufomap", ros::Duration(10)); // Wait 10Sec for the msg.
    ufo::map::OccupancyMap map(1, 21);                                                                                              // Resolution, levels, prune?, th_occ, th_free, p_hit, p_miss, clamp_th_min, clm_th_max

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.height = 1;      // 1 = unordered, otherwise image
    cloud.width = 0;       // Cloud width, automatically updated during push
    cloud.is_dense = true; // No points are invalid

    if (ufomap_msgs::msgToUfo(msg->map, map))
    {
        std::cout << "[[[UFOMAP]]]" << std::endl;
        std::cout << "Resolution: " << map.getResolution() << std::endl;
        std::cout << "Levels: " << map.getTreeDepthLevels() << std::endl;
        // beginTree: nodes from specified levels to maxlevels
        // beginLeaves: only specified type of voxel at specified level
        for (auto it = map.beginLeaves(true, false, false, true, 0); it != map.endLeaves(); ++it)
        {
            // it: containsUnknown(), containsFree(), containsOccupied(), getX(), getY(), getZ()
            // it: getDepth(), getSize(), hasChildren(), isPureLeaf(), getOccupancy()
            // printPoint3(it.getCenter(), "Pt: ");
            cloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
        }

        if (cmdOptionExists(argv, argv + argc, "-pcd"))
        {
            std::cout << "Saving PCD ... " << cloud.width << std::endl;
            pcl::io::savePCDFile("/home/ahmad/catkin_ws/src/ROSExamples/map/ufomap.pcd", cloud, false); // file_name, pointcloud, save_binary
        }
        else if (cmdOptionExists(argv, argv + argc, "-ply"))
        {
            std::cout << "Saving PLY ... " << cloud.width << std::endl;
            pcl::io::savePLYFile("/home/ahmad/catkin_ws/src/ROSExamples/map/ufomap.ply", cloud, false); // file_name, pointcloud, save_binary
        }
        else if (cmdOptionExists(argv, argv + argc, "-ufo"))
        {
            std::cout << "Saving UFO ... " << cloud.width << std::endl;
            map.write("/home/ahmad/catkin_ws/src/ROSExamples/map/ufomap.ufo", false, 0); // file_name, compress, depth level
        }
        else
        {
            std::cout << "Syntex: rosrun gap_detection save_ufomap -pcd | -ply | -ufo ufomap:=/ufomap_mapping_server_node/map" << map.getResolution() << std::endl;
        }
    }
    else
    {
        std::cout << "Failure" << std::endl;
    }

    return 0;
}