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

#include <ros/ros.h>
#include <ufo/map/occupancy_map.h>
#include <bits/stdc++.h>

/*
    Modifications: Load pointcloud map file (ply)
    Notes: Add pcl_ros to the CMakeLists.txt
*/
#include <pcl/io/ply_io.h>
typedef pcl::PointXYZI Point3D;
typedef pcl::PointCloud<Point3D> PointCloud;

using namespace std;

void printPoint3(ufo::map::Point3 pt, std::string prefix)
{
    cout << prefix << pt.x() << "," << pt.y() << "," << pt.z() << endl;
}

void printPoint(ufo::geometry::Point pt, std::string prefix)
{
    cout << prefix << pt.x() << "," << pt.y() << "," << pt.z() << endl;
}

void ufomap_stats(ufo::map::OccupancyMap &map)
{
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

void ufomap_node_stats()
{
    std::cout << " <<<<<<<<<<<<<<<<< UFOMap Stats >>>>>>>>>>>>>>>>> " << std::endl;

    std::cout << " ================================================= " << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout.precision(5);
    std::cout.setf(std::cout.showpos | std::cout.showpoint);

    ufo::map::OccupancyMap map(0.25, 16); // Resolution, levels, prune?, th_occ, th_free, p_hit, p_miss, clamp_th_min, clm_th_max
    map.updateOccupancy(1, 1, 1, 1);
    map.updateOccupancy(1, 1, -1, 1);
    map.updateOccupancy(100, 100, 100, 1);
    map.updateOccupancy(100, 100, -100, 1);
    ufomap_stats(map);
    int i = 0;
    // for (auto it = map.beginNNLeaves(ufo::map::Point3(1, 1, 1), false, false, true, false, 0); it != map.endNNLeaves(); ++it)
    // {
    //     cout << ++i << " - ";
    //     printPoint3(it.getCenter(), "Center:");
    // }

    unsigned int entryopy;
    for (auto it = map.beginTree(true, true, true, false, 0); it != map.endTree(); ++it) // voxels at all levels
    // for (auto it = map.beginLeaves(true, true, true, false, 0); it != map.endLeaves(); ++it)
    {
        
        entryopy = it.containsUnknown() + it.containsFree() + it.containsOccupied();
        if (it.getSize() >= 4 && it.getSize() <= 5 && it.hasChildren() && entryopy > 1)
        {
            std::cout << ++i << "-(" << it.getX() << "," << it.getY() << "," << it.getZ() << ")\t";
            std::cout << "\t Depth:" << it.getDepth();
            std::cout << "\t Size:" << it.getSize();
            std::cout << "\t Children?:" << it.hasChildren();
            std::cout << "\t PureLeaf?:" << it.isPureLeaf();
            std::cout << "\t Unknown?:" << it.containsUnknown();
            std::cout << "\t Free?:" << it.containsFree();
            std::cout << "\t Occupked?:" << it.containsOccupied();
            std::cout << "\t Value:" << it.getOccupancy();
            std::cout << std::endl;
        }
    }

    /*
        Load pointcloud file (ply) into the UFOMap
    */
    std::string map_file_name = "/home/ahmad/catkin_ws/src/gap_detection/map/map.ply";
    PointCloud *pc = new PointCloud();
    if (pcl::io::loadPLYFile(map_file_name.c_str(), *pc) != -1)
    {
        for (auto &&pt : pc->points)
            map.updateOccupancy(map.toCode(pt.x, pt.y, pt.z, 0), map.getClampingThresMax());
        ROS_INFO("Pointcloud file (%s) loaded %d x %d", map_file_name.c_str(), pc->height, pc->width);
    }
    else
    {
        ROS_INFO("Map file (%s) doesn't exist!", map_file_name.c_str());
    }

    return 0;
}