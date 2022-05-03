/**
 * Author: Dr. Ing. Ahmad Kamal Nasir
 * Email: dringakn@gmail.com
 * Modified:  02 May 2022
 * Description:
 *      UFOMap is an extension to the octomapping.
 *      It provides the explicit handling of unknown, free, and occupied space in a map.
 *      The following example loads an existing point cloud from a pcd file.
 *      Modifications: Load pointcloud map file (pcd) [Add pcl_ros to the CMakeLists.txt]
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

#include <ros/ros.h>                     // ROS stuff
#include <ufo/map/occupancy_map.h>       // ufo map
#include <ufo/map/occupancy_map_color.h> // ufo color map
#include <ufomap_msgs/UFOMapStamped.h>   // UFOMap ROS msg
#include <ufomap_msgs/conversions.h>     // To convert between UFO and ROS
#include <ufomap_ros/conversions.h>      // To convert between UFO and ROS
#include <bits/stdc++.h>                 // C++ stuff
#include <pcl/io/pcd_io.h>               // PCL stuff

typedef pcl::PointXYZI Point3D;
typedef pcl::PointCloud<Point3D> PointCloud;

using namespace std;

int main(int argc, char *argv[])
{
    std::cout.precision(5);
    std::cout.setf(std::cout.showpos | std::cout.showpoint);
    ros::init(argc, argv, "example_ufomap_load_pcd_file");
    ros::NodeHandle nh;

    /*
        Load pointcloud file (pcd) into the UFOMap
    */
    ros::Publisher pub_ufo = nh.advertise<ufomap_msgs::UFOMapStamped>("/ufomap", 10, true);

    std::string map_file_name = "/home/ahmad/catkin_ws/src/gap_detection/map/height_avgd_pc_rammelsberg_raw_version.pcd";
    PointCloud *pc = new PointCloud();
    if (pcl::io::loadPCDFile(map_file_name.c_str(), *pc) != -1)
    {
        ROS_INFO("Loading (%s) %d points ...", map_file_name.c_str(), pc->width);
        ufo::map::OccupancyMap map(0.5, 16); // OccupancyMapColor: Create a colored UFOMap
        for (auto &&pt : pc->points)
            map.updateOccupancy(map.toCode(pt.x, pt.y, pt.z, 0), map.getClampingThresMax());

        ROS_INFO("Pointcloud file (%s) loaded %d x %d", map_file_name.c_str(), pc->height, pc->width);

        ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped); // This is the UFOMap message object.
        if (ufomap_msgs::ufoToMsg(map, msg->map, false, 0))                  // Convert UFOMap to ROS message: map, data, compress, depth
        {
            // Conversion was successful
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = "map";
            pub_ufo.publish(msg);
        }

        // Keep the node alive
        ros::Rate rate(1);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        ROS_INFO("Map file (%s) doesn't exist!", map_file_name.c_str());
    }

    return 0;
}