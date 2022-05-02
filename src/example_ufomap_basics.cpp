/**
 * Author: Dr. Ing. Ahmad Kamal Nasir
 * Email: dringakn@gmail.com
 * Modified:  28 June 2021
 * Description: UFOMap is an extension to the octomapping.
 *      It provides the explicit handling of unknown, free, and occupied space in a map.
 * Note:
 *      IMPORTANT:  
 *      To use UFOMap in your package you need to add at the begining:
 *          - Clone the UFOMapping to ~/*_ws/src folder (git clone https://github.com/UnknownFreeOccupied/ufomap.git)
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

// We will create colored UFOMap in this tutorial
#include <ufo/map/occupancy_map_color.h>
// To convert between UFO and ROS
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

ufo::map::OccupancyMapColor *map;               // Create a colored UFOMap
tf2_ros::Buffer *tf_buffer = NULL;               // Transform listen buffer
tf2_ros::TransformListener *tf_listener = NULL; // Transform listener object

void cloudCallback(sensor_msgs::PointCloud2::ConstPtr const &msg)
{
    // Get transform
    ufo::math::Pose6 transform;
    try
    {
        // Lookup transform
        geometry_msgs::TransformStamped tf_trans = tf_buffer->lookupTransform("map",
                                                                              msg->header.frame_id,
                                                                              msg->header.stamp,
                                                                              ros::Duration(0.1));
        // Convert ROS transform to UFO transform
        transform = ufomap_ros::rosToUfo(tf_trans.transform);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        return;
    }

    ufo::map::PointCloudColor cloud;
    // Convert ROS point cloud to UFO point cloud
    ufomap_ros::rosToUfo(*msg, cloud);
    // Transform point cloud to correct frame, do it in parallel (second param true)
    cloud.transform(transform, true);

    // Integrate point cloud into UFOMap, no max range (third param -1),
    // free space at depth level 1 (fourth param 1)
    map->insertPointCloudDiscrete(transform.translation(), cloud, -1, 1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_ufomap_basics");

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("/cloud_in", 10, cloudCallback);

    tf_buffer = new tf2_ros::Buffer(ros::Duration(10), false);

    tf_listener = new tf2_ros::TransformListener(*tf_buffer, nh, true);

    map = new ufo::map::OccupancyMapColor(0.1);

    ros::spin();

    return 0;
}