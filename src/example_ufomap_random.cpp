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

#include <ros/ros.h>                       // ROS Stuff
#include <bits/stdc++.h>                   // C++ stuff
#include <ufo/map/occupancy_map.h>         // Non-colored map
#include <ufo/map/occupancy_map_color.h>   // Colored map
#include <random_numbers/random_numbers.h> // Random number
#include <ufomap_msgs/UFOMapStamped.h>     // UFOMap ROS msg
#include <ufomap_msgs/conversions.h>       // To convert between UFO and ROS
#include <ufomap_ros/conversions.h>        // To convert between UFO and ROS

using namespace std; // Standard namespace

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "example_random_ufomap"); // Initialize the map
    ros::NodeHandle nh;                             // Create the node handle

    std::cout.precision(5);                                  // Set the floating point percision for cout
    std::cout.setf(std::cout.showpos | std::cout.showpoint); // Show the leading +- and zero

    /*
        Create a non colored UFO map with minimum resolution of 0.25m and with 16 levels (Max: 21).
        Each level has double the size of voxel. Thus the extreme size of the enviornment can be
        from (-8192m, -8192m, -8192m) to (8192m, 8192m, 8192m).
    */
    random_numbers::RandomNumberGenerator rng; // Random number generator
    // rosrun ros_examples example_ufomap_random res maxx maxy maxz n_points
    double res = (argc > 1) ? atof(argv[1]) : 1;
    double maxx = (argc > 2) ? atoi(argv[2]) : 10;           // Maximum/Minimum value of the random number along-x
    double maxy = (argc > 3) ? atoi(argv[3]) : 10;           // Maximum/Minimum value of the random number along-y
    double maxz = (argc > 4) ? atoi(argv[4]) : 10;           // Maximum/Minimum value of the random number along-z
    unsigned int n_points = (argc > 5) ? atoi(argv[5]) : 10; // Number of random points
    unsigned int prob = (argc > 6) ? atoi(argv[6]) : 1;      // Occupancy 1=Occupied, 0=Free
    unsigned int inv_prob = (prob == 0) ? 1 : 0;
    unsigned int nlevels = 16;
    ufo::map::OccupancyMap map(res, nlevels);                                               // Resolution, levels, prune?, th_occ, th_free, p_hit, p_miss, clamp_th_min, clm_th_max
    ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("/ufomap", 10, true); // UFO map published topic (latched)

    /*
        Fill/Clear the space.
    */
    size_t ctr = 0, total = 8 * maxx * maxy * maxz;
    for (double x = -maxx; x < maxx; x += res)
        for (double y = -maxy; y < maxy; y += res)
            for (double z = -maxz; z < maxz; z += res)
            {
                map.updateOccupancy(x, y, z, inv_prob, 0); // xyz, prob=1, depth=0
                ctr++;
                ROS_INFO_THROTTLE(5, "Filled percentage: %d", (100 * ctr / total));
            }

    /*
        Set the occupancy value at specified level (convert the prob to log odds): log(p/(1-p))
    */
    double x, y, z;
    for (unsigned int i = 0; i < n_points; i++)
    {
        x = rng.uniformReal(-maxx, maxx);
        y = rng.uniformReal(-maxy, maxy);
        z = rng.uniformReal(-maxz, maxz);
        map.updateOccupancy(x, y, z, prob, 0); // xyz, prob=1, depth=0
        // cout << "Point " << i << ": " << x << "," << y << "," << z << endl;
    }

    /*
        If the UFOMap should be compressed using LZ4. Good if you are sending the UFOMap between computers.
        Lowest depth to publish.Higher value means less data to transfer, good in situation where the data rate is low.
    */
    cout << "Publishing UFOmap [Res, MaxX, MaxY, MaxZ, NPoints, Prob] " << res << "," << maxx << "," << maxy << "," << maxz << "," << n_points << "," << prob << endl;
    bool compress = false;
    ufo::map::DepthType pub_depth = 0;

    ros::Rate rate(0.00001);
    while (ros::ok())
    {
        // This is the UFOMap message object.
        ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);

        // Convert UFOMap to ROS message
        if (ufomap_msgs::ufoToMsg(map, msg->map, compress, pub_depth))
        {
            // Conversion was successful
            msg->header.stamp = ros::Time::now(); // Current computer time
            msg->header.frame_id = "map";         // frame name for the map
            map_pub.publish(msg);                 // publish the message
        }

        rate.sleep(); // Delay for re-publishing
    }
    return 0;
}