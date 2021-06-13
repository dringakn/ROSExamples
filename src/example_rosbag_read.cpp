/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Reading ROSBAG using C++ API
        Use the rosbag API to create a rosbag::View, filter the bag with a rosbag::Query, 
        and iterate over the items in a foreach loop.
        First, we’ll set up the rosbag::Bag object to read from the file:
        We will be creating a rosbag::View of this bag; a filtered-down set of messages. 
        To filter them, we’ll be using queries. As I know the topic names I’m interested in, 
        I’ll be using rosbag::TopicQuery. Its constructor accepts a std::string or std::vector<std::string> of topic names.

        We may iterate over the entries in the view by using a foreach loop. 
        The iterated item m will be of type rosbag::MessageInstance, a type-agnostic object. 
        It needs some inspection to figure out which message you are processing. 
        For instance, you can request the currently contained data type by m.getDataType(), 
        and the topic the message came from with m.getTopic(). 
        You can also just use m.instantiate<T>() with T the desired message type. 
        The returned object is a pointer to that message; if that pointer isn’t NULL, 
        everything went fine and you can process it. If it is NULL, it was a message of a different type.

      Example:
        rosbag info input.bag
        roscore
        rosrun example_rosbag_read input.bag
*/

#include <ros/ros.h>                    // ROS stuff
#include <rosbag/bag.h>                 // ROSBAG stuff
#include <rosbag/view.h>                // ROSBAG view
#include <sensor_msgs/Imu.h>            // IMU message
#include <sensor_msgs/NavSatFix.h>      // GPS message
#include <velodyne_msgs/VelodyneScan.h> // Velodyne raw scan message

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_rosbag_read");
    ros::NodeHandle nh;
    if (argc < 2)
    {
        ROS_INFO("example_rosbag_read inpug.bag");
        exit(1);
    }
    std::string bag_filename = argv[1];

    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    std::string imu_topic = "/dji_osdk_ros/imu";
    std::string gps_topic = "/dji_osdk_ros/gps_position";
    std::string velodyne_topic = "/velodyne_packets";
    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(gps_topic);
    topics.push_back(velodyne_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (const rosbag::MessageInstance m : view)
    {
        if (m.getTopic() == imu_topic)
        {
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if (imu != NULL)
            {
                // process IMU
                ROS_INFO("IMU");
            }
        }
        else if (m.getTopic() == gps_topic)
        {
            sensor_msgs::NavSatFix::ConstPtr gps = m.instantiate<sensor_msgs::NavSatFix>();
            if (gps != NULL)
            {
                // process gps
                ROS_INFO("GPS");
            }
        }
        else if (m.getTopic() == velodyne_topic)
        {
            velodyne_msgs::VelodyneScan::ConstPtr lidar = m.instantiate<velodyne_msgs::VelodyneScan>();
            if (lidar != NULL)
            {
                // process lidar
                ROS_INFO("LIDAR");
            }
        }
        else
        {
            // Other message
        }
    }

    return 0;
}