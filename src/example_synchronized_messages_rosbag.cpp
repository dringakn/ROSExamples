/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Recording synchronized messages.
      Subscribe to two different type of multi-rate messages and record them into a rosbag
      after synchronizing them. The ROS message_filters provides the two type of time synchronization,
      approximate and exact.

      Example:
        roscore
        rosrun ros_examples example_synchronized_messages_rosbag
        rostopic pub -r 1 /point geometry_msgs/PointStampled ... 
        rostopic pub -r 0.2 /vector geometry_msgs/Vector3Stampled ... 

    Note: 
      Only works with messages containing header (time information).
*/

#include <ros/ros.h>                                        // ROS
#include <message_filters/subscriber.h>                     // Message filter subscriber
#include <message_filters/synchronizer.h>                   // Message filter synchronizer
#include <message_filters/sync_policies/approximate_time.h> // Message filer policy
#include <geometry_msgs/PointStamped.h>                     // Point message
#include <geometry_msgs/Vector3Stamped.h>                   // Vector message
#include <rosbag/bag.h>                                     // Rosbag

rosbag::Bag outputBag;

void Callback(const geometry_msgs::PointStamped::ConstPtr &msgP,
              const geometry_msgs::Vector3Stamped::ConstPtr &msgV)
{
    ROS_INFO("%f", ros::Time::now().toSec());
    if (outputBag.isOpen())
    {
        outputBag.write("synced/point", msgP->header.stamp.now(), *msgP);
        outputBag.write("synced/vector", msgV->header.stamp.now(), *msgV);
    }
}

std::string int2string(int value)
{
    std::stringstream ss;
    ss << value;
    return ss.str();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_synchronized_messages_rosbag");
    ros::NodeHandle nh;
    message_filters::Subscriber<geometry_msgs::PointStamped> subPt(nh, "/point", 1);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> subVt(nh, "/vector", 1);
    using syncPolicyA = message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::Vector3Stamped>;
    message_filters::Synchronizer<syncPolicyA> sync(syncPolicyA(10), subPt, subVt);
    sync.registerCallback(Callback);

    //the name of bag file is better to be determined by the system time
    time_t t = std::time(0);
    struct tm *now = std::localtime(&t);
    std::string file_name;
    file_name = "/home/ahmad/" + int2string(now->tm_year + 1900) +
                '-' + int2string(now->tm_mon + 1) +
                '-' + int2string(now->tm_mday) +
                '-' + int2string(now->tm_hour) +
                '-' + int2string(now->tm_min) +
                '-' + int2string(now->tm_sec) +
                ".bag";

    ROS_INFO("Creating bag file.");
    outputBag.open(file_name, rosbag::bagmode::Write);

    ros::spin();

    ROS_INFO("Closing bag file.");
    outputBag.close();
    return 0;
}