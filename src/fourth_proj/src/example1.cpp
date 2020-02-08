// Basic ROS C++ Node structure.
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "example");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("Hellow World: %d", count++);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}