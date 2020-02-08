/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example9");
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}