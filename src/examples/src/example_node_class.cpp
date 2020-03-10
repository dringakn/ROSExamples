#include <ros/ros.h>
#include "examples/MyNodeClass.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MyNodeClass");
    ros::NodeHandle nh;
    MyNodeClass node(&nh);
    ros::spin();
    return 0;
}