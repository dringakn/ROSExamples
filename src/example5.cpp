/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
// ROS parameter server example
#include <ros/ros.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example5");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    double param1 = 0;
    while (ros::ok())
    {
        if(nh.getParam("param1", param1)){
            ROS_INFO("param1: %f", param1);
            nh.setParam("param1", ++param1);
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}