#include <ros/ros.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl-1.7/pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

// Add pcl_ros library in CMakeLists

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example13");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 1000, true);
    pcl::PCLPointCloud2* pc = new pcl::PCLPointCloud2;
    if (pcl::io::loadPLYFile("/home/workshop/LUMS1.ply", *pc) == -1)
    {
        ROS_INFO("Pointcloud file not loaded");
        return -1;
    }
    
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}