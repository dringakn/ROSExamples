#include <ros/ros.h>
#include "../include/gps_conversions.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <octomap/ColorOcTree.h>

#include <octomap/Pointcloud.h>
#include <octomap_ros/conversions.h> // Octomap <-> PCL
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h> // octomap <-> ROS
#include "../include/OctomapFrontiers.h"

#include <pcl/filters/voxel_grid.h> // Voxel grid filter
typedef pcl::PointXYZRGB Point3DColor;
ros::Publisher pubFiltered;
pcl::PointCloud<Point3DColor>::Ptr inputCloud(new pcl::PointCloud<Point3DColor>);       // Input point cloud
pcl::PointCloud<Point3DColor>::Ptr downSampledCloud(new pcl::PointCloud<Point3DColor>); // Voxel grid filtered

ros::Publisher pubPose, pubPos;
tf2_ros::TransformBroadcaster *br = NULL;
octomap_frontiers::OctomapFrontiers *om;
// octomap::OcTree *ot;

void voxelGridFilter(pcl::PointCloud<Point3DColor>::Ptr input, pcl::PointCloud<Point3DColor>::Ptr output, double voxel_size)
{
    ROS_INFO("Voxel grid filtering ...");
    output->clear();
    pcl::VoxelGrid<Point3DColor> filter;
    filter.setInputCloud(input);
    filter.setMinimumPointsNumberPerVoxel(1);
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter.filter(*output);
    ROS_INFO("Input: %d, Output: %d", (int)input->points.size(), (int)output->points.size());
}

void publishPointCloud(pcl::PointCloud<Point3DColor>::Ptr input, ros::Publisher &pub)
{
    pcl::PCLPointCloud2::Ptr pc(new pcl::PCLPointCloud2);       // ROS Pointcloud msg
    pcl::toPCLPointCloud2(*input, *pc);                         // Convert input PC to ROS PC msg
    pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp); // Convert PCL compatible time
    pc->header.frame_id = "map";                                // Set the frame
    pub.publish(*pc);                                           // Publish the message
}

bool geodatic2Cartesian(geometry_msgs::Point &ll, geometry_msgs::Point &xyz)
{
    bool result = false;
    std::string zone;
    static geometry_msgs::Point initPos;
    static bool init = false;
    gps_common::LLtoUTM(ll.x, ll.y, xyz.y, xyz.x, zone);
    if (true)
    {
        xyz.z = ll.z;
        if (!init)
        {
            initPos = xyz;
            init = true;
        }
        xyz.x -= initPos.x;
        xyz.y -= initPos.y;
        xyz.z -= initPos.z;
        result = true;
    }
    return result;
}

void callback(const sensor_msgs::Imu::ConstPtr &msgIMU, const sensor_msgs::NavSatFix::ConstPtr &msgGPS, const sensor_msgs::PointCloud2::ConstPtr &msgLiDAR)
{
    geometry_msgs::Point ll;
    ll.x = msgGPS->latitude;
    ll.y = msgGPS->longitude;
    ll.z = msgGPS->altitude;
    geometry_msgs::PoseStamped uavPose;
    geometry_msgs::PointStamped uavPos;
    geometry_msgs::TransformStamped transform;
    if (geodatic2Cartesian(ll, uavPose.pose.position))
    {
        transform.child_frame_id = "base_link";
        transform.header.frame_id = "odom";
        transform.header.stamp = msgLiDAR->header.stamp;
        // transform.header.stamp = ros::Time(0);
        transform.transform.translation.x = uavPose.pose.position.x;
        transform.transform.translation.y = uavPose.pose.position.y;
        transform.transform.translation.z = uavPose.pose.position.z;
        transform.transform.rotation = msgIMU->orientation;
        br->sendTransform(transform);

        // Publish static transforms
        transform.child_frame_id = "odom";
        transform.header.frame_id = "map";
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        transform.transform.rotation.x = transform.transform.rotation.y = transform.transform.rotation.z = 0;
        transform.transform.rotation.w = 1;
        br->sendTransform(transform);


        uavPose.header.frame_id = "base_link";
        uavPose.header.stamp = transform.header.stamp;
        uavPose.pose.position.x = uavPose.pose.position.y = uavPose.pose.position.z = uavPose.pose.orientation.x = uavPose.pose.orientation.y = uavPose.pose.orientation.z = 0;
        uavPose.pose.orientation.w = 1;
        pubPose.publish(uavPose);
        uavPos.header = uavPose.header;
        pubPos.publish(uavPos);

        // pcl_conversions::toPCL(msgLiDAR, inputCloud);
        // pcl_conversions::toPCL(*msgLiDAR, *inputCloud);
        // inputCloud->
        // voxelGridFilter(inputCloud, downSampledCloud, 0.5);
        // publishPointCloud(downSampledCloud, pubFiltered);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomapping_node");
    ros::NodeHandle nh("~");

    br = new tf2_ros::TransformBroadcaster();

    pubPose = nh.advertise<geometry_msgs::PoseStamped>("/uav_pose", 100);
    pubPos = nh.advertise<geometry_msgs::PointStamped>("/uav_position", 100);

    message_filters::Subscriber<sensor_msgs::NavSatFix> subGPS(nh, "/gps_in", 100);
    message_filters::Subscriber<sensor_msgs::Imu> subIMU(nh, "/imu_in", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLiDAR(nh, "/lidar_in", 100);

    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::NavSatFix, sensor_msgs::PointCloud2>;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subIMU, subGPS, subLiDAR);
    sync.registerCallback(callback);

    // om = new octomap_frontiers::OctomapFrontiers();

    pubFiltered = nh.advertise<pcl::PCLPointCloud2>("/pc_filtered", 1, true);

    ros::spin();
    return 0;
}
