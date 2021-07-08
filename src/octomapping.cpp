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

/**
 * Conversion from octomap::point3d_list (e.g. all occupied nodes from getOccupied()) to sensor_msgs::PointCloud2.
 * pointsOctomapToPointCloud2(const point3d_list& points, sensor_msgs::PointCloud2& cloud)
 * 
 * Conversion from a sensor_msgs::PointCLoud2 to octomap::Pointcloud, used internally in OctoMap
 * pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud, Pointcloud& octomapCloud)
 * 
 * 
 **/

ros::Publisher pubPose, pubPos, pubOctomap;
tf2_ros::TransformBroadcaster *br = NULL;
octomap::OcTree *ot;

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
        transform.transform.translation.x = uavPose.pose.position.x;
        transform.transform.translation.y = uavPose.pose.position.y;
        transform.transform.translation.z = uavPose.pose.position.z;
        transform.transform.rotation = msgIMU->orientation;
        br->sendTransform(transform);

        // Update octree using scan
        // create a octree pointcloud
        // octomap::Pointcloud cloud;
        // octomap::pointCloud2ToOctomap(*msgLiDAR, cloud);
        // octomath::Vector3 sensor_pos(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        // octomath::Quaternion sensor_orient(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
        // octomap::pose6d sensorPose(sensor_pos, sensor_orient);
        // cloud.transformAbsolute(sensorPose);
        // cloud.transform(sensorPose);

        // octomap::point3d lower, upper;
        // cloud.calcBBX(lower, upper);
        // ROS_INFO_STREAM("lower:" << lower << ", uppper:" << upper << ", size:" << cloud.size());

        // Register the pointcloud
        // ot->clear();
        // cloud.clear();
        // const double ds = 10;
        // for (double x = -2*ds; x < 2*ds; x += 0.25)
        //     for (double y = -ds; y < ds; y += 0.25)
        //             cloud.push_back(x, y, -10);
        //             // ot->updateNode(x, y, z, (rand() / (double)RAND_MAX) > 0.5);
        // cloud.transform(sensorPose);
        // ot->insertPointCloud(cloud, sensor_pos, 50, true, false);
        // ot->insertPointCloudRays(cloud, sensor_pos, -1, false);
        // ot->prune();

        // ROS_INFO("Leafs: %d, %d, %d\t", (int)ot->getNumLeafNodes(), ot->isChangeDetectionEnabled(), ot->numChangesDetected());
        // ROS_INFO_STREAM("pose: " << transform.transform.rotation);
        // ot->resetChangeDetection();
        
        // Determine and publish unknown cells
        // Publishe octomap
        // octomap_msgs::Octomap msgOctomap;
        // msgOctomap.header = transform.header;
        // octomap_msgs::binaryMapToMsg(*ot, msgOctomap);
        // pubOctomap.publish(msgOctomap);

        uavPose.header.frame_id = "base_link";
        uavPose.header.stamp = transform.header.stamp;
        uavPose.pose.position.x = uavPose.pose.position.y = uavPose.pose.position.z = uavPose.pose.orientation.x = uavPose.pose.orientation.y = uavPose.pose.orientation.z = 0;
        uavPose.pose.orientation.w = 1;
        pubPose.publish(uavPose);

        uavPos.header = uavPose.header;
        pubPos.publish(uavPos);
    }
}

// void octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg)
// {
//     octomap::OcTree *ot = dynamic_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(*msg));
//     // double sample_res = 4 * msg->resolution;
//     // int depth = floor(16 - (log(sample_res / ot->getResolution()) / log(2)));
//     // sample_res = ot->getResolution() * pow(2, 16 - depth);
//     // ROS_INFO("Res: %3.2f, SampleRes: %3.2f, Size: %d", ot->getResolution(), sample_res, (int)ot->size());
//     // ROS_INFO_STREAM("OctomapRes:" << msg->resolution << ":" << ot << ":" << msg->id << ":" << msg->header.frame_id);
//     double minx, miny, minz, maxx, maxy, maxz;
//     ot->getMetricMin(minx, miny, minz);
//     ot->getMetricMax(maxx, maxy, maxz);
//     ROS_INFO("Min:(%3.1f,%3.1f,%3.1f) Max(%3.1f,%3.1f,%3.1f): %d", minx, miny, minz, maxx, maxy, maxz, ot->calcNumNodes());
//     // double ds = 0.5;
//     // // if (((maxx - minx) > 2 * ds) && ((maxy - miny) > 2 * ds) && ((maxz - minz) > 2 * ds))
//     // {
//     //     minx *= ds, miny *= ds, minz *= ds;
//     //     maxx *= ds, maxy *= ds, maxz *= ds;
//     //     octomap::point3d_list list;
//     //     octomap::point3d lower(minx, miny, minz), upper(maxx, maxy, maxz);
//     //     ot->getUnknownLeafCenters(list, lower, upper, depth);
//     //     // ROS_INFO("Unkonwn: %d ", (int)list.size());
//     //     octomap::ColorOcTree octree(sample_res);
//     //     double x, y, z;
//     //     for (auto &&pt : list)
//     //     {
//     //         // x = pt.x(), y = pt.y(), z = pt.z();
//     //         // if ((((x > minx) && (x < maxx) && (y > miny) && (y < maxy) && (z > minz) && (z < maxz))))
//     //             octree.updateNode(pt, true, false);
//     //     }
//     //     octree.updateInnerOccupancy();
//     //     octomap_msgs::Octomap msgOctomap;
//     //     msgOctomap.header = msg->header;
//     //     octomap_msgs::binaryMapToMsg(octree, msgOctomap);
//     //     pubOctomap.publish(msgOctomap);
//     // }
// }

/*
TODO:
    Create a 3d cubiod of size 1x1x0.5 (res = 0.1).
    At first all voxels are empty. Then make the eight corner voxels occupied.
    Same could be tried with the octomap::Pointcloud.
    Finally, create 8 unknown voxels in the middle of the cubiod toward each corner. 
    - Study the transform method.
    - Study the octreecoord methods.
    -  
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_odom");
    ros::NodeHandle nh("~");

    br = new tf2_ros::TransformBroadcaster();
    ot = new octomap::OcTree(0.25);
    ot->enableChangeDetection(true);

    pubPose = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    pubPos = nh.advertise<geometry_msgs::PointStamped>("pos", 100);
    pubOctomap = nh.advertise<octomap_msgs::Octomap>("octomap_out", 1, true);
    // ros::Subscriber subOctomap = nh.subscribe<octomap_msgs::Octomap>("/octomap_in", 1, octomap_callback);

    message_filters::Subscriber<sensor_msgs::NavSatFix> subGPS(nh, "/gps_in", 100);
    message_filters::Subscriber<sensor_msgs::Imu> subIMU(nh, "/imu_in", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLiDAR(nh, "/lidar_in", 100);

    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::NavSatFix, sensor_msgs::PointCloud2>;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subIMU, subGPS, subLiDAR);
    sync.registerCallback(callback);

    ros::spin();
    return 0;
}
