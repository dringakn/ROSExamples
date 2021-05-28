/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description: Search Nearest Neighbours in an Octree
 *    Notes: Add pcl_ros library in CMakeLists
 **/

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>            // File
#include <pcl/octree/octree_search.h> // Octree

typedef pcl::PointXYZRGB Point3D;
typedef pcl::PointXYZI Point3DI;

inline float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void publishPointCloud(pcl::PointCloud<Point3D>::Ptr input, ros::Publisher &pub)
{
    pcl::PCLPointCloud2::Ptr pc(new pcl::PCLPointCloud2);       // ROS Pointcloud msg
    pcl::toPCLPointCloud2(*input, *pc);                         // Convert input PC to ROS PC msg
    pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp); // Convert PCL compatible time
    pc->header.frame_id = "map";                                // Set the frame
    pub.publish(*pc);                                           // Publish the message
}

void publishPointCloud(pcl::PointCloud<Point3DI>::Ptr input, ros::Publisher &pub)
{
    pcl::PCLPointCloud2::Ptr pc(new pcl::PCLPointCloud2);       // ROS Pointcloud msg
    pcl::toPCLPointCloud2(*input, *pc);                         // Convert input PC to ROS PC msg
    pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp); // Convert PCL compatible time
    pc->header.frame_id = "map";                                // Set the frame
    pub.publish(*pc);                                           // Publish the message
}

void savePointCloud(pcl::PointCloud<Point3D>::Ptr input, std::string output_file_name)
{
    pcl::PLYWriter writer;                                                // I/O object
    ROS_INFO("Saving pointcloud to %s", output_file_name.c_str());        // Info
    writer.write<Point3D>(output_file_name.c_str(), *input, true, false); // Output
}

void savePointCloud(pcl::PointCloud<Point3DI>::Ptr input, std::string output_file_name)
{
    pcl::PLYWriter writer;                                                // I/O object
    ROS_INFO("Saving pointcloud to %s", output_file_name.c_str());        // Info
    writer.write<Point3DI>(output_file_name.c_str(), *input, true, false); // Output
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gap_detection_node");
    ros::NodeHandle nh("~");

    // Read parameters from the server
    std::string map_file_name;
    if (!nh.param<std::string>("map_file_name", map_file_name, "map.ply"))
        nh.setParam("map_file_name", map_file_name);
    ROS_INFO("Map file: %s", map_file_name.c_str());

    float octree_resolution;
    if (!nh.param<float>("octree_resolution", octree_resolution, 1))
        nh.setParam("octree_resolution", octree_resolution);
    ROS_INFO("Octree resolution: %3.2f", octree_resolution);

    float voxel_size;
    if (!nh.param<float>("voxel_size", voxel_size, 1))
        nh.setParam("voxel_size", voxel_size);
    ROS_INFO("Voxel size: %3.2f", voxel_size);
    voxel_size *= 0.5;

    // System variables
    ros::Publisher pub = nh.advertise<pcl::PCLPointCloud2>("/pointcloud", 1000, true);
    pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>); // Input point cloud

    // Read and publish pointcloud
    pcl::PLYReader reader;
    int result = reader.read(map_file_name.c_str(), *cloud, 0);
    if (result == -1)
    {
        ROS_INFO("Pointcloud file (%s) could not be loaded.", map_file_name.c_str());
        return -1;
    }
    else
    {
        ROS_INFO("Loaded %dx%d points from %s", cloud->width, cloud->height, map_file_name.c_str());
        publishPointCloud(cloud, pub);
    }

    // Create an octree
    pcl::octree::OctreePointCloudSearch<Point3D> octree(octree_resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // Get octree bounding box information
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    ROS_INFO("Bounding Box: (%4.1f,%4.1f,%4.1f) (%4.1f,%4.1f,%4.1f)", min_x, min_y, min_z, max_x, max_y, max_z);

    // Neighbors within radius search
    // Calculate neighbour of all points
    ROS_INFO("Calculating density...");
    pcl::PointCloud<Point3DI>::Ptr densityCloud(new pcl::PointCloud<Point3DI>); // Input point cloud
    pcl::Indices k_indices;
    Eigen::Vector3f min_pt, max_pt;
    double minDensity = DBL_MAX, maxDensity = DBL_MIN, meanDensity = 0;
    for (auto it = cloud->begin(); it != cloud->end(); ++it)
    {
        Point3DI pt;
        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;
        min_pt.x() = pt.x - voxel_size;
        min_pt.y() = pt.y - voxel_size;
        min_pt.z() = pt.z - voxel_size;
        max_pt.x() = pt.x + voxel_size;
        max_pt.y() = pt.y + voxel_size;
        max_pt.z() = pt.z + voxel_size;
        pt.intensity = octree.boxSearch(min_pt, max_pt, k_indices);
        if (pt.intensity < minDensity)
            minDensity = pt.intensity;
        if (pt.intensity > maxDensity)
            maxDensity = pt.intensity;
        meanDensity += pt.intensity;
        densityCloud->push_back(pt);
    }
    // Density: Points (1996000) Min (1.00), Max (316.00), Mean (85.62)
    meanDensity /= (int)cloud->size();
    ROS_INFO("Density: Points (%d) Min (%3.2f), Max (%3.2f), Mean (%3.2f)", (int)cloud->size(), minDensity, maxDensity, meanDensity);
    publishPointCloud(densityCloud, pub);
    savePointCloud(densityCloud, "/home/ahmad/personal_ws/src/ROSExamples/map/sample2.ply");
    
    ros::spin();

    return 0;
}