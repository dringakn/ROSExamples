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

    // // Creating the KdTree object for the search method of the extraction
    // pcl::search::KdTree<Point3D>::Ptr tree(new pcl::search::KdTree<Point3D>);
    // tree->setInputCloud(filtered1);
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<Point3D> ec;
    // ec.setClusterTolerance(point_distance_cluster); // 2cm
    // ec.setMinClusterSize(min_points_per_cluster);
    // ec.setMaxClusterSize(maxDensity);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(filtered1);
    // ec.extract(cluster_indices);

    // int j = 0;
    // pcl::PointCloud<Point3D>::Ptr gaps(new pcl::PointCloud<Point3D>);
    // for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++)
    // {
    //     pcl::PointCloud<Point3D>::Ptr cloud_cluster(new pcl::PointCloud<Point3D>);
    //     pcl::CentroidPoint<Point3D> centroid;
    //     for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    //     {
    //         Point3D pt = (*filtered1)[*pit];
    //         cloud_cluster->push_back(pt); //*
    //         centroid.add(pt);
    //     }
    //     Point3D pt;
    //     centroid.get(pt);
    //     gaps->push_back(pt);

    //     cloud_cluster->width = cloud_cluster->size();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;
    //     std::cout << "PointCloud representing the Cluster: " << j << ", " << cloud_cluster->size() << " data points." << std::endl;
    // }

    // ss.str(std::string());
    // ss << "/home/ahmad/catkin_ws/src/gap-detection/map/gaps"
    //    << ".ply";
    // ROS_INFO("Saving gaps to %s", ss.str().c_str());
    // saved->height = 1;
    // saved->width = gaps->points.size();
    // saved->is_dense = false;
    // saved->header.frame_id = "map";
    // writer.write<Point3D>(ss.str(), *gaps, true, false);

    ros::spin();

    return 0;
}