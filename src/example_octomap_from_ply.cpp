/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <bits/stdc++.h>                     // C++ related stuff
#include <octomap/octomap.h>                 // Octree related stuff
#include <octomap_msgs/conversions.h>        //ROS Octomap message
#include <ros/ros.h>                         // ROS related stuff
#include <tf/tf.h>                           //tf
#include <visualization_msgs/MarkerArray.h>  // Octree
#include <pcl/point_cloud.h>                 // PCL
#include <pcl_conversions/pcl_conversions.h> // PCL <-> ROS
#include <pcl/io/ply_io.h>                   // File Read/Write
#include <pcl/filters/voxel_grid.h>          // Voxel grid filter

using namespace std;
typedef octomap::point3d PointOM;
typedef pcl::PointXYZ PointPCL;

void voxelGridFilter(pcl::PointCloud<PointPCL>::Ptr input, pcl::PointCloud<PointPCL>::Ptr output, double voxel_size)
{
    ROS_INFO("Voxel grid filtering ...");
    output->clear();
    pcl::VoxelGrid<PointPCL> filter;
    filter.setInputCloud(input);
    filter.setMinimumPointsNumberPerVoxel(1);
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter.filter(*output);
    ROS_INFO("Input: %d, Output: %d", (int)input->points.size(), (int)output->points.size());
}

void addPointCloud(octomap::ColorOcTree &ot, pcl::PointCloud<PointPCL>::Ptr input, bool occupied)
{
    ROS_INFO("Adding PCL pointcloud to the octomap ...");
    for (auto it = input->begin(); it != input->end(); ++it)
        ot.updateNode(it->x, it->y, it->z, occupied, true);
    ot.updateInnerOccupancy();
}

void addPointCloud(octomap::ColorOcTree &ot, pcl::PointCloud<PointPCL>::Ptr input)
{
    ROS_INFO("Adding octomap scan to the octomap ...");
    octomap::Pointcloud scan;
    scan.reserve(input->points.size());
    for (auto it = input->begin(); it != input->end(); ++it)
    {
        scan.push_back(it->x, it->y, it->z);
    }
    octomap::point3d sensor_origin(0, 0, 0);
    double max_range = -1;
    ot.insertPointCloud(scan, sensor_origin, max_range, false, false);
}

int main(int argc, char *argv[])
{
    cout.precision(5);
    cout.setf(cout.showpos);

    // Initialize the node and create node handle
    ros::init(argc, argv, "example_octomap_from_ply");
    ros::NodeHandle nh("~");

    // Get the parameters from server
    ROS_INFO("Parameters:");
    std::string map_file_name;
    if (!nh.param<std::string>("map_file_name", map_file_name, "map.ply"))
        nh.setParam("map_file_name", map_file_name);
    ROS_INFO("Map file: %s", map_file_name.c_str());

    float octree_resolution;
    if (!nh.param<float>("octree_resolution", octree_resolution, 0.25))
        nh.setParam("octree_resolution", octree_resolution);

    // Read point cloud from ply file
    ROS_INFO("Loading data:");
    pcl::PointCloud<PointPCL>::Ptr inputCloud(new pcl::PointCloud<PointPCL>);       // Input point cloud
    pcl::PointCloud<PointPCL>::Ptr downSampledCloud(new pcl::PointCloud<PointPCL>); // Voxel grid filtered

    pcl::PLYReader reader;
    int result = reader.read(map_file_name.c_str(), *inputCloud, 0);
    if (result == -1)
    {
        ROS_INFO("Pointcloud file (%s) could not be loaded.", map_file_name.c_str());
        return -1;
    }
    else
    {
        ROS_INFO("Loaded %dx%d points from %s", inputCloud->width, inputCloud->height, map_file_name.c_str());
        voxelGridFilter(inputCloud, downSampledCloud, octree_resolution);
    }

    // Create the octomap
    // octomap::OcTree ot(octree_resolution);
    ROS_INFO("Creating octomap ...");
    octomap::ColorOcTree ot(octree_resolution);
    // addPointCloud(ot, downSampledCloud, true);
    addPointCloud(ot, downSampledCloud);

    ROS_INFO("Octree created.");
    ROS_INFO("Pruning octree for compression...");
    ot.prune();
    ROS_INFO("Pruning completed.");

    // Save the ot. To visulize: octovis /tmp/sampleOcTree.bt
    ROS_INFO("Saving octomap to /home/ahmad/personal_ws/src/ROSExamples/map/PlY2OctoMap.bt ...");
    if (ot.writeBinary("/home/ahmad/personal_ws/src/ROSExamples/map/PlY2OctoMap.bt"))
        ROS_INFO("Saving done.");
    else
        ROS_INFO("Saving failed.");

    // Load the octre
    ROS_INFO("Loading octomap from /home/ahmad/personal_ws/src/ROSExamples/map/PlY2OctoMap.bt ...");
    if (ot.readBinary("/home/ahmad/personal_ws/src/ROSExamples/map/PlY2OctoMap.bt"))
        ROS_INFO("Octree successfully loaded from /home/ahmad/personal_ws/src/ROSExamples/map/PlY2OctoMap.bt");
    else
        ROS_INFO("Could not load ot from /home/ahmad/personal_ws/src/ROSExamples/map/PlY2OctoMap.bt");

    // Set properties
    const int limit = 20;
    PointOM tempMax(limit, limit, limit);
    ot.setBBXMax(tempMax);
    PointOM tempMin(-limit, -limit, -limit);
    ot.setBBXMin(tempMin);

    // Octree stats
    PointOM bound = ot.getBBXBounds();
    cout << "Bounding Box:" << bound << endl;
    PointOM center = ot.getBBXCenter();
    cout << "Bounding Box Center:" << center << endl;
    PointOM bbxMax = ot.getBBXMax();
    cout << "Bounding Box Max:" << bbxMax << endl;
    PointOM bbxMin = ot.getBBXMin();
    cout << "Bounding Box Min:" << bbxMin << endl;
    double clampThreshMax = ot.getClampingThresMax();
    cout << "Clamping Threshold Max:" << clampThreshMax << endl;
    double clampThreshMin = ot.getClampingThresMin();
    cout << "Clamping Threshold Min:" << clampThreshMin << endl;

    // Limit: |MetricMax-MetricMin|
    double x, y, z;
    ot.getMetricMax(x, y, z);
    cout << "Metric Max:" << x << ',' << y << ',' << z << endl;
    ot.getMetricMin(x, y, z);
    cout << "Metric Min:" << x << ',' << y << ',' << z << endl;
    octree_resolution = ot.getResolution();
    cout << "Resolution:" << octree_resolution << endl;
    int depth = ot.getTreeDepth();
    cout << "Depth:" << depth << endl;
    double pHit = ot.getProbHit();
    cout << "pHit:" << pHit << endl;
    double pMiss = ot.getProbMiss();
    cout << "pMiss:" << pMiss << endl;
    int leafs = ot.getNumLeafNodes();
    cout << "Leafs:" << leafs << endl;
    int nodes = ot.calcNumNodes();
    cout << "Nodes:" << nodes << endl;
    double volume = ot.volume(); // length x width x height
    cout << "Volume:" << volume << endl;
    int memoryUsage = ot.memoryUsage(); // Binary octree
    cout << "Memory Usage:" << memoryUsage << endl;
    double thresh = ot.getOccupancyThres();
    int memoryUsageNode = ot.memoryUsageNode(); // Node size 16 Bytes = 2 x 8
    cout << "Memory Usage Node:" << memoryUsageNode << endl;
    int memoryFullGrid = ot.memoryFullGrid(); // Including metadata (Bytes)
    cout << "Memory Full Grid:" << memoryFullGrid << endl;
    cout << "Occupancy Threshold:" << thresh << endl;
    // getUnknownLeafCenters(...) return centers of leafs that do NOT exist (but
    // could) in a given bounding box
    ROS_INFO("ot.getUnknownLeafCenters(list, bbxMin, bbxMax, 15): ");
    list<octomath::Vector3> list;
    // 15=124427, 14=15343, 13=1600, 12=277, 11=9, 10=0
    // 125000-x=124427
    ot.getUnknownLeafCenters(list, bbxMin, bbxMax, 15);
    // for (auto &&pt : list)
    // cout << pt << endl;
    cout << "Unknown points: " << list.size() << endl;

    // Multi-level octree_resolution, VoxelLength = res × 2^(16-depth)
    // 10=6.4, 11=3.2, 12=1.6, 13=0.8, 14=0.4, 15=0.2, 16=0.1
    // ROS_INFO("Multi-level leaf nodes Centers[depth][voxelLength] -> LogOdds:");
    // for (int i = 1; i <= 11; i++)
    //     for (auto it = ot.begin_leafs(i); it != ot.end_leafs(); ++it)
    //         cout << it.getCoordinate() << '[' << it.getDepth() << ']' << '['
    //              << it.getSize() << ']' << " -> " << it->getValue() << endl;

    // change detection, voxels whose logodds changes
    // ot.enableChangeDetection(true);
    // ot.isChangeDetectionEnabled();
    // ot.numChangesDetected();
    // ot.resetChangeDetection();

    // Visulize the ot, create Octomap publisher
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
    // Create Octomap message
    octomap_msgs::Octomap msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time(0);
    octomap_msgs::binaryMapToMsg(ot, msg);
    //   msg.id = "OcTree"; // OcTree, ColorOcTree
    // Perodic code
    ros::Rate rate(1);
    while (ros::ok())
    {
        // Publish octomap
        pub.publish(msg);
        // Yield control to ROS
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}