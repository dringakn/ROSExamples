/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Create an octree from Octree::PointCloud and transormation.
*/
#include <bits/stdc++.h>              // C++ related stuff
#include <octomap/octomap.h>          // Octree related stuff
#include <octomap_msgs/conversions.h> //ROS Octomap message
#include <ros/ros.h>                  // ROS related stuff
#include <octomap/math/Utils.h>       // DEG2RAD

using namespace std;
typedef octomap::point3d PointOM;
typedef octomath::Pose6D Pose6D;
typedef octomath::Quaternion Quaternion;
typedef octomath::Vector3 Vector3;
typedef octomap::Pointcloud Pointcloud;

void publishOctree(ros::Publisher &pub, octomap::OcTree &ot)
{
    // Create Octomap message
    octomap_msgs::Octomap msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    // octomap_msgs::binaryMapToMsg(ot, msg);   // Doesn't contains the occupancy value
    octomap_msgs::fullMapToMsg(ot, msg); // Occupancy is bright green to dark green color
    pub.publish(msg);
}

unsigned int countChildrens(octomap::OcTreeNode &node)
{
    unsigned int childrens = 0;
    for (unsigned int i = 0; i < 8; i++)
    {
        if (node.childExists(i))
            childrens++;
    }
    return childrens;
}

void printOctreeConfigurations(octomap::OcTree &ot)
{
    double x, y, z;
    PointOM bbxMin = ot.getBBXMin();
    PointOM bbxMax = ot.getBBXMax();
    ROS_INFO("<<<\tOctree Configuraitons\t>>>");
    ROS_INFO_STREAM("Type: " << ot.getTreeType());
    ROS_INFO_STREAM("Resolution: " << ot.getResolution());
    ROS_INFO_STREAM("Occupancy Threshold: " << ot.getOccupancyThres());
    ROS_INFO_STREAM("OccupancyThresLog: " << ot.getOccupancyThresLog());
    ROS_INFO_STREAM("pHit: " << ot.getProbHit());
    ROS_INFO_STREAM("pMiss: " << ot.getProbMiss());
    ROS_INFO_STREAM("Clamping Threshold Max: " << ot.getClampingThresMax());
    ROS_INFO_STREAM("Clamping Threshold Min: " << ot.getClampingThresMin());
    ROS_INFO_STREAM("ChangeDetection?: " << ot.isChangeDetectionEnabled());
    ROS_INFO_STREAM("NumChangesDetected: " << ot.numChangesDetected());
    ROS_INFO_STREAM("Leafs: " << ot.getNumLeafNodes());
    ROS_INFO_STREAM("Nodes: " << ot.calcNumNodes());
    ROS_INFO_STREAM("Size: " << ot.size());                       // Same as calcNumNodes()
    ROS_INFO_STREAM("Volume[m3]: " << ot.volume());               // length x width x height
    ROS_INFO_STREAM("Memory Usage[Bytes]: " << ot.memoryUsage()); // Binary octree
    ROS_INFO_STREAM("Memory Usage Per Node: " << ot.memoryUsageNode() << " (2 x 8 Bytes)");
    ROS_INFO_STREAM("Memory Full Grid[Bytes]: " << ot.memoryFullGrid()); // Including metadata (Bytes)
    ROS_INFO_STREAM("Bounding Box Set?: " << ot.bbxSet());
    ROS_INFO_STREAM("Bounding Box Bounds: " << ot.getBBXBounds());
    ROS_INFO_STREAM("Bounding Box Center: " << ot.getBBXCenter());
    ROS_INFO_STREAM("Bounding Box Max: " << ot.getBBXMax());
    ROS_INFO_STREAM("Bounding Box Min: " << ot.getBBXMin());
    ot.getMetricMax(x, y, z);
    ROS_INFO("Metric Max: (%3.2f, %3.2f, %3.2f)", x, y, z);
    ot.getMetricMin(x, y, z);
    ROS_INFO("Metric Min: (%3.2f, %3.2f, %3.2f)", x, y, z);
    ot.getMetricSize(x, y, z);
    ROS_INFO("Metric Size: (%3.2f, %3.2f, %3.2f)", x, y, z);
    // ROS_INFO_STREAM("Is (0, 0, 0) in Bounding Box?: " << ot.inBBX(PointOM(0, 0, 0)));
    // ROS_INFO_STREAM("Is (1, 1, 1) in Bounding Box?: " << ot.inBBX(PointOM(1, 1, 1)));
    // ROS_INFO_STREAM("Is (2, 2, 2) in Bounding Box?: " << ot.inBBX(PointOM(2, 2, 2)));
    // ROS_INFO_STREAM("Is (-1, -1, -1) in Bounding Box?: " << ot.inBBX(PointOM(-1, -1, -1)));
    // ROS_INFO_STREAM("Is (-1.1, -1.1, -1.1) in Bounding Box?: " << ot.inBBX(PointOM(-1.1, -1.1, -1.1)));
    // ROS_INFO_STREAM("Is (-1.05, -1.05, -1.05) in Bounding Box?: " << ot.inBBX(PointOM(-1.05, -1.05, -1.05)));
    ROS_INFO_STREAM("Depth: " << ot.getTreeDepth());
    ROS_INFO_STREAM("Depth\tVoxelSide[m]\tLeaf\tInner\tOccupied\tFree\tUnknown");
    for (unsigned int depth = 1; depth <= 16; depth++)
    {
        unsigned int leafNode = 0, innerNode = 0, occupiedNode = 0, freeNode = 0, unknownNode = 0;
        for (auto it = ot.begin_tree(depth); it != ot.end_tree(); ++it)
        {
            if (it.isLeaf())
            {
                leafNode++;
                // Only leaf node can be occupied/free
                (ot.isNodeOccupied(*it)) ? occupiedNode++ : freeNode++;
            }
            else
            {
                innerNode++;
            }
        }
        list<octomath::Vector3> list;
        ot.getUnknownLeafCenters(list, bbxMin, bbxMax, depth); // if (depth == 0) depth = tree_depth;
        ROS_INFO_STREAM(depth << "\t" << ot.getNodeSize(depth) << "\t\t" << leafNode << "\t" << innerNode << "\t" << occupiedNode << "\t\t" << freeNode << "\t" << list.size());
        // for (auto &&pt : list)
        //     cout << pt << endl;
    }

    // Count nodes with similar number of childrens [0 .. 8], 0=max depth i.e. 16
    std::map<unsigned int, unsigned int> childs;
    for (auto it = ot.begin_tree(0); it != ot.end_tree(); it++)
        childs[countChildrens(*it)]++;
    for (unsigned int i = 0; i < childs.size(); i++)
        ROS_INFO("Nodes with %d Children: %d", i, childs[i]);

    // Use Octovis to view the structure: octovis sample_octomap.ot
    ot.write("/home/ahmad/personal_ws/src/ROSExamples/map/sample_octomap.ot");
}

void printChanges(octomap::OcTree &tree)
{
    tree.expand();

    // iterate through the changed nodes
    unsigned int changedOccupied = 0;
    unsigned int changedFree = 0;
    unsigned int missingChanged = 0;
    for (auto it = tree.changedKeysBegin(); it != tree.changedKeysEnd(); it++)
    {
        octomap::OcTreeNode *node = tree.search(it->first);
        if (node != NULL)
        {
            if (tree.isNodeOccupied(node))
                changedOccupied += 1;
            else
                changedFree += 1;
        }
        else
        {
            missingChanged += 1;
        }
    }
    std::cout << "change detection: " << changedOccupied << " occ; " << changedFree << " free; " << missingChanged << " missing" << std::endl;

    // iterate through the entire tree
    unsigned int actualOccupied = 0;
    unsigned int actualFree = 0;
    for (auto it = tree.begin_tree(), end = tree.end_tree(); it != end; ++it)
    {
        if (it.isLeaf())
        {
            if (tree.isNodeOccupied(*it))
                actualOccupied += 1;
            else
                actualFree += 1;
        }
    }
    std::cout << "actual: " << actualOccupied << " occ; " << actualFree << " free; " << std::endl;

    tree.prune();
}

void exampleSphere(ros::Publisher &pub, octomap::OcTree &ot)
{
    // Create s sphere
    PointOM origin(0.0f, 0.0f, 0.0f);
    PointOM point_on_surface(2.0f, 0.0f, 0.0f);
    ot.clear();
    std::cout << "generating spherical scan at " << origin << " ..." << std::endl;
    for (int i = -100; i < 101; i++)
    {
        Pointcloud cloud;
        for (int j = -100; j < 101; j++)
        {
            PointOM rotated = point_on_surface;
            rotated.rotate_IP(0, DEG2RAD(i * 0.5), DEG2RAD(j * 0.5));
            cloud.push_back(rotated);
        }

        // insert in global coordinates:
        ot.insertPointCloud(cloud, origin, -1, false);
    }
    ot.resetChangeDetection(); // change detection, voxels whose logodds changes
    printOctreeConfigurations(ot);
    printChanges(ot);
    publishOctree(pub, ot);
    ros::spin();
}

void exampleCube(ros::Publisher &pub, octomap::OcTree &ot)
{
    bool lazy_eval = false;
    // Test Nodes with no neighbours
    ot.clear();
    ot.updateNode(1, 1, 1, true, lazy_eval);
    ot.updateNode(-1, 1, 1, true, lazy_eval);
    ot.updateNode(-1, -1, 1, true, lazy_eval);
    ot.updateNode(1, -1, 1, true, lazy_eval);
    ot.updateNode(1, 1, -1, true, lazy_eval);
    ot.updateNode(-1, 1, -1, true, lazy_eval);
    ot.updateNode(-1, -1, -1, true, lazy_eval);
    ot.updateNode(1, -1, -1, true, lazy_eval);
    printOctreeConfigurations(ot);
    printChanges(ot);
    publishOctree(pub, ot);
    for (auto it = ot.begin_tree(16); it != ot.end_tree(); it++)
    {
        if (!it->hasChildren())
        {
            ROS_INFO_STREAM("No child: " << it.getCoordinate());
        }
    }
    ros::spin();
}

void examplePlane(ros::Publisher &pub, octomap::OcTree &ot)
{
    // Test Nodes with no neighbours in a plane
    double octree_resolution = ot.getResolution();
    bool lazy_eval = false;
    ot.clear();
    for (float x = -1; x < 1; x += octree_resolution)
        for (float y = -1; y < 1; y += octree_resolution)
            ot.updateNode(x, y, 0, true, lazy_eval);
    printOctreeConfigurations(ot);
    printChanges(ot);
    publishOctree(pub, ot);
}

void exampleIndexKey(ros::Publisher &pub, octomap::OcTree &ot)
{
    bool lazy_eval = false;
    ot.clear();
    ot.updateNode(2, 4, 1, true, lazy_eval);
    ot.updateNode(2, 4, 2, true, lazy_eval);
    ot.updateNode(2, 4, 3, false, lazy_eval);
    ot.updateNode(2, 4, 4, true, lazy_eval);
    printOctreeConfigurations(ot);
    printChanges(ot);
    publishOctree(pub, ot);

    for (unsigned int depth = 3; depth <= 16; depth++)
    {
        ROS_INFO_STREAM("Octree depth: " << depth << "[" << ot.getNodeSize(depth) << " m]");
        for (auto it = ot.begin_leafs(depth); it != ot.end_leafs(); ++it)
        {
            ROS_INFO_STREAM(it.getCoordinate() << " Occupied(" << ot.isNodeOccupied(*it)
                                               << ") IndexKey(" << it.getIndexKey().k[0] << ", " << it.getIndexKey().k[1] << ", " << it.getIndexKey().k[2]
                                               << ") Key(" << it.getKey().k[0] << ", " << it.getKey().k[1] << ", " << it.getKey().k[2] << ")");
        }
    }
}

void exampleVoxelChangeDetection(ros::Publisher &pub, octomap::OcTree &ot)
{
    bool lazy_eval = false;
    ot.clear();
    double octree_resolution = ot.getResolution();

    ot.updateNode(0, 0, 0, true, lazy_eval);       // Create an occupied voxel
    ot.updateNode(0.5, 0.5, 0, false, lazy_eval);  // Create a free voxel
    ot.updateNode(0.75, 0.75, 0, true, lazy_eval); // Create an occupied voxel
    ot.deleteNode(-1, -1, 0, 0);                   // Delete voxel
    printChanges(ot);
    ot.updateNode(0, 0, 1, true, lazy_eval);       // Create an occupied voxel
    ot.updateNode(0.5, 0.5, 1, false, lazy_eval);  // Create a free voxel
    ot.updateNode(0.75, 0.75, 1, true, lazy_eval); // Create an occupied voxel
    ot.deleteNode(-1, -1, 0, 1);                   // Delete voxel
    printChanges(ot);
    ot.updateNode(0, 0, 2, true, lazy_eval);       // Create an occupied voxel
    ot.updateNode(0.5, 0.5, 2, false, lazy_eval);  // Create a free voxel
    ot.updateNode(0.75, 0.75, 2, true, lazy_eval); // Create an occupied voxel
    ot.deleteNode(-1, -1, 0, 2);                   // Delete voxel
    printChanges(ot);

    printOctreeConfigurations(ot);
    publishOctree(pub, ot);

    PointOM searchDistance(0.5, 0.5, 0.5);                            // Neighbourhood distance
    int depth = floor(16 - (log(1.0 / ot.getResolution()) / log(2))); // Depth corresponding to 1m resolution
    unsigned int maxVoxels = pow(1.0 / ot.getResolution(), 3.0);      // Max. voxels within neighbourhood

    // Extract occupied, free, and unknown voxels using leaf bounding box
    // Now, I could create an color octree with r=unknown, g=free, b=occupied
    for (auto leaf = ot.begin_leafs(0); leaf != ot.end_leafs(); ++leaf)
    {
        PointOM pt = leaf.getCoordinate();
        PointOM lower = pt - searchDistance;
        PointOM upper = pt + searchDistance;
        unsigned int occ = 0, free = 0, unknown = 0;
        for (auto it = ot.begin_leafs_bbx(lower, upper, 0); it != ot.end_leafs_bbx(); ++it)
        {
            (ot.isNodeOccupied(*it)) ? occ++ : free++;
        }
        unknown = maxVoxels - occ - free;
        ROS_INFO_STREAM(pt << " Occupied(" << occ << ") Free(" << free << ") Unknown(" << unknown << ")");
    }
}

void exampleVoxelNeighbourhood(ros::Publisher &pub, octomap::OcTree &ot)
{
    bool lazy_eval = false;
    ot.clear();
    double octree_resolution = ot.getResolution();

    ot.updateNode(0, 0, 0, true, lazy_eval);       // Create an occupied voxel
    ot.updateNode(0.5, 0.5, 0, false, lazy_eval);  // Create a free voxel
    ot.updateNode(0.75, 0.75, 0, true, lazy_eval); // Create an occupied voxel

    // for (float x = 0; x < 1; x += octree_resolution)
    //     for (float y = 0; y < 1; y += octree_resolution)
    //         for (float z = 0; z < 1; z += octree_resolution)
    //             ot.updateNode(x, y, z, ((rand() / (double)RAND_MAX) > 0.5), lazy_eval);

    printOctreeConfigurations(ot);
    printChanges(ot);
    publishOctree(pub, ot);

    PointOM searchDistance(0.5, 0.5, 0.5);                            // Neighbourhood distance
    int depth = floor(16 - (log(1.0 / ot.getResolution()) / log(2))); // Depth corresponding to 1m resolution
    unsigned int maxVoxels = pow(1.0 / ot.getResolution(), 3.0);      // Max. voxels within neighbourhood

    // Extract unknown voxels using unknownleafcenters
    // for (auto it = ot.begin_leafs(0); it != ot.end_leafs(); ++it)
    // {
    //     PointOM pt = it.getCoordinate();
    //     PointOM lower = pt - searchDistance;
    //     PointOM upper = pt + searchDistance;
    //     std::list<octomath::Vector3> unknown;
    //     ot.getUnknownLeafCenters(unknown, lower, upper, 0);
    //     ROS_INFO_STREAM(pt << lower << upper << "Occupied(" << ot.isNodeOccupied(*it)
    //                        << ") Unknown(" << unknown.size() << ")");
    //     // for (auto &&p : unknown)
    //     //     ROS_INFO_STREAM(p);
    // }

    // Extract occupied, free, and unknown voxels using leaf bounding box
    // Now, I could create an color octree with r=unknown, g=free, b=occupied
    for (auto leaf = ot.begin_leafs(0); leaf != ot.end_leafs(); ++leaf)
    {
        PointOM pt = leaf.getCoordinate();
        PointOM lower = pt - searchDistance;
        PointOM upper = pt + searchDistance;
        unsigned int occ = 0, free = 0, unknown = 0;
        for (auto it = ot.begin_leafs_bbx(lower, upper, 0); it != ot.end_leafs_bbx(); ++it)
        {
            (ot.isNodeOccupied(*it)) ? occ++ : free++;
        }
        unknown = maxVoxels - occ - free;
        ROS_INFO_STREAM(pt << " Occupied(" << occ << ") Free(" << free << ") Unknown(" << unknown << ")");
    }
}

void exampleNeighbours(ros::Publisher &pub, octomap::OcTree &ot)
{
    bool lazy_eval = false;
    ot.clear();
    for (double z = 0; z < 4; z += ot.getResolution())
    {
        ot.updateNode(2, 4, z, true, lazy_eval);
    }

    printOctreeConfigurations(ot);
    printChanges(ot);
    publishOctree(pub, ot);

    for (unsigned int depth = 3; depth <= 16; depth++)
    {
        ROS_INFO_STREAM("Octree depth: " << depth << "[" << ot.getNodeSize(depth) << " m]");
        for (auto it = ot.begin_leafs(depth); it != ot.end_leafs(); ++it)
        {
            ROS_INFO_STREAM(it.getCoordinate() << " Occupied(" << ot.isNodeOccupied(*it)
                                               << ") IndexKey(" << it.getIndexKey().k[0] << ", " << it.getIndexKey().k[1] << ", " << it.getIndexKey().k[2]
                                               << ") Key(" << it.getKey().k[0] << ", " << it.getKey().k[1] << ", " << it.getKey().k[2] << ")");
        }
    }
}

void findNeighbours(ros::Publisher &pub, octomap::OcTree &ot)
{
    // Save prior neighbours
    PointOM prevBBXMax = ot.getBBXMax();
    PointOM prevBBXMin = ot.getBBXMin();
    PointOM searchDistance(0.5, 0.5, 0.5);                            // Neighbourhood distance
    int depth = floor(16 - (log(1.0 / ot.getResolution()) / log(2))); // Depth corresponding to 1m resolution
    ROS_INFO("Depth @ 1m: %d", depth);

    for (auto it = ot.begin_leafs(0); it != ot.end_leafs(); ++it)
    {
        PointOM pt = it.getCoordinate();
        PointOM lower = pt - searchDistance;
        PointOM upper = pt + searchDistance;

        std::list<octomath::Vector3> unknown;
        ot.getUnknownLeafCenters(unknown, lower, upper, depth);
        ROS_INFO_STREAM(pt << lower << upper << "Occupied(" << ot.isNodeOccupied(*it)
                           << ") Unknown(" << unknown.size() << ")");
        for (auto &&p : unknown)
            // for (auto p = unknown.begin(); p != unknown.end(); ++p)
            ROS_INFO_STREAM(p);
    }
}

int main(int argc, char *argv[])
{
    std::cout.precision(5);
    std::cout.setf(std::cout.showpos);

    // Initialize the node and create node handle
    ros::init(argc, argv, "example_octomap_basics");
    ros::NodeHandle nh("~");

    // Get the parameters from server
    ROS_INFO("Parameters:");
    float octree_resolution;
    if (!nh.param<float>("octree_resolution", octree_resolution, 0.1))
        nh.setParam("octree_resolution", octree_resolution);
    ROS_INFO_STREAM("octree_resolution: " << octree_resolution);

    // Visulize the ot, create Octomap publisher
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1, true);

    // Create the octomap, 2x2x2=8m3
    ROS_INFO("Creating octomap ...");
    octomap::OcTree ot(octree_resolution);
    PointOM bbxMax(2, 2, 2), bbxMin(-2, -2, -2);
    bool lazy_eval = false; // Effects number of nodes; false no effect on pruning

    ot.setOccupancyThres(0.5);      // 0.50
    ot.setClampingThresMax(0.971);  // 0.971
    ot.setClampingThresMin(0.119);  // 0.1192
    ot.setProbHit(0.70);            // 0.70
    ot.setProbMiss(0.40);           // 0.40
    ot.enableChangeDetection(true); // Enable change detection
    ot.useBBXLimit(true);           // Enable BBX utilities
    ot.setBBXMax(bbxMax);           // Upper
    ot.setBBXMin(bbxMin);           // Lower

    // Insert a ray from (0,0,0) -> (1, 1, 1)
    // ot.insertPointCloudRays(scan, rigin, -1, false);
    // printOctreeConfigurations(ot);
    // printChanges(ot);
    // publishOctree(pub, ot);
    // ros::spin();

    // exampleSphere(pub, ot);
    // exampleneighbour(pub, ot);
    // exampleNeighbours(pub, ot);
    // findNeighbours(pub, ot);
    // exampleVoxelNeighbourhood(pub, ot);
    exampleVoxelChangeDetection(pub, ot);
    ros::spin();

    // Create all cells as free
    ROS_INFO("Populating Octree...");
    ot.clear(); // Delete tree structure
    for (float x = -1; x < 1; x += octree_resolution)
        for (float y = -1; y < 1; y += octree_resolution)
            for (float z = -1; z < 1; z += octree_resolution)
                ot.updateNode(x, y, z, false, lazy_eval);
    // Create occupied cells
    ot.updateNode(1, 1, 1, true, lazy_eval);
    ot.updateNode(-1, 1, 1, true, lazy_eval);
    ot.updateNode(-1, -1, 1, true, lazy_eval);
    ot.updateNode(1, -1, 1, true, lazy_eval);
    ot.updateNode(1, 1, -1, true, lazy_eval);
    ot.updateNode(-1, 1, -1, true, lazy_eval);
    ot.updateNode(-1, -1, -1, true, lazy_eval);
    ot.updateNode(1, -1, -1, true, lazy_eval);
    // Create unknown cells
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(0.5, 0.5, 0.5, 16));
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(-0.5, 0.5, 0.5, 16));
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(-0.5, -0.5, 0.5, 16));
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(0.5, -0.5, 0.5, 16));
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(0.5, 0.5, -0.5, 16));
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(-0.5, 0.5, -0.5, 16));
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(-0.5, -0.5, -0.5, 16));
    ROS_INFO_STREAM("Octree deleteNode: " << ot.deleteNode(0.5, -0.5, -0.5, 16));
    ot.updateInnerOccupancy(); // set inner node colors
    ROS_INFO_STREAM("Octree size before pruning: " << ot.size() << " nodes");
    // ot.toMaxLikelihood();
    ot.prune();
    ROS_INFO_STREAM("Octree size after pruning: " << ot.size() << " nodes");
    printOctreeConfigurations(ot);
    printChanges(ot);
    // publishOctree(pub, ot);
    // ros::spin();

    // Search a point in the tree
    // Search node using point3d, key or xyz. Returns a node or null(Unknown)
    for (int i = 0; i < 3; i++)
    {
        PointOM queryPt(1, 1, 1);
        ot.updateNode(queryPt, true, lazy_eval); // Re-Update to modify probability, piror:0.61
        ROS_INFO_STREAM("Querying Octree " << queryPt);
        octomap::OcTreeNode *result = ot.search(queryPt, 0); // NULL = Unknown
        if (result != NULL)
            ROS_INFO("Querypt Occupancy: %5.2f, %d, %d", result->getOccupancy(), ot.isNodeOccupied(result), ot.isNodeAtThreshold(result));
        else
            ROS_INFO("Querypt in an unkown voxel.");

        PointOM queryPt2(1, 1, -1);
        ot.updateNode(queryPt2, false, lazy_eval); // Re-Update to modify probability, piror:0.61
        ROS_INFO_STREAM("Querying Octree " << queryPt2);
        octomap::OcTreeNode *result2 = ot.search(queryPt2, 0); // NULL = Unknown
        if (result2 != NULL)
            ROS_INFO("Querypt Occupancy: %5.2f, %d, %d", result2->getOccupancy(), ot.isNodeOccupied(result2), ot.isNodeAtThreshold(result2));
        else
            ROS_INFO("Querypt in an unkown voxel.");
        // ot.updateInnerOccupancy(); // set inner node colors
        // ot.updateNode(queryPt2, true, lazy_eval); // Re-Update to modify probability, piror:0.61
    }
    PointOM queryPt(1, 1, -1);
    ot.updateNode(queryPt, true, lazy_eval); // Re-Update to modify probability, piror:0.61
    ROS_INFO_STREAM("Querying Octree " << queryPt);
    octomap::OcTreeNode *result = ot.search(queryPt, 0); // NULL = Unknown
    if (result != NULL)
        ROS_INFO("Querypt Occupancy: %5.2f, %d, %d", result->getOccupancy(), ot.isNodeOccupied(result), ot.isNodeAtThreshold(result));
    else
        ROS_INFO("Querypt in an unkown voxel.");

    // Scanning Nodes/Leafs
    // int idx = 0;
    // for (auto it = ot.begin_tree(16); it != ot.end_tree(); ++it)
    // {
    //     unsigned int childrens = 0;
    //     for (unsigned int i = 0; i < 8; i++)
    //     {
    //         if (it->childExists(i))
    //             childrens++;
    //     }
    //     if (childrens == 0 && it.getDepth() == 16)
    //         ROS_INFO_STREAM("Leaf " << idx++ << " :" << it.getCoordinate() << " :" << childrens << " :" << it.getDepth() << " :" << it.getSize());
    // }
    ROS_INFO_STREAM("CoordToKey: " << queryPt << " -> " << ot.coordToKey(queryPt, 15).k[0]);

    // Multi-level octree_resolution, VoxelLength = res × 2^(16-depth)
    // 10=6.4, 11=3.2, 12=1.6, 13=0.8, 14=0.4, 15=0.2, 16=0.1
    // ROS_INFO("Multi-level leaf nodes Centers[depth][voxelLength] -> LogOdds = getMeanChildLogOdds");
    // for (unsigned int i = 15; i <= 15; i++)
    //     for (auto it = ot.begin_leafs(i); it != ot.end_leafs(); ++it)
    //         cout << it.getCoordinate() << '[' << it.getDepth() << ']' << '['
    //              << it.getSize() << ']' << " -> " << it->getValue() << " = " << it->getMeanChildLogOdds() << endl;

    publishOctree(pub, ot);

    ros::spin();
    return 0;
}