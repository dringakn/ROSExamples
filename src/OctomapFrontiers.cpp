#include "../include/OctomapFrontiers.h"

namespace octomap_frontiers
{

    OctomapFrontiers::OctomapFrontiers() : octomap_server::OctomapServer()
    {
        ros::NodeHandle private_nh("~");
        pubChangeSet = private_nh.advertise<sensor_msgs::PointCloud2>("/octomap_change", 1);
        m_octree->enableChangeDetection(true);
    }

    OctomapFrontiers::~OctomapFrontiers()
    {
    }

    void OctomapFrontiers::insertScan(const tf::Point &sensorOrigin, const PCLPointCloud &ground, const PCLPointCloud &nonground)
    {
        octomap_server::OctomapServer::insertScan(sensorOrigin, ground, nonground);
        pcl::PointCloud<pcl::PointXYZI> changedCells = pcl::PointCloud<pcl::PointXYZI>();

        for (auto iter = m_octree->changedKeysBegin(); iter != m_octree->changedKeysEnd(); ++iter)
        {
            octomap::OcTreeNode *node = m_octree->search(iter->first);
            octomap::point3d center = m_octree->keyToCoord(iter->first);
            pcl::PointXYZI pt;
            pt.x = center(0);
            pt.y = center(1);
            pt.z = center(2);
            // pt.intensity = (m_octree->isNodeOccupied(node)) ? 1000 : -1000;
            if (m_octree->isNodeOccupied(node))
            {
                pt.intensity = 1000;
                changedCells.push_back(pt);
            }
        }

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(changedCells, msg);
        msg.header.frame_id = "odom";
        msg.header.stamp = ros::Time().now();
        pubChangeSet.publish(msg);
        m_octree->resetChangeDetection();
    }

} /* namespace octomap_frontiers */