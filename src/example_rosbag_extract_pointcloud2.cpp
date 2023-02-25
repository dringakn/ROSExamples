/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Extract sensor_msgs/PointCloud2 messages from a rosbag
    Notes:
*/

#include <bits/stdc++.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h> // voxel filtering

int main(int argc, char const *argv[])
{
    std::string filepath = "~";
    std::string filename = "samplebag";
    std::string extension = ".bag";
    std::string topicname = "/velodyne_points";
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--filename") == 0)
        {
            std::filesystem::path pathObj(argv[i + 1]);
            filename = pathObj.stem().string();
            extension = pathObj.extension().string();
            filepath = pathObj.parent_path().string();
        }
        else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--topicname") == 0)
        {
            topicname = argv[i + 1];
        }
    }

    std::string bagfile = filepath + "/" + filename + extension;
    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(topicname);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::cout << "Messages found:" << view.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> merged_pointcloud; // Merged pointcloud
    unsigned int ctr = 0;
    for (const rosbag::MessageInstance &msg : view)
    {
        sensor_msgs::PointCloud2::ConstPtr pc_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != nullptr)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*pc_msg, cloud);
            merged_pointcloud += cloud;
            ctr++;
        }
    }
    bag.close();

    if (ctr > 0)
    {
        std::cout << "Voxel filtering... " << std::endl;

        // Perform Voxel Grid filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(merged_pointcloud.makeShared());
        float leaf_size = 0.25f;
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.filter(*filtered_cloud);

        std::string outputfile = filepath + "/" + filename + "_pointcloud.pcd";
        std::cout << "Saving " << outputfile << std::endl;
        pcl::io::savePCDFile(outputfile, *filtered_cloud, false);
    }

    return 0;
}
