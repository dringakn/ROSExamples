/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Extract pointclouds and tf from a rosbag. Register and downsample the pointcloud 
    Notes:
*/

#include <bits/stdc++.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

int main(int argc, char const *argv[])
{
    std::string filepath = "~";
    std::string filename = "samplebag";
    std::string extension = ".bag";
    std::string topicname = "/tf";
    float leaf_size = 0.25f;
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
        else if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--leaf") == 0)
        {
            leaf_size = atof(argv[i + 1]);
        }
    }

    std::string bagfile = filepath + "/" + filename + extension;
    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery({topicname, "/velodyne_points"}));
    std::cout << "Messages found:" << view.size() << std::endl;

    unsigned int ctr = 0;
    pcl::PointCloud<pcl::PointXYZ> merged_pointcloud; // Merged pointcloud
    tf::Transform tf_rob, tf_sens(tf::createQuaternionFromRPY(-1.3439, 0.0, -1.5708), tf::Vector3(0, 0, 0));

    for (const rosbag::MessageInstance &msg : view)
    {
        if (msg.getTopic() == "/tf")
        {
            tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
            if (tf_msg != nullptr)
            {
                // Loop through each transform in the message
                for (const auto &transform : tf_msg->transforms)
                {
                    // Check if the transform is from "odom" to "base_link"
                    if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link")
                    {
                        // Store robot transform: odom->base_link
                        tf::transformMsgToTF(transform.transform, tf_rob);
                    }
                }
            }
        }
        else if ((msg.getTopic() == "/velodyne_points") && (msg.getDataType() == "sensor_msgs/PointCloud2"))
        {
            sensor_msgs::PointCloud2::ConstPtr pc_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            pcl::PointCloud<pcl::PointXYZ> cloud_in, cloud_out;
            pcl::fromROSMsg(*pc_msg, cloud_in);

            // tf_sens: base_link -> velodyne
            pcl_ros::transformPointCloud(cloud_in, cloud_out, tf_rob*tf_sens);

            // Append Pointcloud
            merged_pointcloud += cloud_out;
            ctr++;
        }
    }
    bag.close();

    if (ctr > 0)
    {
        std::cout << "Voxel filtering, leaf_size: " << leaf_size << std::endl;
        std::cout << "Input points: " << merged_pointcloud.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(merged_pointcloud.makeShared());
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.filter(*filtered_cloud);
        std::cout << "Output points: " << filtered_cloud->size() << std::endl;

        std::string outputfile = filepath + "/" + filename + "_pointcloud.pcd";
        std::cout << "Saving " << ctr << ": " << outputfile << std::endl;
        pcl::io::savePCDFile(outputfile, *filtered_cloud, true);
    }

    return 0;
}
