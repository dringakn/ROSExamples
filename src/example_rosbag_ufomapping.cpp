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

#include <ufo/map/occupancy_map.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

int main(int argc, char const *argv[])
{
    std::string filepath = "~";
    std::string filename = "samplebag";
    std::string extension = ".bag";
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
        else if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--leaf") == 0)
        {
            leaf_size = atof(argv[i + 1]);
        }
    }

    std::string bagfile = filepath + "/" + filename + extension;
    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery({std::string("/tf"),
                                               std::string("/velodyne_points")}));

    std::cout << "Messages found:" << view.size() << std::endl;

    unsigned int ctr = 0;
    ufo::map::OccupancyMap map(leaf_size, 16, true, 0.5, 0.5, 0.7, 0.4, 0.1192, 0.971);
    // map.enableAutomaticPruning(true);
    // map.enableChangeDetection(true);
    // map.enableMinMaxChangeDetection(true);

    tf::Transform tf_rob;
    tf::Transform tf_sens(tf::createQuaternionFromRPY(-1.3439, 0.0, -1.5708), tf::Vector3(0, 0, 0));

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
            if (ctr++ % 10 == 0)
            {
                sensor_msgs::PointCloud2::ConstPtr pc_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                ufo::map::PointCloud cloud;
                ufomap_ros::rosToUfo(*pc_msg, cloud);

                tf::Transform tf_ = tf_rob * tf_sens; // tf_sens: base_link -> velodyne
                tf::Vector3 pos = tf_.getOrigin();
                tf::Quaternion quat = tf_.getRotation();
                ufo::math::Pose6 tf(pos.x(), pos.y(), pos.z(),
                                    quat.w(), quat.x(), quat.y(), quat.z());
                cloud.transform(tf, true);

                map.insertPointCloud(tf.translation(), cloud, 60, 0, true, 0, true);
                // map.insertPointCloudDiscrete(tf.translation(), cloud, 100, 2, true, 0, true);

                std::cout << pos.x() << ", " << pos.y() << ", " << pos.z() << ", " << ctr << std::endl;
            }
        }
    }
    bag.close();

    if (ctr > 0)
    {
        std::string outputfile = filepath + "/" + filename + "_ufomapping.ufo";
        std::cout << ctr << "Saving UFOMap: " << outputfile << std::endl;
        map.write(outputfile, false, 0, 1, 0);
    }

    return 0;
}
