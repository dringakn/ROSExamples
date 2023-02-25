/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Extract UFOMaps from a rosbag.
    Notes:
*/

#include <bits/stdc++.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ufo/map/occupancy_map.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char const *argv[])
{
    std::string filepath = "~";
    std::string filename = "samplebag";
    std::string extension = ".bag";
    std::string topicname = "/ufomap_mapping_server_node/map";
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

    ufomap_msgs::UFOMapStamped::ConstPtr msg;
    for (const rosbag::MessageInstance &m : view)
    {
        msg = m.instantiate<ufomap_msgs::UFOMapStamped>();
    }
    bag.close();

    if (msg != NULL)
    {
        ufo::map::OccupancyMap map(1, 21);
        if (ufomap_msgs::msgToUfo(msg->map, map))
        {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.height = 1;
            cloud.width = 0;
            cloud.is_dense = true;

            for (auto it = map.beginLeaves(true, false, false, true, 0); it != map.endLeaves(); ++it)
                cloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));

            std::string outputfile = filepath + "/" + filename + "_ufo.pcd";
            std::cout<< "Saving " << outputfile << std::endl;
            pcl::io::savePCDFile(outputfile, cloud, false);
            // ufo::geometry::BoundingVolume bv;
            // map.write("/home/ahmad/delme.ufo", bv, false, 0);
        }
    }

    return 0;
}
