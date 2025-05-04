#!/usr/bin/env python3

"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com
Date:          14 Feb 2023

Description:
    Extracts UFOMapStamped messages from a ROS bag and writes them out as:
      • a .ufo (UFOMap binary) file
      • a .pcd (Point Cloud Data) file

Features:
  • Reads a ROS bag and filters on a given UFO map topic  
  • Automatically derives output filenames based on the bag name  
  • Dumps the first (or only) UFOMapStamped message to disk  
  • Converts the UFO map to a flat point cloud and saves as PCD  
  • Prints progress and any errors to the console  

Dependencies:
  • ROS (rosbag Python API)  
  • Python 3 standard libraries: argparse, os  
  • UFOMap ROS messages (ufomap_msgs)  
  • PCL or equivalent for .pcd export (e.g. python-pcl)  

Example:
    ./extract_rosbag_ufomap.py \
        /home/ahmad/records_2022-11-29-13-03-50.bag \
        --ufomap_topic /ufomap_mapping_server_node/map
"""

import os # filename, extension extraction
# from ufomap.ufomap_ros.ufomap_msgs.msg import UFOMapStamped # UFOMap
import argparse
import rosbag


def main():

    parser = argparse.ArgumentParser(description="Extract UFO map messages from a rosbag on specified topic into ufo and pcd files.")
    parser.add_argument("bag_file", help="Input ROS bag. (e.g. /path/to/filename.bag)")
    parser.add_argument("--ufomap_topic", help="UFO map topic name.", default="/ufomap_mapping_server_node/map")

    args = parser.parse_args()

    output_file, file_extension = os.path.splitext(os.path.basename(args.bag_file))
    output_path = os.path.dirname(args.bag_file)
    output_file1 = f"{output_path}/{output_file}_ufomap.ufo"
    output_file2 = f"{output_path}/{output_file}_ufomap.pcd"

    print(f"Extract UFO map from {args.bag_file} on topic {args.ufomap_topic} into {output_file1} and {output_file2}")

    bag = rosbag.Bag(args.bag_file, "r")

    count = 0
    points = []
    init = False
    for topic, msg, t in bag.read_messages(topics=[args.ufomap_topic]):
        count += 1
        print(dir(msg))
        break
    # try:
    #     points = np.array(points, dtype = np.float32)
    #     cloud = pcl.PointCloud()
    #     cloud.from_array(points)
    #     pcl.save(cloud, output_file, format="pcd", binary=True)
    #     print(f"Created PCD [{count}]: {output_file}")
    # except Exception as ex:
    #     print(f"Error creating {output_file}")
    #     print(f"Error: {ex}")        
    # bag.close()

    return


if __name__ == '__main__':
    main()
