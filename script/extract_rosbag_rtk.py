#!/usr/bin/env python3

"""
Author:      Dr. Ing. Ahmad Kamal Nasir
Email:       dringakn@gmail.com
Date:        14 Feb 2023

Description:
    Command‐line utility to extract GPS (NavSatFix) messages from a ROS bag
    and write them into a normalized PCD (Point Cloud Data) file.

Features:
  • Reads all NavSatFix messages from the specified bag on the given topic  
  • Converts latitude/longitude into UTM coordinates via the `utm` library  
  • Translates all points so that the first GPS fix becomes the origin (0,0,0)  
  • Saves the resulting point cloud in binary PCD format using `pcl`  
  • Prints summary of how many points were written  

Usage:
    ./extract_rosbag_rtx.py <bag_file> [--gps_topic <topic>]

Arguments:
    bag_file      Path to the input ROS bag (e.g. `/path/to/data.bag`)
    --gps_topic   Name of the GPS topic inside the bag (default: `/dji_osdk_ros/rtk_position`)

Example:
    ./extract_rosbag_rtx.py \
        /home/ahmad/records_2022-11-29-13-03-50.bag \
        --gps_topic /dji_osdk_ros/rtk_position

Dependencies:
    • Python packages: pcl, numpy, utm, argparse, rosbag  
    • ROS sensor_msgs (NavSatFix)  
"""

import os # filename, extension extraction
import pcl # Pointcloud
import numpy as np
import utm
import argparse
import rosbag
from sensor_msgs.msg import NavSatFix


def main():

    parser = argparse.ArgumentParser(description="Extract gps messages from a rosbag on specified topic into a pcd file.")

    parser.add_argument("bag_file", help="Input ROS bag. (e.g. /path/to/filename.bag)")

    parser.add_argument("--gps_topic", help="GPS topic.", default="/dji_osdk_ros/rtk_position")

    args = parser.parse_args()

    output_file, file_extension = os.path.splitext(os.path.basename(args.bag_file))
    output_path = os.path.dirname(args.bag_file)
    output_file = f"{output_path}/{output_file}_rtk.pcd"

    print(f"Extract gps from {args.bag_file} on topic {args.gps_topic} into {output_file}")

    bag = rosbag.Bag(args.bag_file, "r")

    count = 0
    points = []
    init = False
    x0, y0, z0 = 0, 0, 0
    for topic, msg, t in bag.read_messages(topics=[args.gps_topic]):
        x, y, zone, _ = utm.from_latlon(msg.latitude, msg.longitude)
        z = msg.altitude
        if not init:
            x0, y0, z0 = x, y, z
            init = True
        points.append([x-x0, y-y0, z-z0])
        count += 1

    try:
        points = np.array(points, dtype = np.float32)
        cloud = pcl.PointCloud()
        cloud.from_array(points)
        pcl.save(cloud, output_file, format="pcd", binary=True)
        print(f"Created PCD [{count}]: {output_file}")
    except Exception as ex:
        print(f"Error creating {output_file}")
        print(f"Error: {ex}")        
    bag.close()

    return


if __name__ == '__main__':
    main()
