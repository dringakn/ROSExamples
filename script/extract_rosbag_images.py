#!/usr/bin/env python3

"""
Author:    Dr. Ing. Ahmad Kamal Nasir
Email:     dringakn@gmail.com

Description:
    Extracts all image frames from a ROS bag on a given topic and writes them
    as individual image files to a specified output directory.

Features:
  • Reads any sensor_msgs/Image stream via cv_bridge (passthrough encoding).
  • Automatically creates the output directory if it doesn’t exist.
  • Names files sequentially as frame000001.png, frame000002.png, …
  • Prints progress to the console.
  • Simple CLI with positional arguments.

Usage:
    rosrun <your_package> extract_images.py <bag_file> <output_dir> <image_topic>

Arguments:
    bag_file    Path to the input ROS bag file (.bag or .rosbag).
    output_dir  Directory where extracted frames will be saved.
    image_topic ROS topic name of the Image messages to extract.

Dependencies:
  • ROS (rosbag, sensor_msgs)
  • OpenCV (cv2)
  • cv_bridge
  • Python standard libs: argparse, os

Example:
    python extract_images.py recordings.bag /tmp/frames /camera/image_raw
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    """Extract a folder of images from a rosbag.
    """

    parser = argparse.ArgumentParser(
        description="Extract images from a ROS bag.")

    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    print(
        f"Extract images from {args.bag_file} on topic {args.image_topic} into {args.output_dir}")

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join(args.output_dir,
                    "frame%06i.png" % count), cv_img)
        print(f"Wrote image {count}")

        count += 1

    bag.close()

    return


if __name__ == '__main__':
    main()


@peteflorence
