#!/usr/bin/env python3

"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Description: Extract images from a rosbag on specified topic to the output folder.
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
