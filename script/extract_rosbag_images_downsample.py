#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Description: Extract images from a rosbag on specified topic to the output folder (Downsapmled version).
Usuage: ./extract_rosbag_images_downsampled.py bag_files.bag ./output/images/folder/path topic_name save_every_n_image
Example: ./extract_rosbag_images_downsampled.py bag_files.bag ./output /cam0 1
Notes: 
    ffmpeg -framerate 25 -i frame%06.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4
    sudo apt install mjpegtools
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import subprocess

def main():
    """
    Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("save_frames", help="Save frame every multiple of n.", type=int)

    args = parser.parse_args()

    print(f"Extract images from {args.bag_file} on topic {args.image_topic} into {args.output_dir} [save frame multiple of {args.save_frames}]")

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    frame_number = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # passthrough, bgr8
        if args.save_frames<=1:
            cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % frame_number), cv_img)
            print(f"Wrote image {frame_number}")
            frame_number +=1
        elif count%args.save_frames==0:
            cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % frame_number), cv_img)
            print(f"Wrote image {frame_number}")
            frame_number +=1
        count += 1

    bag.close()

    print(subprocess.run(['ffmpeg', '-framerate 25 -i frame%06.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4'], stdout=subprocess.PIPE))
    return

if __name__ == '__main__':
    main()
