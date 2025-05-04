#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    Extract and downsample images from a ROS bag file for a specified image topic.

Features:
    - Reads sensor_msgs/Image messages from a ROS bag.
    - Converts each Image to OpenCV BGR8 format via cv_bridge.
    - Saves every N‑th frame as a zero‑padded PNG (frame000001.png, frame000002.png, …).
    - Optionally invokes ffmpeg to assemble saved frames into an MP4 video.
    - Minimal dependencies and POSIX‑style CLI.

Usage:
    ./extract_rosbag_images_downsampled.py <bag_file> <output_dir> <image_topic> <save_every_n>

Positional arguments:
    bag_file      Path to the input ROS bag (e.g. `data.bag`).
    output_dir    Directory into which frames will be written.
    image_topic   ROS image topic (e.g. `/cam0/image_raw`).
    save_every_n  Integer N: save one frame every N messages (use 1 for every frame).

Example:
    ./extract_rosbag_images_downsampled.py mydata.bag ./frames /cam0/image_raw 5

Dependencies:
    • ROS packages: rosbag, cv_bridge, sensor_msgs.msg.Image  
    • OpenCV Python bindings (cv2)  
    • ffmpeg (for optional video encoding)  
    • mjpegtools (optional, for alternative encodings)

Notes:
    - Ensure `output_dir` exists (e.g. `mkdir -p ./frames`) before running.  
    - To build a video after extraction:
        ffmpeg -framerate 25 -i frame%06d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4  
    - Install required tools:
        sudo apt update && sudo apt install ffmpeg mjpegtools
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
