#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Author:      Dr. Ing. Ahmad Kamal Nasir
Email:       dringakn@gmail.com
Modified:    14 Feb 2023

Description:
    Extracts images from a ROS bag on a specified image topic, downsampling
    by saving every Nth frame, and then assembles them into a video.

Features:
  • Reads any ROS bag (.bag) file and extracts sensor_msgs/Image messages.
  • Downsampling – only saves every Nth frame as PNG, to reduce disk usage.
  • Automatic video creation via ffmpeg, using H.264 encoding and yuv420p pixel format.
  • Prints progress to console for both image extraction and video encoding.

Requirements:
  • ROS (noetic/melodic) with python3 support
  • cv_bridge, rosbag, rospy
  • OpenCV (cv2)
  • ffmpeg (for video encoding)
  • Permissions to read the bag file and write to its directory

Usage:
    ./extract_rosbag_images_downsampled.py <bag_file> [--image_topic TOPIC] [--save_frames N]

Arguments:
  bag_file         Path to input .bag file
  --image_topic    ROS image topic to extract (default: /dji_fpv_camera_downsampled)
  --save_frames    Save every Nth frame (default: 1, i.e. every frame)

Example:
    ./extract_rosbag_images_downsampled.py \
      ~/records_2022-12-05-13-30-08.bag \
      --image_topic /dji_fpv_camera_downsampled \
      --save_frames 5

Notes:
  • After extraction, run:
        ffmpeg -framerate 25 -i frame%06d.png \
               -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4
  • Install dependencies:
        sudo apt update && sudo apt install ros-noetic-cv-bridge \
            ros-noetic-rosbag python3-opencv ffmpeg
"""

import os
import argparse
import subprocess

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """
    Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("--image_topic", help="Image topic name.", default="/dji_fpv_camera_downsampled")
    parser.add_argument("--save_frames", help="Extract frame multiple of save_frames.", type=int, default=1)

    args = parser.parse_args()

    input_file, file_extension = os.path.splitext(os.path.basename(args.bag_file))
    output_path = f"{os.path.dirname(args.bag_file)}/{input_file}_images/"
    if not os.path.exists(output_path):
        os.makedirs(output_path)
        
    print(f"Extract images from {args.bag_file} on topic {args.image_topic} into {output_path} [every {args.save_frames} frame]")

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    frame_number = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # passthrough, bgr8
        output_file = os.path.join(output_path, "frame%06i.png" % frame_number)
        if args.save_frames<=1:
            cv2.imwrite(output_file, cv_img)
            print(f"Extracted image :{output_file}")
            frame_number +=1
        elif count%args.save_frames==0:
            cv2.imwrite(output_file, cv_img)
            print(f"Extracted image: {output_file}")
            frame_number +=1
        count += 1

    bag.close()

    command = f"ffmpeg -framerate 25 -i {output_path}frame%06d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p {output_path}{input_file}.mp4"
    result = subprocess.run(command, stdout=subprocess.PIPE, shell=True)
    print(result.stdout.decode("utf-8"))
    
    return

if __name__ == '__main__':
    main()
