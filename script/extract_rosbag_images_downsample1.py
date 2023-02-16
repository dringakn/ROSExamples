#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Modified: 14 Feb 2023
Description: Extract images from a rosbag on specified topic to the output folder (Downsapmled version).
             Create also a video. 
Usuage: ./extract_rosbag_images_downsampled.py bag_files.bag topic_name save_every_n_image
Example: ./extract_rosbag_images_downsample.py ~/records_2022-12-05-13-30-08.bag /dji_fpv_camera_downsampled 1

Notes: 
    ffmpeg -framerate 25 -i frame%06d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4
    sudo apt install mjpegtools
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
