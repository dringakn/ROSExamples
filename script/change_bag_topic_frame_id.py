#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    This script renames the frame_id in the header of every message on a specified topic
    within a ROS bag file, writing the modified messages to a new bag.

Features:
  • Reads an existing bag file (`input_file`)
  • Iterates through all messages, matching on a target `topic_name`
  • For messages on that topic, overwrites `msg.header.frame_id` with `frame_name`
  • Preserves original timestamps and other topics/messages untouched
  • Writes everything into a new bag file (`output_file`)

Usage:
    # Edit the four parameters below or override them via ROS params:
    input_file   – path to the source bag (must exist)
    output_file  – path for the modified bag (will be created/overwritten)
    topic_name   – name of the topic whose frame_id you wish to replace
    frame_name   – new frame_id string to assign

Dependencies:
  • rospy
  • rosbag (ROS package)
"""

import rospy
import rosbag

# Author: Dr. Ing. Ahmad Kamal Nasir
# Email: dringakn@gmail.com
# Description: Rename the frame_id of the topic in the bag file.

# Change as required
input_file = "/home/ahmad/catkin_ws/src/package/map/input.bag"
output_file = "/home/ahmad/catkin_ws/src/package/map/output.bag"
topic_name = "/topic_name"
frame_name = "frame_id"

with rosbag.Bag(output_file, 'w') as outbag:
    print('Processing ' + input_file)
    for topic, msg, t in rosbag.Bag(input_file).read_messages():
        # Check for the topic name if found then replace the frame_id in the message header,
        # otherwise, save the message as it is.
        if topic == topic_name:
            msg.header.frame_id = frame_name
            outbag.write(topic, msg, t)  # msg.header.stamp
        else:
            outbag.write(topic, msg, t)
    print('Modified Bag: ' + output_file)
