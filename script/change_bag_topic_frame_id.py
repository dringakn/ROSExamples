#!/usr/bin/env python3

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
