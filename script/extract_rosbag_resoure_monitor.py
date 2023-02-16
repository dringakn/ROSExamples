#!/usr/bin/env python3

"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Created: 14 Feb 2023
Modified: 14 Feb 2023
Description: Extract cpu_monitor messages from a rosbag on specified topics into a pickle file.
Example: ./extract_rosbag_gresource_monitor.py ~/filename.bag cpu_monitor_topics_prefix
"""

import os # filename, extension extraction
import argparse
import rosbag
from std_msgs.msg import Float32, UInt64
import pandas as pd
import numpy as np

def main():

    parser = argparse.ArgumentParser(description="Extract cpu_monitor messages from a rosbag on specified topics into a pickle file.")

    parser.add_argument("bag_file", help="Input ROS bag. (e.g. /path/to/filename.bag)")
    parser.add_argument("--cpu_monitor_topic_prefix", help="cpu_monitor topics prefix.", default="/cpu_monitor/")

    args = parser.parse_args()

    input_file, input_file_extension = os.path.splitext(os.path.basename(args.bag_file))
    output_path = os.path.dirname(args.bag_file)
    output_file = f"{output_path}/{input_file}.pickle"

    bag = rosbag.Bag(args.bag_file, "r")

    print(f"Extracting cpu_monitor data from {args.bag_file} on topic(s) {args.cpu_monitor_topic_prefix} into {output_file}")
    count = 0
    msgs = {'time': [], 'topic': [], 'data': []}
    skip_n = len(args.cpu_monitor_topic_prefix)
    for topic, msg, t in bag.read_messages():
        if topic.startswith(args.cpu_monitor_topic_prefix):
            topic = topic[skip_n:].replace("/", ", ")
            msgs['time'].append(t.to_sec())
            msgs['topic'].append(topic)
            msgs['data'].append(msg.data)
            count += 1
    
    bag.close()
    
    print(f"Saving {count} messages as pickle file...")
    df = pd.DataFrame(msgs)
    df = df.pivot_table(index='time', columns='topic', values='data')
    df.to_pickle(output_file)
    df.filter(regex=', cpu').mean().plot(kind='barh').get_figure().savefig(f'{output_path}/{input_file}_CPU.png',dpi=300, bbox_inches = "tight")
    df.filter(regex=', mem').mean().plot(kind='barh').get_figure().savefig(f'{output_path}/{input_file}_MEM.png',dpi=300, bbox_inches = "tight")

    return


if __name__ == '__main__':
    main()
