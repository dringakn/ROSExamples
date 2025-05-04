#!/usr/bin/env python3

"""
Author:      Dr. Ing. Ahmad Kamal Nasir
Email:       dringakn@gmail.com
Created:     14 Feb 2023
Modified:    04 May 2025

Description:
    Extract CPU and memory usage messages from a ROS bag on specified
    /cpu_monitor/ topics into a Pandas DataFrame, save as a pickle file,
    and generate summary bar charts.

Features:
  • Reads all topics under a given prefix (default: /cpu_monitor/).
  • Builds a time-indexed DataFrame of resource usage by topic.
  • Saves raw time series data as a .pickle for later analysis.
  • Computes average CPU and memory usage and plots horizontal bar charts.
  • Outputs two PNG files: one for CPU, one for memory.

Parameters:
  bag_file                 Path to input ROS bag (e.g. /path/to/file.bag)
  --cpu_monitor_topic_prefix
                            Prefix for cpu_monitor topics (default: /cpu_monitor/)

Outputs:
  {basename}.pickle        Pickled Pandas DataFrame of raw metrics
  {basename}_CPU.png       Bar chart of average CPU usage per topic
  {basename}_MEM.png       Bar chart of average memory usage per topic

Dependencies:
  • ROS Python API: rosbag, std_msgs
  • pandas, numpy, matplotlib
  • Python ≥ 3.6

Usage:
    ./extract_rosbag_gresource_monitor.py <bag_file> [--cpu_monitor_topic_prefix PREFIX]

Example:
    ./extract_rosbag_gresource_monitor.py \
        ~/data/my_run.bag \
        --cpu_monitor_topic_prefix /cpu_monitor/
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
