#!/usr/bin/env python3

"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com
Created:       23 Mar 2023
Modified:      04 May 2025
Description:
    Extracts `/rosout` messages from a ROS bag, serializes them to a pandas
    DataFrame (pickled), and generates time‑series plots of ERROR/WARN/INFO
    counts.

Features:
  • Flexible topic selection (default: `/rosout`)
  • Maps ROS log severity integers to names:
      – 1  = DEBUG
      – 2  = INFO
      – 4  = WARN
      – 8  = ERROR
      – 16 = FATAL
  • Builds a DataFrame with columns:
      – time (indexed, converted to datetime)
      – level (DEBUG/WARN/INFO/etc.)
      – node name
      – message text
      – source file, function, line number
      – original topics
  • Saves raw DataFrame as `<bagname>_rosout.pickle`
  • Generates and saves PNGs of per-node ERROR/WARN/INFO counts over 10 s windows
  • Light dependency footprint: `rosbag`, `rosgraph_msgs`, `pandas`, `numpy`, `matplotlib`

Usage:
    ./extract_rosbag_rosout.py <input.bag> [--topic /rosout]

Example:
    ./extract_rosbag_rosout.py ~/logs/myrun.bag --topic /rosout

Notes:
  • Ensure `rosbag` and `rosgraph_msgs` are installed in your ROS environment.
  • Requires matplotlib for the PNG output.
"""

import os  # filename, extension extraction
import argparse
import rosbag
from rosgraph_msgs.msg import Log, TopicStatistics, Clock
import pandas as pd
import numpy as np
import time
import matplotlib.pyplot as plt

def main():

    parser = argparse.ArgumentParser(
        description="Extract rosout messages from a rosbag on specified topics into a pickle file.")

    parser.add_argument(
        "bag_file", help="Input ROS bag. (e.g. /path/to/filename.bag)")
    parser.add_argument("--topic", help="rosout topic.", default="/rosout")

    args = parser.parse_args()

    input_file, input_file_extension = os.path.splitext(
        os.path.basename(args.bag_file))
    output_path = os.path.dirname(args.bag_file)
    output_file = f"{output_path}/{input_file}_rosout.pickle"

    bag = rosbag.Bag(args.bag_file, "r")

    print(
        f"Extracting rosout data from {args.bag_file} on topic(s) {args.topic} into {output_file}")
    count = 0
    msgs = {'time': [], 'level': [], 'name': [], 'message': [],
            'file': [], 'function': [], 'line': [], 'topic': []}
    level_names = {1: 'DEBUG', 2: 'INFO', 4: 'WARN', 8: 'ERROR', 16: 'FATAL'}
    for topic, msg, t in bag.read_messages(topics=[args.topic]):
        # print(msg)
        # name = c.name.replace("/", "_").replace(" ", "_").replace(":","_")
        msgs['time'].append(t.to_sec())
        msgs['level'].append(level_names[msg.level])
        msgs['name'].append(msg.name.replace("/", ""))
        msgs['message'].append(msg.msg)
        msgs['file'].append(msg.file)
        msgs['function'].append(msg.function)
        msgs['line'].append(msg.line)
        msgs['topic'].append(', '.join(msg.topics))
        count += 1

    bag.close()

    print(f"Saving {count} messages as pickle file...")
    df = pd.DataFrame(msgs)
    # df = df.pivot(index='time', columns=['name'], values='message')
    df.set_index('time', inplace=True)
    df.index = pd.to_datetime(df.index.map(time.ctime))
    df = df.ffill().dropna()
    df.to_pickle(output_file)

    df['level'] = pd.Categorical(df.level)
    df1 = df.groupby(['name','level']).message.count().unstack().sort_values(['ERROR','WARN','INFO'], ascending=False)
    
    # df1.plot(kind='bar', stacked=True, color=['red','green','yellow'], xlabel='Nodes', ylabel='Count', legend=True, figsize=(16,10)).get_figure().savefig(f'{output_path}/{input_file}_VERBOSE_AGG.png',dpi=300, bbox_inches = "tight")
    # df.groupby(['name']).resample('1min').level.apply(lambda s: s.value_counts()).unstack().plot(kind='bar', stacked=True, color=['red','green','yellow'], xlabel='Nodes', ylabel='Count', legend=True, figsize=(16,10)).get_figure().savefig(f'{output_path}/{input_file}_VERBOSE.png',dpi=300, bbox_inches = "tight")

    df.groupby(['name']).resample('10s').level.apply(lambda s: (s=='ERROR').sum()).to_frame().reset_index().pivot(index='time', columns='name', values='level').plot(xlabel='Time', ylabel='Counts', subplots=True, figsize=(16,10))
    plt.savefig(f'{output_path}/{input_file}_ERROR.png',dpi=300, bbox_inches = "tight")

    df.groupby(['name']).resample('10s').level.apply(lambda s: (s=='WARN').sum()).to_frame().reset_index().pivot(index='time', columns='name', values='level').plot(xlabel='Time', ylabel='Counts', subplots=True, figsize=(16,10))
    plt.savefig(f'{output_path}/{input_file}_WARN.png',dpi=300, bbox_inches = "tight")

    df.groupby(['name']).resample('10s').level.apply(lambda s: (s=='INFO').sum()).to_frame().reset_index().pivot(index='time', columns='name', values='level').plot(xlabel='Time', ylabel='Counts', subplots=True, figsize=(16,10))
    plt.savefig(f'{output_path}/{input_file}_INFO.png',dpi=300, bbox_inches = "tight")

    return


if __name__ == '__main__':
    main()
