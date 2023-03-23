#!/usr/bin/env python3

"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Created: 22 Mar 2023
Modified: 14 Mar 2023
Description: Extract libsensors_monitor messages from a rosbag on specified topics into a pickle file.
Example: ./extract_rosbag_sensors.py ~/filename.bag cpu_monitor_topics
Notes: Possible level of operations: OK=0, WARN=1, ERROR=2, STALE=3
"""

import os # filename, extension extraction
import argparse
import rosbag
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pandas as pd
import numpy as np
import time

def main():

    parser = argparse.ArgumentParser(description="Extract libsensors_monitor messages from a rosbag on specified topics into a pickle file.")

    parser.add_argument("bag_file", help="Input ROS bag. (e.g. /path/to/filename.bag)")
    parser.add_argument("--topic", help="libsensors_monitor topic.", default="/diagnostics")

    args = parser.parse_args()

    input_file, input_file_extension = os.path.splitext(os.path.basename(args.bag_file))
    output_path = os.path.dirname(args.bag_file)
    output_file = f"{output_path}/{input_file}_diagnostics.pickle"

    bag = rosbag.Bag(args.bag_file, "r")

    print(f"Extracting cpu_monitor data from {args.bag_file} on topic(s) {args.topic} into {output_file}")
    count = 0
    msgs = {'time': [], 'level': [], 'name': [], 'message': [], 'hardware_id': [], 'key': [], 'value': []}
    for topic, msg, t in bag.read_messages(topics=[args.topic]):
        for c in msg.status: # List of components
            for item in c.values:
                name = c.name.replace("/", "_").replace(" ", "_").replace(":","_")
                msgs['time'].append(t.to_sec())
                msgs['level'].append(c.level)
                msgs['name'].append(name)
                msgs['message'].append(c.message)
                msgs['hardware_id'].append(c.hardware_id)
                msgs['key'].append(item.key)
                msgs['value'].append(item.value)
            
            count += 1
    
    bag.close()
    
    print(f"Saving {count} messages as pickle file...")
    df = pd.DataFrame(msgs)
    df = df.pivot_table(index='time', columns=['name','key'], values='value')
    df.index = pd.to_datetime(df.index.map(time.ctime))
    df = df.ffill().dropna()
    df.to_pickle(output_file)
    
    # names = [col for col in df.columns for name in col if name in ['Fan Speed (RPM)', 'Temperature (C)']]
    names = [col for col in df.columns for name in col if name in ['Temperature (C)']]
    df[names].droplevel(axis=1, level=1).iloc[:,1:].rolling(100).mean().plot(legend=True, ylabel='Temperature (C)', xlabel='Time', grid='both', figsize=(16,10)).get_figure().savefig(f'{output_path}/{input_file}_TEMP.png',dpi=300, bbox_inches = "tight")

    return


if __name__ == '__main__':
    main()
