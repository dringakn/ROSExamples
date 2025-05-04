#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    ROS node that monitors disk I/O rates of running rosbag record processes
    and publishes per-second read/write throughput.

Features:
  • Discovers processes whose name contains "rosbag" (or "record")
  • Uses psutil to read each process’s io_counters()
  • Computes read_bytes and write_bytes delta per second (KB/s)
  • Publishes Float64 messages on:
      – /io_read_rate
      – /io_write_rate
  • Runs at 1 Hz by default
  • Logs I/O counters to console for debugging

Dependencies:
  • Python package psutil (install via `pip install psutil`)
  • Access to /proc filesystem
  • ROS packages: rospy, std_msgs

Example:
    rosrun your_package io_monitor.py
"""

import psutil
import os
import rospy
from std_msgs.msg import Float64

def get_process():
    process_names = []

    for pid in os.listdir('/proc'):
        if pid.isdigit():
            with open(os.path.join('/proc', pid, 'comm'), 'r') as f:
                name = f.read().strip()
                
                if 'rosbag' in name:
                    process_names.append((name, pid))
                    
                    with open(os.path.join('/proc', pid, 'io'), 'r') as f1:
                        io_counts = name = f1.read().strip()
                        print(io_counts)

    return process_names

def publish_io_rates():
    pub_read = rospy.Publisher('io_read_rate', Float64, queue_size=10)
    pub_write = rospy.Publisher('io_write_rate', Float64, queue_size=10)
    rospy.init_node('io_monitor')

    rate = rospy.Rate(1) # 1 Hz
    prev_io_counters = None
        
    while not rospy.is_shutdown():
        for proc in psutil.process_iter(['name','pid']):
            if 'record' in str(proc.info['name']):
                io_counters = proc.io_counters() # psutil.disk_io_counters()

                print(f"{proc.info['name']} [{proc.info['pid']}]", io_counters)
                # print(get_process())
               
                if prev_io_counters == None:
                    prev_io_counters = io_counters
                    break
                
                read_rate = (io_counters.read_bytes - prev_io_counters.read_bytes) / 1024.0 # convert to KB/s
                write_rate = (io_counters.write_bytes - prev_io_counters.write_bytes) / 1024.0 # convert to KB/s
                pub_read.publish(read_rate)
                pub_write.publish(write_rate)
                prev_io_counters = io_counters
                break

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_io_rates()
    except rospy.ROSInterruptException:
        pass
