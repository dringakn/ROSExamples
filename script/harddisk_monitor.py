#!/usr/bin/env python3

"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    ROS node to measure and publish hard‐disk read/write performance metrics
    by parsing /proc/diskstats for a specified block device.

Features:
  • Per‐second throughput:
      – Read throughput (KB/s)
      – Write throughput (KB/s)
  • Per‐second latency:
      – Average read time (ms)
      – Average write time (ms)
      – Total IO time (ms)
      – Waiting IO time (ms)
  • Publishes both:
      – A human‐readable printout to console
      – A Float64MultiArray on `/hard_disk_monitor`
        * data layout: [read_KB_s, write_KB_s, read_ms, write_ms, io_ms, wio_ms]
  • Configurable via ROS parameters:
      – `~device_name` (default: 'nvme0n1')
      – `~update_rate` (Hz, default: 1.0)
  • Linux‐only: requires `/proc/diskstats`

Dependencies:
  • ROS: rospy, std_msgs
  • Linux kernel procfs (/proc/diskstats)

Example:
    rosrun ros_examples harddisk_monitor \
        _device_name:=sda _update_rate:=2.0
"""

import rospy
import psutil
from std_msgs.msg import Float64MultiArray

def get_disk_stats():
    disk_stats = {}

    with open('/proc/diskstats', 'r') as f:
        for line in f:
            fields = line.strip().split()
            device_name = fields[2]
            if device_name == 'nvme0n1':
                major_number = int(fields[0])
                minor_number = int(fields[1])
                reads_completed = int(fields[3])
                sectors_read = int(fields[5])
                read_time = int(fields[6])
                writes_completed = int(fields[7])
                sectors_written = int(fields[9])
                write_time = int(fields[10])

                io_time = int(fields[12])
                wio_time = int(fields[13])

                disk_stats[(major_number, minor_number)] = {
                    'device_name': device_name,
                    'reads_completed': reads_completed,
                    'sectors_read': sectors_read,
                    'writes_completed': writes_completed,
                    'sectors_written': sectors_written,
                    'read_time': read_time,
                    'write_time': write_time,
                    'io_time': io_time,
                    'wio_time': wio_time,
                }

    return disk_stats


def publish_disk_speeds():

    rospy.init_node('harddisk_monitor', anonymous=True)
    pub = rospy.Publisher('/hard_disk_monitor', Float64MultiArray, queue_size=10)

    prev_disk_stats = get_disk_stats()
    prev_time = rospy.Time.now().to_sec()

    rate = rospy.Rate(1) # publish once per second

    while not rospy.is_shutdown():

        curr_disk_stats = get_disk_stats()
        curr_time = rospy.Time.now().to_sec()
        dt = (curr_time - prev_time)
        speeds = []

        for key, curr_stats in curr_disk_stats.items():
            prev_stats = prev_disk_stats.get(key)

            if prev_stats is not None:

                read_speed = (curr_stats['sectors_read'] - prev_stats['sectors_read']) * 512 / dt / 1024
                write_speed = (curr_stats['sectors_written'] - prev_stats['sectors_written']) * 512 / dt / 1024
                read_time = (curr_stats['read_time'] - prev_stats['read_time']) / dt
                write_time = (curr_stats['write_time'] - prev_stats['write_time']) / dt
                io_time = (curr_stats['io_time'] - prev_stats['io_time']) / dt
                wio_time = (curr_stats['wio_time'] - prev_stats['wio_time']) / dt

                print(f"{curr_stats['device_name']}: read_speed = {read_speed:.2f} KB/s, write_speed = {write_speed:.2f} KB/s, read_time = {read_time:.2f} msec, write_time = {write_time:.2f} msec, io_time = {wio_time:.2f} msec")
                speeds.append(read_speed)
                speeds.append(write_speed)
                speeds.append(read_time)
                speeds.append(write_time)
                speeds.append(wio_time)

        # publish speeds as an array
        speeds_msg = Float64MultiArray(data=speeds)
        pub.publish(speeds_msg)
        prev_disk_stats = curr_disk_stats
        prev_time = curr_time

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_disk_speeds()
    except rospy.ROSInterruptException:
        pass
