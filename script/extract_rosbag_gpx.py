#!/usr/bin/env python3

"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Description: Extract gps messages from a rosbag on specified topic into a gpx file.
Example: ./extract_rosbag_gpx.py bagfile.bag gps_track.gpx /gps_position
"""

import gpxpy  # pip install pgxpy
import gpxpy.gpx
import argparse
import rosbag
from sensor_msgs.msg import NavSatFix


def main():

    parser = argparse.ArgumentParser(
        description="Extract gps messages from a rosbag on specified topic into a gpx file.")

    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument(
        "output_file", help="Output gpx file name with path (e.g ./filename.gpx).")
    parser.add_argument("gps_topic", help="GPS topic.")

    args = parser.parse_args()

    print(
        f"Extract gps from {args.bag_file} on topic {args.gps_topic} into {args.output_file}")

    gpx = gpxpy.gpx.GPX()
    gpx.name = "DJIM300RTK"
    gpx.description = "DJI M300 RTK"

    bag = rosbag.Bag(args.bag_file, "r")

    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.gps_topic]):
        gpx_wps = gpxpy.gpx.GPXWaypoint()
        gpx_wps.latitude = msg.latitude
        gpx_wps.longitude = msg.longitude
        gpx_wps.elevation = msg.altitude
        # gpx_wps.time = msg.header.stamp # datetime.strptime(timest, '%m/%d/%Y %H:%M:%S').isoformat()
        gpx_wps.symbol = f"P{count}"
        gpx_wps.name = f"P{count}"
        gpx_wps.description = f"P{count}"
        gpx.waypoints.append(gpx_wps)
        count += 1

    with open(args.output_file, "w") as f:
        f.write(gpx.to_xml())
        print(f"Created GPX: {args.output_file}")

    bag.close()

    return


if __name__ == '__main__':
    main()
