#!/usr/bin/env python3

"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Description: Extract gps messages from a rosbag on specified topic into a gpx file.
Example: ./extract_rosbag_gpx.py /home/ahmad/records_2022-11-29-13-03-50.bag gps_track.gpx /dji_osdk_ros/rtk_position
"""

import os # filename, extension extraction
import gpxpy  # pip install pgxpy
import gpxpy.gpx
import argparse
import rosbag
from sensor_msgs.msg import NavSatFix


def main():

    parser = argparse.ArgumentParser(description="Extract gps messages from a rosbag on specified topic into a gpx file.")

    parser.add_argument("bag_file", help="Input ROS bag. (e.g. /path/to/filename.bag)")

    # parser.add_argument(
    #     "output_file", help="Output gpx file name with path (e.g ./filename.gpx).")

    parser.add_argument("--gps_topic", help="GPS topic.", default="/dji_osdk_ros/rtk_position")

    args = parser.parse_args()

    output_file, file_extension = os.path.splitext(os.path.basename(args.bag_file))
    output_path = os.path.dirname(args.bag_file)
    output_file = f"{output_path}/{output_file}.gpx"

    print(f"Extract gps from {args.bag_file} on topic {args.gps_topic} into {output_file}")

    gpx = gpxpy.gpx.GPX()
    gpx.name = "DJIM300RTK"
    gpx.description = "DJI M300 RTK"

    bag = rosbag.Bag(args.bag_file, "r")
    track = gpxpy.gpx.GPXTrack(name=f"{output_file}", description="RTK position")
    segment = gpxpy.gpx.GPXTrackSegment()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.gps_topic]):
        gpx_wps = gpxpy.gpx.GPXTrackPoint()
        gpx_wps.latitude = msg.latitude
        gpx_wps.longitude = msg.longitude
        gpx_wps.elevation = msg.altitude
        # gpx_wps.time = msg.header.stamp # datetime.strptime(timest, '%m/%d/%Y %H:%M:%S').isoformat()
        gpx_wps.symbol = f"P{count}"
        gpx_wps.name = f"P{count}"
        gpx_wps.description = f"P{count}"
        segment.points.append(gpx_wps)
        count += 1

    track.segments.append(segment)
    gpx.tracks.append(track)
    with open(output_file, "w") as f:
        f.write(gpx.to_xml())
        print(f"Created GPX: {output_file}")

    bag.close()

    return


if __name__ == '__main__':
    main()
