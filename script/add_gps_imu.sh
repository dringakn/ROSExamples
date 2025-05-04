#!/bin/bash
# Author:        Dr. Ing. Ahmad Kamal Nasir
# Email:         dringakn@gmail.com
#
# Description:
#   Embed GPS and IMU metadata into an image file using ExifTool.
#
# Features:
#   • GPS tags: latitude, longitude, altitude
#   • IMU tags: pitch, roll, yaw
#   • Automatically writes to “output.png” in same directory
#   • Cleans up any existing output before writing
#
# Requirements:
#   • exiftool (install via: sudo apt update && sudo apt install libimage-exiftool-perl)
#   • bash shell
#
# Usage:
#   ./add_metadata.sh <input_image>
#
# Example:
#   ./add_metadata.sh /home/user/photo.jpg
#
# Notes:
#   • Coordinates must be in “DD.dddd N/S” and “DDD.dddd E/W” format
#   • Altitude in meters (“10 m”)
#   • IMU angles in degrees
#
# Exit codes:
#   0 – success
#   1 – missing input file
#   2 – exiftool error
#

# Set the GPS coordinates
lat="40.7128 N"
lon="74.0060 W"
alt="10 m"

# Set the IMU data
pitch="1.5"
roll="0.5"
yaw="2.0"

# Set the input and output filenames
input_file="$1"
input_path=$(dirname "$input_file")
output_file="$input_path/output.png"

# Remove if output already exists
if [ -f "$output_file" ]; then
    rm "$output_file"
fi

# Use ExifTool to add the GPS and IMU information to the image
exiftool \
    -GPSLatitude="$lat" \
    -GPSLongitude="$lon" \
    -GPSAltitude="$alt" \
    -PitchAngle="$pitch" \
    -RollAngle="$roll" \
    -YawAngle="$yaw" \
    "$input_file" \
    -o "$output_file"
