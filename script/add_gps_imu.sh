#!/bin/bash

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
