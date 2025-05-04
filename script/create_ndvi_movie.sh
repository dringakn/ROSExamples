#!/bin/bash
#
# Author:        Dr. Ing. Ahmad Kamal Nasir
# Email:         dringakn@gmail.com
#
# Description:
#   This script applies a color‑relief colormap to all files matching
#   `*_INDEX_f32.tif` in a given directory, writes the colorized TIFFs into
#   an `output/` subdirectory, and then concatenates them into a time‑lapse
#   MP4 movie.
#
# Features:
#   • Generates a customizable colormap definition (`colormap.txt`).
#   • Uses `gdaldem color-relief` to colorize each input TIFF.
#   • Builds an FFmpeg concat list (`list.txt`) for seamless video generation.
#   • Produces both the individual colorized TIFFs and a final `ndvi_output.mp4`.
#
# Requirements:
#   • GDAL (`gdaldem`) — install via your package manager, e.g.:
#       sudo apt update && sudo apt install gdal-bin
#   • FFmpeg — for video encoding:
#       sudo apt install ffmpeg
#
# Usage:
#   1. Place this script in the directory with your `*_INDEX_f32.tif` images.
#   2. Make executable: `chmod +x colorize_and_movie.sh`
#   3. Run: `./colorize_and_movie.sh`
#   4. Find outputs under `./output/` and the video `ndvi_output.mp4`.
#
# Customization:
#   • Change `input_dir` to point elsewhere (default: current directory).
#   • Tweak `colormap.txt` values or name.
#   • Uncomment or adjust the `ffmpeg` options (e.g. frame rate, codec).
#

# -- Configuration ----------------------------------------------------------
input_dir="."
output_dir="${input_dir}/output"

# Remove old output and recreate directory
rm -rf "${output_dir}"
mkdir -p "${output_dir}"

# -- Build colormap file ----------------------------------------------------
cat > "${output_dir}/colormap.txt" <<EOL
1.0 255 255 255
0.8 0   255   0
0.4 128 255   0
0.4 255 255   0
0.2 255 128   0
0.0 255   0   0
EOL

# -- Apply colormap & assemble file list -----------------------------------
# Clear any existing list
: > "${output_dir}/list.txt"

for file in "${input_dir}"/*_INDEX_f32.tif; do
    filename=$(basename "${file}")
    echo "Processing ${filename}…"
    # Colorize with alpha channel
    gdaldem color-relief \
        "${file}" \
        "${output_dir}/colormap.txt" \
        "${output_dir}/color_${filename}" \
        -alpha
    # Append to ffmpeg concat list
    echo "file 'color_${filename}'" >> "${output_dir}/list.txt"
done

# -- Create time‑lapse movie -----------------------------------------------
ffmpeg -r 10 -f concat -safe 0 \
    -i "${output_dir}/list.txt" \
    -c:v libx264 \
    -pix_fmt yuv420p \
    "${output_dir}/ndvi_output.mp4"

echo "Done! Colorized images and movie are in ${output_dir}/"
