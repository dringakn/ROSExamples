#!/bin/bash

# Set input and output directories
input_dir="."
output_dir="${input_dir}/output/"
rmdir $output_dir
mkdir $output_dir

# Create colormap file
cat > "${output_dir}/colormap.txt" <<EOL
1.0 255 255 255
0.8 0 255 0
0.4 128 255 0
0.4 255 255 0
0.2 255 128 0
0.0 255 0 0
EOL

# Apply colormap and create movie for each image in input directory
for file in "${input_dir}"/*_INDEX_f32.tif; do
    filename=$(basename "${file}")
    # Apply colormap to image
    echo "${filename}"
    gdaldem color-relief "${file}" "${output_dir}/colormap.txt" "${output_dir}/color_${filename}" -alpha
    # Add processed image to list
    echo "file 'color_${filename}'" >> "${output_dir}/list.txt"
done

# Create movie from processed images
#ffmpeg -r 10 -f concat -safe 0 -i "${output_dir}/list.txt" -c:v libx264 -pix_fmt yuv420p "${output_dir}/ndvi_output.mp4"
ffmpeg -r 10 -f concat -safe 0 -i "${output_dir}/list.txt" -c:v libx264 "${output_dir}/ndvi_output.mp4"