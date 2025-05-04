#!/usr/bin/env python3
"""
Author:    Dr. Ing. Ahmad Kamal Nasir
Email:     dringakn@gmail.com
Date:      14 Feb 2023

Description:
    Adjust video brightness frame‑by‑frame using the VEVID GPU algorithm,
    outputting a side‑by‑side comparison of original vs. processed frames.

Features:
  • Per‑frame VEVID processing in “lite” mode for speed.
  • Configurable brightness (b) and gain (G) knobs.
  • Automatic device selection (CUDA if available, else CPU).
  • Concatenates original and VEVID output horizontally.
  • Writes result at 20 fps to “<input>_processed.mp4”.

Dependencies:
  • Python 3.x
  • numpy
  • torch
  • torchvision
  • imageio.v3
  • phycv (install via `pip install phycv`)

Usage:
    ./adjust_video_brightness.py <input_video.mp4>

Example:
    ./adjust_video_brightness.py ~/sample_video.mp4
"""


import os # filename, extension extraction
import argparse
import imageio.v3 as iio
import numpy as np
import torch
import torchvision
from phycv import VEVID_GPU # pip install phycv


def main():
   
    parser = argparse.ArgumentParser(description="Adjust brightness of a video file (20 seconds).")
    parser.add_argument("video_file", help="Input video file. (e.g. ~/sample_video.mp4)", default="~/sample_video.mp4")
    args = parser.parse_args()
    output_file, file_extension = os.path.splitext(os.path.basename(args.video_file))
    output_path = os.path.dirname(args.video_file)
    output_file = f"{output_path}/{output_file}_processed.mp4"

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # indicate the video to be processed
    vid = torchvision.io.read_video(args.video_file)
    print("Video loaded.")

    # get how many frames are in the video
    # create a empty array to store the VEViD output
    vid_frames = vid[0]
    length = vid_frames.shape[0]

    # indicate VEViD-lite parameters
    b = 0.5
    G = 0.6
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # run VEViD for each frame
    vevid = VEVID_GPU(device=device)
    concat_frames = []

    for i in range(length):
        frame = torch.permute(vid_frames[i], (2, 0, 1)) / 255.0
        frame = frame.to(device)
        vevid.load_img(img_array=frame)
        vevid.apply_kernel(b, G, lite=True)
        vevid_out_vid = vevid.vevid_output.cpu().numpy()

        raw_frame = vid_frames[i].numpy()
        vevid_frame = (np.transpose(vevid_out_vid, (1, 2, 0)) * 255).astype(np.uint8)
        concat_frame = np.concatenate((raw_frame, vevid_frame), 1)
        concat_frames.append(concat_frame)
        print(f"Frame-{i} of {length} processed")
            
    # create video from the processed frames
    print("Creating video ...")
    iio.imwrite(output_file, concat_frames, fps=20)


if __name__ == "__main__":
    main()