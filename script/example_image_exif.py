#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    Standalone script to embed EXIF metadata—including GPS coordinates,
    capture timestamp, altitude, and camera make/model—into PNG or JPEG images.

Features:
  • Set GPS tags:
      – GPSLatitudeRef, GPSLatitude
      – GPSLongitudeRef, GPSLongitude
      – GPSAltitudeRef, GPSAltitude
  • Set DateTimeOriginal
  • Set Image Make & Model tags
  • Uses piexif for metadata writing
  • Inspect or verify with exiftool

Usage:
    1) Adjust the `img` path and `exif_dict` entries below, or
    2) Extend into a CLI by wrapping `embed_exif()` with argparse.
    
Example:
    exiftool -gpslatitude="37 deg 48' 35.58\" N" \
             -gpslongitude="122 deg 27' 22.20\" W" \
             -gpslatituderef="N" \
             -gpslongituderef="W" \
             image.jpg

Dependencies:
    • piexif      (pip install piexif)
    • Pillow     (pip install Pillow)
    • exiftool   (sudo apt-get install exiftool)
"""

# For quick inspection:
#   exiftool image.jpg
#   sudo apt-get install exiftool

import piexif
from PIL import Image
import os

# --- Configuration --------------------------------------------------------

# Path to your image file (PNG or JPEG)
img = os.path.expanduser(
    "~/Deals3D_2023-02-24-17-46-02_images/frame001039.png"
)

# EXIF tags to embed
exif_dict = {
    # Capture timestamp
    piexif.ExifIFD.DateTimeOriginal: "2022:01:15 12:00:00",

    # GPS coordinates: degrees/minutes/seconds as rational tuples
    piexif.GPSIFD.GPSLatitudeRef:  "N",
    piexif.GPSIFD.GPSLatitude:     ((51,1), (30,1), (0,1)),
    piexif.GPSIFD.GPSLongitudeRef: "W",
    piexif.GPSIFD.GPSLongitude:    ((0,1),  (7,1),  (39,1)),
    piexif.GPSIFD.GPSAltitudeRef:  0,
    piexif.GPSIFD.GPSAltitude:     (1000,1),

    # Camera info
    piexif.ImageIFD.Make:  "My Camera",
    piexif.ImageIFD.Model: "My Camera Model",
}

# --- Core Functionality ---------------------------------------------------

def embed_exif(image_path: str, exif_data: dict) -> None:
    """
    Open the image at `image_path`, embed the EXIF tags from `exif_data`,
    and overwrite the file in place.
    """
    img = Image.open(image_path)
    exif_bytes = piexif.dump(exif_data)
    img.save(image_path, exif=exif_bytes)
    print(f"[OK] Embedded EXIF into: {image_path}")


# --- Entry Point ----------------------------------------------------------

if __name__ == "__main__":
    if not os.path.isfile(img):
        print(f"[ERROR] File not found: {img}")
    else:
        embed_exif(img, exif_dict)