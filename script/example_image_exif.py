#!/usr/bin/env python3

# For image exif information view: exiftool image.jpg
# sudo apt-get install exiftool
# exiftool -gpslatitude="37 deg 48' 35.58\" N" -gpslongitude="122 deg 27' 22.20\" W" -gpslatituderef="North" -gpslongituderef="West" image.jpg

# from PIL import Image
# from PIL.ExifTags import TAGS, GPSTAGS

img = "~/Deals3D_2023-02-24-17-46-02_images/frame001039.png"

# def add_gps_info_to_image(image_path, lat, lng):
#     image = Image.open(image_path)

#     # Get exif data
#     exif_data = {}
#     try:
#         for tag_id, value in image._getexif().items():
#             tag = TAGS.get(tag_id, tag_id)
#             exif_data[tag] = value
#     except AttributeError:
#         pass

#     print(exif_data)
#     print(GPSTAGS)
    
#     # Set GPS data
#     exif_data['GPSInfo'] = {
#         GPSTAGS['GPSLatitudeRef']: 'N' if lat >= 0 else 'S',
#         GPSTAGS['GPSLatitude']: get_decimal_from_dms(abs(lat)),
#         GPSTAGS['GPSLongitudeRef']: 'E' if lng >= 0 else 'W',
#         GPSTAGS['GPSLongitude']: get_decimal_from_dms(abs(lng))
#     }

#     # Save new exif data to image
#     image.save(image_path, exif=exif_data)

#     # Verify GPS data has been added
#     new_image = Image.open(image_path)
#     new_exif_data = {}
#     for tag_id, value in new_image._getexif().items():
#         tag = TAGS.get(tag_id, tag_id)
#         new_exif_data[tag] = value

#     if 'GPSInfo' not in new_exif_data:
#         return False

#     gps_info = new_exif_data['GPSInfo']
#     if GPSTAGS['GPSLatitude'] not in gps_info or GPSTAGS['GPSLongitude'] not in gps_info:
#         return False

#     return True

# def get_decimal_from_dms(dms):
#     degrees = dms // 100
#     minutes = (dms - (degrees * 100)) / 60
#     seconds = (dms - (degrees * 100) - (minutes * 60)) / 3600
#     return degrees + minutes + seconds

import piexif
from PIL import Image

im = Image.open(img)

exif_dict = {
    piexif.ExifIFD.DateTimeOriginal: "2022:01:15 12:00:00",
    piexif.GPSIFD.GPSLatitudeRef: "N",
    piexif.GPSIFD.GPSLatitude: ((51, 1), (30, 1), (0, 1)),
    piexif.GPSIFD.GPSLongitudeRef: "W",
    piexif.GPSIFD.GPSLongitude: ((0, 1), (7, 1), (39, 1)),
    piexif.GPSIFD.GPSAltitudeRef: 0,
    piexif.GPSIFD.GPSAltitude: (1000, 1),
    piexif.ImageIFD.Make: "My Camera",
    piexif.ImageIFD.Model: "My Camera Model",
}

exif_bytes = piexif.dump(exif_dict)
im.save(img, "png", exif=exif_bytes)


