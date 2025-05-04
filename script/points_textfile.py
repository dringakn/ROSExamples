#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Module:        PointsTextFile
Description:
    Simple utility for storing and retrieving lists of point coordinates
    to and from plain-text files. Each point is represented as a comma‑
    separated list of floats on its own line.

Features:
  • Read points:
      – Parses each line as a list of floats
      – Returns a List[List[float]]
  • Write points:
      – Takes a List[List[float]]
      – Serializes each inner list as comma‑separated floats
      – Overwrites (or creates) the target file
  • Flexible format:
      – Any dimensionality (2D, 3D, etc.) so long as each line has
        a consistent number of comma‑separated values
  • Error handling:
      – Raises FileNotFoundError if file is missing on read
      – Propagates IOError on write failure

Usage:
    from points_text_file import PointsTextFile

    # read existing points
    reader = PointsTextFile("my_points.txt")
    pts = reader.read_points_from_text_file()  # e.g. [[x1,y1,z1], [x2,y2,z2], ...]

    # write new points
    writer = PointsTextFile("out_points.txt")
    writer.write_points_to_text_file([[0.0, 1.2], [3.4, 5.6], [7.8, 9.0]])
"""

class PointsTextFile:
    
    def __init__(self, file_name):
        """Read or write list of points coordinates from/to a text file.

        Args:
            file_name (string): Filename with extension, e.g. pointsfile.txt
        """
        self.file_name = file_name
    
    def read_points_from_text_file(self):
        """Read the points coordinates from the text file and return as list of list.

        Returns:
            points (list): List of points coordinates
        """
        pts = []
        with open(self.file_name, 'r') as f:
            for line in f:
                pts.append([float(item) for item in line.rstrip().split(',')])
                
        return pts


    def write_points_to_text_file(self, pts):
        """Write the points coordinates to the text file.

        Args:
            pts (list): List of points to be saved.
        """
        with open(self.file_name, 'w') as f:
            for line in pts:
                f.write(f"{','.join([str(x) for x in line])}\n")