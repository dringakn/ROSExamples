#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    A simple PyQt5 GUI application that fetches a map tile image from OpenStreetMap
    via the Overpass API and displays it in a resizable window.

Features:
  • PyQt5-based main window with a QLabel for image display.
  • Uses the `Overpass` helper (from our `osm` module) to download a map tile via OSM/Overpass.
  • Saves the fetched tile as "image.png" in the working directory.
  • Automatically loads and displays the PNG, scaling to fit the window.
  • Adjusts window size to match the image dimensions on startup.

Dependencies:
  • Python 3.x
  • PyQt5
      Install with: pip install PyQt5 richdem elevation alphashape
  • `osm` module providing `Overpass` with a `.getImageTile()` method.
      (Ensure your PYTHONPATH includes the directory containing `osm.py`.)

Usage:
    $ chmod +x show_image_qt.py
    $ ./show_image_qt.py

    Or:
    $ python3 show_image_qt.py

You can extend `Overpass.getImageTile()` to accept bounding‐box, zoom level,
or style parameters as needed—just modify the call in `MainWindow.__init__`.
"""

from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from osm import Overpass 
import sys

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.label = QtWidgets.QLabel()
        self.setCentralWidget(self.label)
        self.label.setScaledContents(True)

        osm = Overpass()
        img = osm.getImageTile()
        img.save("image.png", "PNG")

        self.label.setPixmap(QtGui.QPixmap('image.png'))
        self.label.adjustSize()

        

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
