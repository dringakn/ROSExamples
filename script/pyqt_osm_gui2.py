#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com


Description:
    A minimal PyQt5 application that fetches and displays a stitched OpenStreetMap
    tile image for a given latitude, longitude, and zoom level.

Features:
  • GUI fields for latitude, longitude and zoom level input
  • “Fetch Map” button to download and assemble OSM tiles around the specified point
  • Automatic conversion from latitude/longitude to OSM tile coordinates
  • Displays the resulting map image in the window
  • Easy to extend to other tile providers or add overlays

Dependencies:
  • PyQt5      (pip install pyqt5)
  • requests   (pip install requests)
  • Pillow     (pip install pillow)
  • Python 3.6+
    Install PyQt or PySide using: pip install pyqt5

Usage:
    $ python3 map_viewer.py
    Enter your coordinates, click “Fetch Map”, and see the map.

Example:
    rosrun my_package map_viewer.py
"""

import sys, math
import requests
from PyQt5 import QtWidgets, QtGui, QtCore
from PIL import Image  # For OSM image tile

class MapWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
    def initUI(self):
        self.latitude_edit = QtWidgets.QLineEdit()
        self.longitude_edit = QtWidgets.QLineEdit()
        self.zoom_edit = QtWidgets.QLineEdit()
        self.map_label = QtWidgets.QLabel()
        self.fetch_button = QtWidgets.QPushButton("Fetch Map")
        self.fetch_button.clicked.connect(self.fetchMap)
        
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(QtWidgets.QLabel("Latitude:"))
        layout.addWidget(self.latitude_edit)
        layout.addWidget(QtWidgets.QLabel("Longitude:"))
        layout.addWidget(self.longitude_edit)
        layout.addWidget(QtWidgets.QLabel("Zoom Level:"))
        layout.addWidget(self.zoom_edit)
        layout.addWidget(self.fetch_button)
        layout.addWidget(self.map_label)
        
        self.setLayout(layout)
        self.show()

    def deg2num(self, lat_deg, lon_deg, zoom):
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.log(math.tan(lat_rad) +
                    (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return (xtile, ytile)
    
    def getImageTile(self, lat=51.88863727036334, lon=10.417070845502911, delta_lat=0.001,  delta_long=0.001, zoom=19):
        xmin, ymax = self.deg2num(lat, lon, zoom)
        xmax, ymin = self.deg2num(lat + delta_lat, lon + delta_long, zoom)

        Cluster = Image.new('RGB', ((xmax-xmin+1)*256-1, (ymax-ymin+1)*256-1))
        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin,  ymax+1):
                try:
                    imgurl = f"http://tile.openstreetmap.org/{zoom}/{xtile}/{ytile}.png"
                    # print("Opening: " + imgurl)
                    imgstr = requests.get(imgurl, headers=self.headers)
                    tile = Image.open(BytesIO(imgstr.content))
                    Cluster.paste(tile, box=(
                        (xtile-xmin)*256,  (ytile-ymin)*255))
                except Exception as ex:
                    print(f"Couldn't download image: {ex}")
                    tile = None

        return Cluster
            
    def fetchMap(self):
        lat = self.latitude_edit.text()
        lon = self.longitude_edit.text()
        zoom = self.zoom_edit.text()
        
        image = self.getImageTile(lat, lon, 0.001, 0.001, zoom)
        self.img_widget.setPixmap(QPixmap.fromImage(ImageQt.ImageQt(image)))    
            
        # url = f"https://tile.openstreetmap.org/{zoom}/{lon}/{lat}.png"
        # response = requests.get(url)
        # if response.status_code == 200:
        #     image = QtGui.QImage.fromData(response.content)
        #     pixmap = QtGui.QPixmap.fromImage(image)
        #     self.map_label.setPixmap(pixmap)
        # else:
        #     self.map_label.setText("Failed to fetch image")

app = QtWidgets.QApplication(sys.argv)
window = MapWindow()
sys.exit(app.exec_())
