#!/usr/bin/env python3
# Ensure it runs as a python script!
# Install PyQt or PySide using: pip install pyqt5

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap
import requests
import overpy
from io import BytesIO
from PIL import Image

def fetch_osm_image(lat, lon, zoom):
    api = overpy.Overpass()
    result = api.query("""
        way({0},{1},{2},{3}) ["highway"];
        (._;>;);
        out body;
        """.format(lat - 0.01, lon - 0.01, lat + 0.01, lon + 0.01))

    # Get bounds of the area
    minlat = minlon = 90
    maxlat = maxlon = -90
    for way in result.ways:
        for node in way.nodes:
            if node.lat < minlat:
                minlat = node.lat
            if node.lat > maxlat:
                maxlat = node.lat
            if node.lon < minlon:
                minlon = node.lon
            if node.lon > maxlon:
                maxlon = node.lon

    # Calculate image size based on zoom level
    size = 2**zoom * 256

    # Fetch the map image
    response = requests.get("https://static-maps.yandex.ru/1.x/?l=map&size={0}x{0}&z={1}&ll={2},{3}".format(size, zoom, (minlat + maxlat) / 2, (minlon + maxlon) / 2))
    Image.open(BytesIO(response.content)).show()

class MapDisplay(QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'OSM Map Display'
        self.left = 10
        self.top = 10
        self.width = 640
        self.height = 480
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        # Create line edit to input latitude and longitude
        self.coordinates = QLineEdit(self)
        self.coordinates.move(20, 20)
        self.coordinates.resize(280, 40)
        self.coordinates.setText("49.787590458445095, 9.969618322691607")

        # Create image widget to display the map
        self.image = QLabel(self)
        self.image.move(20, 80)
        self.image.resize(600, 400)

        # Create a button to get the map image
        self.button = QPushButton('Get Map', self)
        self.button.move(320, 20)
        self.button.clicked.connect(self.get_map)

        self.show()

    def get_map(self):
        lat, lon = map(float, self.coordinates.text().split(','))
        # https://maps.wikimedia.org/#19/49.78236/9.94912
        url = f"https://maps.wikimedia.org/#19/{lat:0.5f}/{lon:0.5f}"
        url = "https://maps.wikimedia.org/#19/49.78370/9.94655"
        print(url)
        response = requests.get(url)
        img = QPixmap()
        print(response)
        img.loadFromData(response.content)
        self.image.setPixmap(img)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MapDisplay()
    sys.exit(app.exec_())
