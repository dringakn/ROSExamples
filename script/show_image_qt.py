#!/usr/bin/env python3

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
