#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    A reusable plotting utility class for embedding Matplotlib figures
    in a Tkinter application and easily rendering geometric shapes
    and GeoPandas geometries.

Features:
  • Figure setup:
      – Configurable title, grid style, and figure size
      – Automatic tight layout and subplot margins
  • Basic plotting primitives:
      – plot_xy: scatter/line plots with customizable markers and styles
      – plot_circle: filled circle patches
      – plot_shp_circle / plot_shp_box: buffered point and rectangular shapes
      – plot_shp_point / plot_shp_points: Shapely Point(s) rendering
  • GeoPandas support:
      – plot_gdf_df: render an entire GeoDataFrame
      – plot_shp_pg_ext / plot_shp_pg: exterior boundary or arbitrary Polygon
  • Canvas handling:
      – Automatic clearing, axis equalization, and redraw/update

Usage Example:
    import tkinter as tk
    from myplot import MyPlot  # this file

    root = tk.Tk()
    plot = MyPlot(root, title="Map View", grid='both', figsize=(10, 6))
    # draw a circle at (2,3) radius 1
    plot.plot_circle(2, 3, 1.0, color='blue', alpha=0.5)
    root.mainloop()

Dependencies:
  • matplotlib
  • geopandas
  • shapely
  • tkinter
"""

from matplotlib.figure import Figure
import matplotlib.backends.backend_tkagg as tkagg  # FigureCanvasTkAgg, NavigationToolbar2Tk
import geopandas as gpd
import tkinter as tk
from shapely.geometry import *
import matplotlib.pyplot as plt


class MyPlot:
    def __init__(self, app, title='', grid='none', figsize=(16, 8)):
        self.tk_window = app
        self.fig = Figure(figsize=figsize, dpi=100)
        self.axes = self.fig.add_subplot(111)
        self.title = title
        self.grid = grid
        self.canvas = tkagg.FigureCanvasTkAgg(self.fig, self.tk_window)
        self.canvas._tkcanvas.grid(row=3, column=0)
        self.fig.tight_layout()
        self.fig.subplots_adjust(0.035, 0.15, 0.84, 0.93)  # left, bottom, right, top
        self.clear()

    def equal_axis(self):
        self.axes.axis('equal')

    def clear(self):
        self.axes.clear()
        self.axes.set_title(self.title)
        self.axes.grid(self.grid)
        self.axes.set_axis_off()

    def update(self):
        self.axes.relim()  # Recompute new limits
        self.axes.autoscale_view()  # Apply new limits
        self.canvas.draw()

    def plot_xy(self, x, y, color='red', alpha=1.0, clear=True, marker='', linestyle='-', markersize=1):
        if clear:
            self.clear()
        self.axes.plot(x, y, color=color, alpha=alpha, marker=marker, linestyle=linestyle, markersize=markersize)
        self.update()

    def plot_circle(self, x, y, r, color='red', alpha=1.0, clear=True):
        if clear:
            self.clear()
        self.axes.add_patch(plt.Circle((x, y), r, facecolor=color, edgecolor='none', alpha=alpha))
        self.update()

    def plot_shp_pg_ext(self, pg, color='red', alpha=1.0, clear=True):
        x, y = pg.exterior.xy
        self.plot_xy(x.tolist(), y.tolist(), color, alpha, clear)

    def plot_gdf_df(self, gdf, color='red', alpha=1.0, clear=True):
        if clear:
            self.clear()
        gdf.plot(ax=self.axes, color=color, alpha=alpha)
        self.update()

    def plot_shp_pg(self, pg, color='red', alpha=1.0, clear=True):
        df = gpd.GeoDataFrame(geometry=[pg])
        self.plot_gdf_df(df, color, alpha, clear)

    def buffer_pg(self, pg, dist):
        return pg.buffer(dist)

    def plot_shp_circle(self, x, y, r=1, color='red', alpha=1.0, clear=True):
        pt = Point(x, x)
        self.plot_shp_pg(self.buffer_pg(pt, r), color, alpha, clear)

    def plot_shp_box(self, xmin, ymin, xmax, ymax, color='red', alpha=1.0, clear=True):
        pg = box(xmin, ymin, xmax, ymax)
        self.plot_shp_pg(pg, color, alpha, clear)

    def plot_shp_point(self, pt, color='red', alpha=1.0, clear=True):
        if clear:
            self.clear()

        self.axes.scatter(pt.x, pt.y, color=color, alpha=alpha)
        self.update()

    def plot_shp_points(self, pts, color='red', alpha=1.0, clear=True):
        x = []
        y = []
        [[x.append(pt.x), y.append(pt.y)] for pt in pts.geoms]
        self.plot_xy(x, y, color, alpha, clear, marker='o', linestyle='')
