"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    A Tkinter‐based GUI for specifying a Region of Interest (ROI) and
    scan/tessellation parameters, then triggering plan generation via a
    user‐supplied callback.

Features:
  • ROI Settings:
      – Latitude & Longitude
      – Square side length & offset in meters
      – “Generate Plan” button  
  • Scan/Tessellation Settings:
      – Node size & overlap percentage
      – Sensor radius & inflation distance in meters
      – Merge boundaries & simplify scan locations toggles  
  • Responsive grid layout  
  • Exposes `read_inputs()` to retrieve all current settings as a tuple

Example:
    import tkinter as tk
    from my_module import MyGUI

    def update():
        roi, size, offset, radius, inflation, node, overlap, merge, simplify = gui.read_inputs()
        # …generate your plan…

    root = tk.Tk()
    gui  = MyGUI(root, update)
    root.mainloop()

Dependencies:
    • Python 3.x (with tkinter & ttk)
"""

import tkinter as tk
from tkinter import ttk


class MyGUI:
    def __init__(self, app, update):
        # make the main window resize nicely
        app.columnconfigure(0, weight=1)

        # Result label
        self.lbl_result = tk.Label(app, text='Result:')
        self.lbl_result.grid(row=0, column=0, sticky="w", padx=10, pady=5)

        # --- ROI settings ---
        frame_roi = ttk.LabelFrame(app, text="Region of Interest", padding=(10,5))
        frame_roi.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        for c in range(9):
            frame_roi.columnconfigure(c, weight=1)

        tk.Label(frame_roi, text='Latitude:').grid(row=0, column=0, sticky="w")
        self.var_lat = tk.DoubleVar(value=51.8903)
        tk.Spinbox(frame_roi, textvariable=self.var_lat,
                   from_=-90, to=90, increment=0.00001,
                   width=9).grid(row=0, column=1, sticky="ew")

        tk.Label(frame_roi, text='Longitude:').grid(row=0, column=2, sticky="w")
        self.var_lng = tk.DoubleVar(value=10.41933)
        tk.Spinbox(frame_roi, textvariable=self.var_lng,
                   from_=-180, to=180, increment=0.00001,
                   width=9).grid(row=0, column=3, sticky="ew")

        tk.Label(frame_roi, text='ROI side [m]:').grid(row=0, column=4, sticky="w")
        self.var_roi_size = tk.DoubleVar(value=100)
        tk.Spinbox(frame_roi, textvariable=self.var_roi_size,
                   from_=1, to=10000, increment=1,
                   width=6).grid(row=0, column=5, sticky="ew")

        tk.Label(frame_roi, text='ROI offset [m]:').grid(row=0, column=6, sticky="w")
        self.var_roi_offset = tk.DoubleVar(value=10)
        tk.Spinbox(frame_roi, textvariable=self.var_roi_offset,
                   from_=0, to=10000, increment=1,
                   width=6).grid(row=0, column=7, sticky="ew")

        ttk.Button(frame_roi, text="Generate Plan", command=update)\
            .grid(row=0, column=8, sticky="e")

        # --- Scan/Tessellation settings ---
        frame_tess = ttk.LabelFrame(app, text="Scan Parameters", padding=(10,5))
        frame_tess.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        for c in range(10):
            frame_tess.columnconfigure(c, weight=1)

        tk.Label(frame_tess, text='Node Size:').grid(row=0, column=0, sticky="w")
        self.var_node_size = tk.DoubleVar(value=4)
        tk.Spinbox(frame_tess, textvariable=self.var_node_size,
                   from_=0.1, to=100, increment=0.1,
                   width=4).grid(row=0, column=1, sticky="ew")

        tk.Label(frame_tess, text='Overlap [%]:').grid(row=0, column=2, sticky="w")
        self.var_pct_overlap = tk.DoubleVar(value=0.33)
        tk.Spinbox(frame_tess, textvariable=self.var_pct_overlap,
                   from_=0, to=1, increment=0.01,
                   width=5).grid(row=0, column=3, sticky="ew")

        tk.Label(frame_tess, text='Sensor Radius [m]:').grid(row=0, column=4, sticky="w")
        self.var_sensor_radius = tk.DoubleVar(value=15)
        tk.Spinbox(frame_tess, textvariable=self.var_sensor_radius,
                   from_=1, to=1000, increment=1,
                   width=6).grid(row=0, column=5, sticky="ew")

        tk.Label(frame_tess, text='Inflation [m]:').grid(row=0, column=6, sticky="w")
        self.var_inflation_distance = tk.DoubleVar(value=8)
        tk.Spinbox(frame_tess, textvariable=self.var_inflation_distance,
                   from_=0, to=1000, increment=1,
                   width=6).grid(row=0, column=7, sticky="ew")

        # use names that match read_inputs
        self.var_merge_boundaries = tk.IntVar(value=0)
        tk.Checkbutton(frame_tess, text='Merge',
                       variable=self.var_merge_boundaries)\
          .grid(row=0, column=8, sticky="w")

        self.var_simplify_scan_locations = tk.IntVar(value=1)
        tk.Checkbutton(frame_tess, text='Simplify',
                       variable=self.var_simplify_scan_locations)\
          .grid(row=0, column=9, sticky="w")


    def read_inputs(self):
        """
        Read the GUI settings and return them in the expected order:
        roi (lat,lon), roi_size, roi_offset, radius, inflation,
        node_size, pct_overlap, merge_boundaries (bool),
        simplify_scan_locations (bool)
        """
        lat = self.var_lat.get()
        lng = self.var_lng.get()
        roi = (lat, lng)
        roi_size = self.var_roi_size.get()
        roi_offset = self.var_roi_offset.get()
        radius = self.var_sensor_radius.get()
        node_size = self.var_node_size.get()
        pct_overlap = self.var_pct_overlap.get()
        inflation = self.var_inflation_distance.get()
        merge_boundaries = bool(self.var_merge_boundaries.get())
        simplify_scan_locations = bool(self.var_simplify_scan_locations.get())

        return (
            roi, roi_size, roi_offset,
            radius, inflation, node_size,
            pct_overlap, merge_boundaries, simplify_scan_locations
        )
