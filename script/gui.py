import tkinter as tk
from tkinter import ttk
import math


class MyGUI:
    def __init__(self, app, update):
        self.lbl_result = tk.Label(app, text='Result')
        self.lbl_result.grid(row=0, column=0)

        frame_roi = tk.Frame(app)

        lbl_lat = tk.Label(frame_roi, text='Latitude:')
        lbl_lat.grid(row=0, column=0)
        self.var_lat = tk.DoubleVar(frame_roi, value=51.8903)
        entry_lat = tk.Spinbox(
            frame_roi, textvariable=self.var_lat, width=9, from_=1, to=180, increment=0.00001)
        entry_lat.grid(row=0, column=1)

        lbl_lng = tk.Label(frame_roi, text='Longitude:')
        lbl_lng.grid(row=0, column=2)
        self.var_lng = tk.DoubleVar(frame_roi, value=10.41933)
        entry_lng = tk.Spinbox(
            frame_roi, textvariable=self.var_lng, width=9, from_=1, to=180, increment=0.00001)
        entry_lng.grid(row=0, column=3)

        lbl_roi_size = tk.Label(frame_roi, text='ROI side length [m]:')
        lbl_roi_size.grid(row=0, column=4)
        self.var_roi_size = tk.DoubleVar(frame_roi, value=100)
        entry_roi_size = tk.Spinbox(
            frame_roi, textvariable=self.var_roi_size, width=6, from_=1, to=10000, increment=1)
        entry_roi_size.grid(row=0, column=5)

        lbl_roi_offset = tk.Label(frame_roi, text='ROI offset [m]:')
        lbl_roi_offset.grid(row=0, column=6)
        self.var_roi_offset = tk.DoubleVar(frame_roi, value=10)
        entry_roi_offset = tk.Spinbox(
            frame_roi, textvariable=self.var_roi_offset, width=6, from_=1, to=10000, increment=1)
        entry_roi_offset.grid(row=0, column=7)
        btn_get_plan = tk.Button(frame_roi, text="Generate Plan", command=update)
        btn_get_plan.grid(row=0, column=8)

        frame_roi.grid(row=1, column=0)

        frame_tessellation = tk.Frame(app)

        # lbl_strategy = tk.Label(frame_tessellation, text='Strategy:')
        # lbl_strategy.grid(row=0, column=0)
        # self.cbox_strategy = ttk.Combobox(frame_tessellation, values=('largest_first', 'random_sequential', 'smallest_last',
        #                                                               'independent_set', 'connected_sequential_bfs',
        #                                                               'connected_sequential_dfs', 'connected_sequential',
        #                                                               'saturation_largest_first'))
        # self.cbox_strategy.current(1)
        # self.cbox_strategy.grid(row=0, column=1)

        lbl_node_size = tk.Label(frame_tessellation, text='Node Size:')
        lbl_node_size.grid(row=0, column=2)
        self.var_node_size = tk.DoubleVar(frame_tessellation, value=4)
        entry_node_size = tk.Spinbox(frame_tessellation, textvariable=self.var_node_size, width=4, from_=0.1, to=100,
                                     increment=0.1)
        entry_node_size.grid(row=0, column=3)

        lbl_pct_overlap = tk.Label(
            frame_tessellation, text='Percentage Overlap:')
        lbl_pct_overlap.grid(row=0, column=4)
        self.var_pct_overlap = tk.DoubleVar(frame_tessellation, value=0.33)
        entry_pct_overlap = tk.Spinbox(frame_tessellation, textvariable=self.var_pct_overlap, width=5, from_=0, to=1,
                                       increment=0.01)
        entry_pct_overlap.grid(row=0, column=5)

        lbl_sensor_radius = tk.Label(
            frame_tessellation, text='Sensor Radius [m]:')
        lbl_sensor_radius.grid(row=0, column=6)
        self.var_sensor_radius = tk.DoubleVar(frame_tessellation, value=15)
        entry_sensor_radius = tk.Spinbox(frame_tessellation, textvariable=self.var_sensor_radius, width=6, from_=1, to=1000,
                                         increment=1)
        entry_sensor_radius.grid(row=0, column=7)

        lbl_inflation_distance = tk.Label(
            frame_tessellation, text='Inflation [m]:')
        lbl_inflation_distance.grid(row=0, column=8)
        self.var_inflation_distance = tk.DoubleVar(frame_tessellation, value=8)
        entry_inflation_distance = tk.Spinbox(frame_tessellation, textvariable=self.var_inflation_distance, width=6, from_=0,
                                              to=1000, increment=1)
        entry_inflation_distance.grid(row=0, column=9)

        self.var_merge_boundries = tk.IntVar(frame_tessellation, value=0)
        chk_merge_boundries = tk.Checkbutton(
            frame_tessellation, text='Merge', variable=self.var_merge_boundries, onvalue=1, offvalue=0)
        chk_merge_boundries.grid(row=0, column=10)

        self.var_simplify_scan_locations = tk.IntVar(
            frame_tessellation, value=1)
        chk_simplify_scan_locations = tk.Checkbutton(
            frame_tessellation, text='Simplify', variable=self.var_simplify_scan_locations, onvalue=1, offvalue=0)
        chk_simplify_scan_locations.grid(row=0, column=11)

        frame_tessellation.grid(row=2, column=0)

    def read_inputs(self):
        """
        Reed the GUI settings.
        :return: ROI, ROI_Size, ROI_Offset, Radius, Inflation, Node_Size, Percentage_Overlap, Merge_Boundries
        """
        # Read Parameters
        lat = self.var_lat.get()
        lng = self.var_lng.get()
        roi_size = self.var_roi_size.get()
        roi_offset = self.var_roi_offset.get()
        radius = self.var_sensor_radius.get()
        node_size = self.var_node_size.get()
        # strategy = self.cbox_strategy.get()
        pct_overlap = self.var_pct_overlap.get()
        inflation = self.var_inflation_distance.get()
        merge_boundries = self.var_merge_boundries.get()
        simplify_scan_locations = self.var_simplify_scan_locations.get()
        roi = (lat, lng)

        return roi, roi_size, roi_offset, radius, inflation, node_size, pct_overlap, merge_boundries, simplify_scan_locations
