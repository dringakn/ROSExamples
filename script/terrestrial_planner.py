import tkinter as tk
import open_street_map as osm
from plotting import MyPlot
from gui import MyGUI
from shapely.geometry import *
from shapely.ops import *


def update():
    roi, roi_size, roi_offset, radius, inflation, node_size, pct_overlap, merge_boundaries, simplify_scan_locations = gui.read_inputs()
    gdf, bbox, pgx = osm.get_holes_and_bbox(roi, roi_size, roi_offset)

    plot.clear()
    plot.title = f"{roi[0]}_{roi[1]}_{pct_overlap}_{radius}m_{inflation}m"
    plot.axes.set_title(plot.title)
    
    plot.plot_gdf_df(gdf, color='red', alpha=0.5, clear=False)
    
    pgx1 = osm.inflate_holes(pgx, inflation, merge_boundaries)
    
    plot.plot_gdf_df(gdf.buffer(inflation), color='green', alpha=0.5, clear=False)
    
    sampling_distance = osm.calculate_sampling_distance(pct_overlap, radius)
    pts = osm.sample_points_on_polygon_boundary(pgx1, sampling_distance, simplify_scan_locations)
    
    # plot.plot_shp_points(pts, color='green', alpha=1.0, clear=False)

    x = []
    y = []
    scan_locations = []
    scan = []
    for pt in pts.geoms:
        if pt.intersection(pgx):
            scan_locations.append(Point(pt.x, pt.y))
            scan.append(Point(pt.x, pt.y).buffer(radius))
            plot.plot_circle(pt.x, pt.y, radius, color='blue', alpha=0.1, clear=False)
            x.append(pt.x)
            y.append(pt.y)
    
    plot.plot_shp_pg(bbox.buffer(radius), color='gray', alpha=0.1, clear=False)
        
    plot.plot_xy(x, y, color='black', alpha=1, clear=False, marker='o', linestyle='', markersize=node_size)
    
    mpg = unary_union(MultiPolygon(scan))
    MultiPolygon(scan)
    plot.plot_shp_pg(mpg, clear=False, alpha=0.1)
    
    gui.lbl_result['text'] = f"Rammelsberg, Total Scan Locaitons: {len(x)}, Scanned Area: {mpg.area:0.2f} m2"

    osm.save_points_to_shapefile(scan_locations, f'./result/scan_locations.shp')
    plot.fig.savefig(f'./result/scan_plan.png')

if __name__ == '__main__':

    app = tk.Tk()
    app.title('Terrestrial LiDAR Planning - Test')
    # app.state('zoomed')
    # app.attributes('-fullscreen', True)
    # w, h = app.winfo_screenwidth(), app.winfo_screenheight()
    # app.geometry("%dx%d+0+0" % (w, h))

    gui = MyGUI(app, update)
    plot = MyPlot(app, title='', grid='none', figsize=(32, 16))

    app.mainloop()
