"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    Utility module for geospatial sampling and analysis using OSMnx, Shapely, GeoPandas, NumPy, and SciPy.

Features:
  • Export: save Shapely Points/MultiPoint to ESRI Shapefile.  
  • Scaling: linear remapping of a value between arbitrary ranges.  
  • Overlap-based sampling distance: compute the spacing needed so two circles of radius r overlap by a given fraction.  
  • Polygon inflation: buffer a polygon or its interior holes, with optional merging.  
  • Random interior sampling: generate uniformly random points inside any polygon.  
  • Grid sampling: uniformly sample a regular grid of points inside a polygon at specified resolution.  
  • Linear sampling: sample points along LineString or polygon boundary/interior at fixed intervals.  
  • OSM Building extraction: fetch building footprints around a lat/lng via OSMnx, project to metric CRS, compute bounding box with holes.

Dependencies:
    osmnx
    shapely
    geopandas
    numpy
    scipy

Example:
    from geo_utils import (
        save_points_to_shapefile,
        sample_points_in_polygon,
        get_holes_and_bbox
    )
    # uniform grid at 10 m resolution
    multi_pt, pts = sample_points_in_polygon(my_polygon, res=10)
    save_points_to_shapefile(multi_pt, file_name='grid.shp')
    # extract buildings around a point
    gdf, bbox, pgx = get_holes_and_bbox((49.79, 9.95), roi_size=200, roi_offset=20)
"""

import osmnx as ox
from shapely.geometry import *
from shapely.ops import *
import geopandas as gpd
import numpy as np
import scipy.optimize as optimize

def save_points_to_shapefile(pts, file_name='Polygon.shp', crs=3857):
    gpd.GeoSeries(pts, crs=crs).to_file(filename=file_name)

def scale_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def calculate_sampling_distance(pct_overlap, r):

    if pct_overlap < 0:
        pct_overlap = 0
    elif pct_overlap > 1:
        pct_overlap = 1

    r2 = r * r
    intersection_area = pct_overlap * np.pi * r2

    # Initial guess (theta, distance)
    theta = scale_value(pct_overlap, 0.0, 1.0, 0.0, np.pi)
    guess = (theta, 2.0 * r * np.cos(0.5 * theta))

    def func(x): return (intersection_area - r2 * (x[0] - np.sin(x[0])),
                         x[1] - 2.0 * r * np.cos(0.5 * x[0]))

    return optimize.fsolve(func, x0=guess)[1]  # return distance


def inflate_polygon(pg, dist):
    """
    Inflate the specified polygon
    :param pg: polygon
    :param dist: inflation distance
    :return: polygon
    """
    return pg.buffer(dist)


def inflate_holes(pg, dist, merge=True):
    """
    Inflate the holes of the specified polygon
    :param pg: polygon
    :param dist: inflation distance
    :param merge: boolean to merge overlapping polygons
    :return: polygon
    """
    if isinstance(pg, MultiPolygon):
        print('Multipolygon ')
        pg = pg.geoms[0]
        
    pgs = [Polygon(p).buffer(dist) for p in pg.interiors]
    if merge:
        pgx = MultiPolygon(unary_union(pgs))
    else:
        pgx = MultiPolygon(pgs)
        
    return pgx

def random_points_within_pg(pg, num_points=1):
    """
    Get the specified number of random points within the polygon.
    :param pg: polygon
    :param num_points: number of points
    :return: list of points
    """
    min_x, min_y, max_x, max_y = pg.bounds
    points = []
    while len(points) < num_points:
        random_point = Point([np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)])
        if random_point.within(pg):
            points.append(random_point)

    return points


def sample_points_in_polygon(pg, res):
    """
    Uniformly sample the points within the polygon at specified resolution.
    :param pg: polygon
    :param res: resolution in meters
    :return: Multipoint object and list of valid points
    """
    x_min, y_min, x_max, y_max = pg.bounds

    # construct rectangle of points
    x, y = np.round(np.meshgrid(np.arange(x_min, x_max, res), np.arange(y_min, y_max, res)), 4)
    points = MultiPoint(list(zip(x.flatten(), y.flatten())))

    # validate each point falls inside shapes
    valid_points = []
    valid_points.extend(list(points.intersection(pg)))

    return MultiPoint(valid_points), valid_points


def sample_points_on_line(ln, dist=1.0):
    """
    Sample point on a line.
    :param ln: line
    :param dist: sampling distance
    :return: Multipoint object
    """
    pts = [ln.interpolate(d) for d in np.arange(0, ln.length, dist)]
    return MultiPoint(pts)  # unary_union(pts)
    

def sample_points_on_polygon_interior(pg, dist=1.0):
    """
    Sample points from the interior of the polygon
    :param pg: polygon
    :param dist: sampling distance
    :return: Multipoint object
    """
    pts = []
    for ln in pg:
        pts.extend([ln.interpolate(d) for d in np.arange(0, ln.length, dist)])
    return MultiPoint(pts)  # unary_union(pts)


def sample_points_on_polygon_boundary(pg, dist=1.0, simplify=True):
    """
    Sample point on the boundary and interior of the polygon
    :param pg: polygon
    :param dist: samping distance
    :param simplify: boolean to filter nearest neighbours
    :return: Multipoint object
    """
    if pg.boundary.geom_type == 'MultiLineString':
        pts = []
        for ln in pg.boundary.geoms:
            # pts.extend([ln.interpolate(d) for d in np.arange(0, ln.length, dist)])
            for d in np.arange(0, ln.length, dist):
                pt = ln.interpolate(d)
                if simplify:
                    if not pt.buffer(dist/2).intersection(MultiPoint(pts)):
                        pts.append(pt)
                else:
                    pts.append(pt)
                    
        return MultiPoint(pts)  # unary_union(pts)

    elif pg.boundary.type == 'LineString':
        return sample_points_on_line(pg.boundary, dist)


def get_holes_and_bbox(roi, roi_size=100, roi_offset=10, crs='epsg:3857'):
    """
    Get buildings outline as holes and a bounding box around them
    :param roi: (lat, lng)
    :param roi_size: Side length of the square around region of interest in meters
    :param roi_offset: Inflation distance in meters around the roi
    :return:
    gdf = holes as a geo-pandas dataframe
    bbox = bounding box as shapely polygon
    pgx = shapely bounding-box polygon with holes
    """

    try:
        # new API in OSMnx v2+
        gdf = ox.features_from_point(roi, dist=roi_size, tags={'building': True})
    except AttributeError:
        # fallback for older OSMnx
        gdf = ox.geometries_from_point(roi, dist=roi_size, tags={'building': True})

    gdf = ox.projection.project_gdf(gdf, to_crs=crs)
    gdf.reset_index(drop=True, inplace=True)
    bbox = gdf.bounds.agg({'minx': 'min', 'miny': 'min', 'maxx': 'max', 'maxy': 'max'}).values
    bbox = box(bbox[0] - roi_offset, bbox[1] - roi_offset, bbox[2] + roi_offset, bbox[3] + roi_offset)

    # Combine the overlapping/touching geometries
    gdf = gpd.GeoDataFrame(geometry=[gdf.geometry.unary_union])

    # Separate the multipolygon shape into regions
    gdf = gdf.explode(index_parts=False).reset_index(drop=True)

    # Subtract holes from bbox
    pgx = bbox
    for idx, row in gdf.iloc[0:].iterrows():
        pgx = pgx.difference(row.geometry.simplify(1))

    return gdf, bbox, pgx
