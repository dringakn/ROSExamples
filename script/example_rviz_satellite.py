#!/usr/bin/env python3
"""
Author:        Dr.-Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com
Description:   ROS node that
               1) subscribes to a latched GPS reference (/gps_ref) to set the local ENU origin via GeodeticConverter,
               2) listens for RViz “Publish Point” clicks (/clicked_point), converts them to geodetic NavSatFix messages,
                  emits GeoJSON Features, and renders a sphere marker at each click,
               3) subscribes to incoming GeoJSON (/geojson_in) and visualizes Point and LineString features as RViz markers,
               4) subscribes to a file‐load topic (/load_geojson_file) that takes an absolute path, reads that file,
                  and publishes its contents on /geojson_out for immediate visualization.
"""
import math
import os
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point, Pose, Vector3, Quaternion
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from geographic_converter import GeodeticConverter

class GeoVizNode:
    def __init__(self):
        rospy.init_node("example_rviz_satellite")
        self.gc = GeodeticConverter()
        self.click_id = 0
        self.scale = 10

        # publishers
        self.pub_fix     = rospy.Publisher("clicked_point_gps",    NavSatFix,    queue_size=5)
        self.pub_click_m = rospy.Publisher("clicked_point_marker", Marker,       queue_size=5)
        self.pub_gj_mark = rospy.Publisher("geojson_markers",      MarkerArray,  queue_size=1)
        self.pub_geojson = rospy.Publisher("geojson_out",          String,       queue_size=5)

        # subscribers
        rospy.Subscriber("gps_ref",           NavSatFix,    self.on_ref_fix)
        rospy.Subscriber("clicked_point",     PointStamped, self.on_click)
        rospy.Subscriber("load_geojson_file", String,       self.on_load_file)
        rospy.Subscriber("geojson_in",        String,       self.on_geojson)

        rospy.loginfo("GeoVizNode ready—waiting for /gps_ref…")
        rospy.loginfo("Click RViz PublishPoint Tool, publish to /geojson_in, or send file path to /load_geojson_file")

    def on_ref_fix(self, msg: NavSatFix):
        self.gc.initialise_reference(msg.latitude, msg.longitude, msg.altitude)
        rospy.loginfo(f"Reference set to {msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.2f}m")

    def on_click(self, msg: PointStamped):
        if not self.gc.is_initialised():
            rospy.logwarn("No reference yet—ignoring click")
            return

        # ENU → geodetic + heading
        lat, lon, alt = self.gc.enu2geodetic(msg.point.x, msg.point.y, msg.point.z)
        # heading: 0°=North, 90°=East, 270°=West etc.
        heading_rad   = math.atan2(msg.point.x, msg.point.y)
        heading_deg   = (math.degrees(heading_rad) + 360.0) % 360.0

        fix = NavSatFix(
            header=msg.header,
            latitude=lat,
            longitude=lon,
            altitude=alt
        )
        self.pub_fix.publish(fix)
        rospy.loginfo(f"(x,y,z,°)->(lat,lon,alt): "
                      f"({msg.point.x:.1f},{msg.point.y:.1f},{msg.point.z:.1f},{heading_deg:.1f}) -> "
                      f"({lat:.6f},{lon:.6f},{alt:.2f})")

        # GeoJSON Feature
        feat = {
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [lon, lat, alt]},
            "properties": {
                "speed":        0.0,
                "altitude":     alt,
                "heading":      heading_deg,
                "crs":          "EPSG4979_WGS84",
                "headingType":  "DEGREE_TRUE_NORTH",
                "altitudeType": "ABOVE_ELLIPSOID"
            }
        }
        self.pub_geojson.publish(String(data=json.dumps(feat)))

        # sphere marker at clicked ENU point
        m = Marker(
            header=msg.header,
            ns="clicked_points",
            id=self.click_id,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(
                position=msg.point,
                orientation=Quaternion(x=0, y=0, z=0, w=1)
            ),
            scale=Vector3(self.scale, self.scale, self.scale)
        )
        m.color.r = 1.0
        m.color.a = 1.0
        self.pub_click_m.publish(m)
        self.click_id += 1

    def on_geojson(self, msg: String):
        if not self.gc.is_initialised():
            return

        data = json.loads(msg.data)
        feats = [data] if data.get("type") == "Feature" else data.get("features", [])
        markers = MarkerArray()
        idx = 0

        for feat in feats:
            geom = feat["geometry"]
            typ  = geom["type"]
            coords = geom["coordinates"]

            if typ == "Point":
                lon, lat, alt = (coords + [0.0])[:3]
                e, n, u       = self.gc.geodetic2enu(lat, lon, alt)
                p = Point(x=e, y=n, z=u)

                m = Marker(
                    header=rospy.Header(frame_id="map", stamp=rospy.Time.now()),
                    ns="geojson_points",
                    id=idx,
                    type=Marker.SPHERE,
                    action=Marker.ADD,
                    pose=Pose(
                        position=p,
                        orientation=Quaternion(x=0, y=0, z=0, w=1)
                    ),
                    scale=Vector3(self.scale, self.scale, self.scale)
                )
                m.color.b = 1.0
                m.color.a = 1.0
                markers.markers.append(m)
                idx += 1

            elif typ in ("LineString", "MultiLineString"):
                lines = [coords] if typ == "LineString" else coords
                for line in lines:
                    m = Marker(
                        header=rospy.Header(frame_id="map", stamp=rospy.Time.now()),
                        ns="geojson_paths",
                        id=idx,
                        type=Marker.LINE_STRIP,
                        action=Marker.ADD,
                        pose=Pose(
                            position=Point(x=0, y=0, z=0),
                            orientation=Quaternion(x=0, y=0, z=0, w=1)
                        ),
                        scale=Vector3(self.scale, self.scale, self.scale)
                    )
                    m.color.b = 1.0
                    m.color.a = 1.0

                    for lon, lat in line:
                        e, n, u = self.gc.geodetic2enu(lat, lon, 0.0)
                        m.points.append(Point(x=e, y=n, z=u))

                    markers.markers.append(m)
                    idx += 1

        self.pub_gj_mark.publish(markers)
        rospy.loginfo(f"idx: {idx}")
        rospy.loginfo(f"{markers}")

    def on_load_file(self, msg: String):
        path = msg.data.strip()
        if not os.path.isabs(path):
            rospy.logerr(f"load_geojson_file: not an absolute path: {path}")
            return
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            rospy.loginfo(f"Loaded GeoJSON from {path}")
            self.pub_geojson.publish(String(data=json.dumps(data)))
        except Exception as e:
            rospy.logerr(f"Failed to load GeoJSON {path}: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    GeoVizNode().run()
