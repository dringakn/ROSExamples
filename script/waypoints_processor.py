#!/usr/bin/env python3
"""
Author:        Dr.-Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com
Description:   waypoints_processor.py

Provides Waypoint and WaypointsProcessor classes to:
  • load and save waypoint lists (including yaw) to/from GeoJSON
  • convert between GPS, ENU, NED and ECEF frames via GeodeticConverter
  • compute headings (auto, manual, fixed, point‐of‐interest)
  • interpolate, measure distance, track progress through a waypoint list
  • generate common mission patterns (takeoff/land, rectangle, circle, spiral,
    lawnmower, figure‐eight, Lissajous, cardioid, Hilbert, etc.)
"""

import json
import math
from typing import List, Tuple, Optional, Callable, Dict
from dataclasses import dataclass
from geographic_converter import GeodeticConverter

@dataclass
class Waypoint:
    """3D waypoint with optional yaw (radians)."""
    x: float
    y: float
    z: float
    yaw: Optional[float] = None

class WaypointsProcessor:
    def __init__(self, params: dict):
        self.gc = GeodeticConverter()
        self._init_state()
        self._load_parameters(params)
        self._build_coord_maps()

    def _init_state(self):
        self.waypoints: List[Waypoint] = []
        self.odometry = Waypoint(0.0, 0.0, 0.0, 0.0)
        self.poi = Waypoint(0.0, 0.0, 0.0)
        self.fixed_angle = 0.0
        self.reference_altitude = 0.0

        self.heading_mode = "auto"
        self.interpolate_waypoints = False
        self.intermediate_waypoint_distance = 1.0
        self.takeoff_height = 0.0
        self.landing_height = 0.0

        self.current_segment = 0
        self.total_distance = 0.0
        self.distance_tolerance = 0.1

    def _load_parameters(self, p: dict):
        req = {"heading_mode","interpolate_waypoints","intermediate_waypoint_distance",
               "takeoff_height","landing_height"}
        missing = req - set(p)
        if missing:
            raise ValueError(f"Missing parameters: {missing}")

        hm = p["heading_mode"]
        if hm not in ("auto","manual","fixed","poi"):
            raise ValueError("heading_mode must be auto, manual, fixed, or poi")
        if p["intermediate_waypoint_distance"] <= 0:
            raise ValueError("intermediate_waypoint_distance must be > 0")
        if p["takeoff_height"] < 0 or p["landing_height"] < 0:
            raise ValueError("takeoff/landing heights must be >= 0")

        self.heading_mode = hm
        self.interpolate_waypoints = bool(p["interpolate_waypoints"])
        self.intermediate_waypoint_distance = float(p["intermediate_waypoint_distance"])
        self.takeoff_height = float(p["takeoff_height"])
        self.landing_height = float(p["landing_height"])

    def _build_coord_maps(self):
        def to_local_gps(wp: Waypoint):
            return self.gc.geodetic2enu(wp.x, wp.y, wp.z + self.reference_altitude)
        def to_local_enu(wp: Waypoint):
            return wp.x, wp.y, wp.z
        def to_local_ned(wp: Waypoint):
            return wp.y, wp.x, -wp.z
        def to_local_ecef(wp: Waypoint):
            n,e,d = self.gc.ecef2ned(wp.x, wp.y, wp.z)
            return e, n, -d

        def from_local_gps(x: float, y: float, z: float):
            lat, lon, alt_abs = self.gc.enu2geodetic(x, y, z)
            return Waypoint(lat, lon, alt_abs - self.reference_altitude)
        def from_local_enu(x: float, y: float, z: float):
            return Waypoint(x, y, z)
        def from_local_ned(x: float, y: float, z: float):
            return Waypoint(y, x, -z)
        def from_local_ecef(x: float, y: float, z: float):
            xe, ye, ze = self.gc.ned2ecef(y, x, -z)
            return Waypoint(xe, ye, ze)

        self._to_local_map: Dict[str, Callable[[Waypoint], Tuple[float,float,float]]] = {
            "gps": to_local_gps,
            "enu": to_local_enu,
            "ned": to_local_ned,
            "ecef": to_local_ecef,
        }
        self._from_local_map: Dict[str, Callable[[float,float,float], Waypoint]] = {
            "gps": from_local_gps,
            "enu": from_local_enu,
            "ned": from_local_ned,
            "ecef": from_local_ecef,
        }

    def _to_local(self, wp: Waypoint, coord: str) -> Tuple[float,float,float]:
        try:
            return self._to_local_map[coord](wp)
        except KeyError:
            raise ValueError(f"Unsupported coord: {coord}")

    def _from_local(self, x: float, y: float, z: float, coord: str) -> Waypoint:
        try:
            return self._from_local_map[coord](x, y, z)
        except KeyError:
            raise ValueError(f"Unsupported coord: {coord}")

    def _normalize_yaw(self, yaw: float) -> float:
        return math.atan2(math.sin(yaw), math.cos(yaw))

    def _select_yaw(self,
                    prev: Waypoint,
                    tx: float,
                    ty: float,
                    downstream: Optional[float] = None
                    ) -> float:
        m = self.heading_mode
        if m == "auto":
            raw = math.atan2(ty - prev.y, tx - prev.x)
        elif m == "fixed":
            raw = self.fixed_angle
        elif m == "poi":
            raw = math.atan2(self.poi.y - prev.y, self.poi.x - prev.x)
        else:  # manual
            raw = downstream if downstream is not None \
                else (prev.yaw if prev.yaw is not None else (self.odometry.yaw or 0.0))
        return self._normalize_yaw(raw)

    def _compute_total_distance(self):
        self.total_distance = sum(
            math.dist((a.x,a.y,a.z),(b.x,b.y,b.z))
            for a,b in zip(self.waypoints,self.waypoints[1:])
        )

    def _update_current_segment(self, x, y, z, thresh=0.5):
        thr2 = thresh*thresh
        n = len(self.waypoints)
        while self.current_segment < n - 1:
            w = self.waypoints[self.current_segment+1]
            if (w.x-x)**2 + (w.y-y)**2 + (w.z-z)**2 < thr2:
                self.current_segment += 1
            else:
                break

    # ─ Public setters/getters ────────────────────────────────────────

    def set_reference(self, lat: float, lon: float, alt: float):
        self.gc.initialise_reference(lat, lon, alt)
        self.reference_altitude = alt

    def get_reference(self) -> Tuple[float,float,float]:
        lat0, lon0, alt0 = self.gc.get_reference()
        self.reference_altitude = alt0
        return lat0, lon0, alt0

    def set_odometry(self, x: float, y: float, z: float, yaw: float):
        self.odometry = Waypoint(x, y, z, yaw)
        self._update_current_segment(x, y, z)

    def get_odometry(self) -> Waypoint:
        return self.odometry

    def set_point_of_interest(self, poi: Tuple[float,float,float], coord: str):
        self.poi = Waypoint(*self._to_local(Waypoint(*poi), coord))

    def get_point_of_interest(self, coord: str) -> Tuple[float,float,float]:
        poi = self._from_local(self.poi.x, self.poi.y, self.poi.z, coord)
        return poi.x, poi.y, poi.z

    def set_fixed_angle(self, deg: float):
        self.fixed_angle = math.radians(deg)

    def get_fixed_angle(self) -> float:
        return math.degrees(self.fixed_angle)

    def get_total_distance(self) -> float:
        return self.total_distance

    def get_current_segment(self) -> int:
        return self.current_segment

    def get_waypoints_local(self) -> List[Waypoint]:
        return self.waypoints

    def get_waypoints_gps(self) -> List[Tuple[float,float,float,float]]:
        return [(*self.gc.enu2geodetic(w.x,w.y,w.z), w.yaw) for w in self.waypoints]

    def get_distance_between(self,
                             wp1: Waypoint,
                             wp2: Waypoint,
                             coord: str) -> float:
        x1,y1,z1 = self._to_local(wp1, coord)
        x2,y2,z2 = self._to_local(wp2, coord)
        return math.dist((x1,y1,z1),(x2,y2,z2))

    def reset(self):
        self.waypoints = []
        self.current_segment = 0
        self.total_distance = 0.0

    # ─ Planner ────────────────────────────────────────────────────────

    def goto_waypoints(self,
                       raw_wps: List[Waypoint],
                       liftoff: bool,
                       coord: str) -> List[Waypoint]:
        if not raw_wps:
            return []
        self.reset()
        self.waypoints.append(self.odometry)

        # vertical hop
        if liftoff:
            tx,ty,tz = self._to_local(raw_wps[0], coord)
            yaw = self._select_yaw(self.odometry, tx, ty, raw_wps[0].yaw)
            self.waypoints.append(Waypoint(self.odometry.x,
                                           self.odometry.y,
                                           tz,
                                           yaw))

        # traverse
        for wp in raw_wps:
            tx,ty,tz = self._to_local(wp, coord)
            prev = self.waypoints[-1]
            yaw = self._select_yaw(prev, tx, ty, wp.yaw)
            self.waypoints.append(Waypoint(tx, ty, tz, yaw))

        if self.interpolate_waypoints:
            self._interpolate()
        self._compute_total_distance()
        return self.waypoints

    def goto_waypoint(self,
                      raw_wp: Waypoint,
                      coord: str) -> List[Waypoint]:
        return self.goto_waypoints([raw_wp], liftoff=False, coord=coord)

    def goto_height(self, height: float) -> List[Waypoint]:
        tgt = Waypoint(self.odometry.x, self.odometry.y, height)
        return self.goto_waypoint(tgt, coord="enu")

    def takeoff(self) -> List[Waypoint]:
        return self.goto_height(self.odometry.z + self.takeoff_height)

    def land(self) -> List[Waypoint]:
        return self.goto_height(self.landing_height)

    def abort(self):
        self.reset()

    # ─ Interpolation ─────────────────────────────────────────────────┐

    def _interpolate(self):
        if len(self.waypoints) < 2:
            return
        out = [self.waypoints[0]]
        sep = self.intermediate_waypoint_distance

        for a,b in zip(self.waypoints, self.waypoints[1:]):
            prev = a
            dist = math.dist((a.x,a.y,a.z),(b.x,b.y,b.z))
            while dist > sep + self.distance_tolerance:
                frac = sep/dist
                nx = prev.x + frac*(b.x-prev.x)
                ny = prev.y + frac*(b.y-prev.y)
                nz = prev.z + frac*(b.z-prev.z)
                nyaw = self._select_yaw(prev, b.x, b.y, downstream=b.yaw)
                iwp = Waypoint(nx, ny, nz, nyaw)
                out.append(iwp)
                prev = iwp
                dist = math.dist((prev.x,prev.y,prev.z),(b.x,b.y,b.z))
            out.append(b)

        self.waypoints = out

    # ─ Primitives ──────────────────────────────────────────────────────

    def _build_path(self,
                    enu_pts: List[Tuple[float,float,float]],
                    coord: str) -> List[Waypoint]:
        return self.goto_waypoints([Waypoint(*p) for p in enu_pts],
                                   liftoff=False, coord=coord)

    def _parametric_path(self,
                        fn: Callable[[float], Tuple[float,float]],
                        steps: int,
                        cz: float,
                        coord: str
                        ) -> List[Waypoint]:
        # sample θ from 0→2π, close the loop by adding fn(0)
        enu_pts = [
            (*fn(2*math.pi*i/steps), cz)
            for i in range(steps)
        ] + [(*fn(0.0), cz)]
        return self._build_path(enu_pts, coord)

    def create_rectangle(self,
                         center: Waypoint,
                         length: float,
                         width: float,
                         coord: str,
                         altitude: Optional[float] = None
                         ) -> List[Waypoint]:
        if length <= 0 or width <= 0:
            raise ValueError("length and width must be > 0")
        cx,cy,cz = self._to_local(center, coord)
        cz = cz if altitude is None else altitude
        hl,hw = length/2, width/2
        corners = [
            (cx-hl, cy-hw, cz),
            (cx-hl, cy+hw, cz),
            (cx+hl, cy+hw, cz),
            (cx+hl, cy-hw, cz),
        ]
        return self._build_path(corners+[corners[0]], coord)

    def create_circle(self, center: Waypoint, radius: float, coord: str):
        cx, cy, cz = self._to_local(center, coord)
        circ = 2*math.pi*radius
        steps = max(8, math.ceil(circ/self.intermediate_waypoint_distance))
        fn = lambda theta: (cx + radius*math.cos(theta),
                        cy + radius*math.sin(theta))
        return self._parametric_path(fn, steps, cz, coord)

    def create_ellipse(self, center, a: float, b: float, coord: str):
        cx, cy, cz = self._to_local(center, coord)
        steps = max(12, math.ceil(2*math.pi*max(a,b)/self.intermediate_waypoint_distance))
        fn = lambda theta: (cx + a*math.cos(theta),
                        cy + b*math.sin(theta))
        return self._parametric_path(fn, steps, cz, coord)

    def create_spiral(self,
                      center: Waypoint,
                      r_max: float,
                      turns: int,
                      coord: str,
                      altitude: float = None
                      ) -> List[Waypoint]:
        """Create a spiral path (outward Archimedean spiral) with increasing radius."""
        cx, cy, cz0 = self._to_local(center, coord)
        cz = cz0 if altitude is None else altitude

        θ_max = 2*math.pi*turns
        # choose step size ~ intermediate_waypoint_distance
        sep = self.intermediate_waypoint_distance
        # approximate total length = ∫₀^θₘₐₓ √((r′(θ))² + (0)²) dθ 
        # but roughly ≈ sqrt((r_max/θ_max * Δθ)² + 0) = r_max/θ_max * Δθ, so Δθ ~ sep*θ_max/r_max
        steps = max(16, math.ceil(r_max*turns*2*math.pi / sep))
        
        pts = []
        for i in range(steps+1):
            θ = θ_max * i/steps
            r = r_max * θ/θ_max
            x = cx + r * math.cos(θ)
            y = cy + r * math.sin(θ)
            pts.append((x, y, cz))
        return self._build_path(pts, coord)

    def create_lissajous(self,
                         center: Waypoint,
                         A: float,
                         B: float,
                         a: int,
                         b: int,
                         delta: float,
                         coord: str,
                         altitude: float = None
                         ) -> List[Waypoint]:
        """Create a Lissajous curve (sensor‐sweep pattern) with given parameters."""
        cx, cy, cz = self._to_local(center, coord)
        cz = cz if altitude is None else altitude
        # calculate the least common multiple of a and b
        g = math.gcd(a, b)
        lcm = abs(a*b) // g if g else 0
        steps = max(32, lcm * 8)
        fn = lambda theta: (
            cx + A * math.sin(a*theta + delta),
            cy + B * math.sin(b*theta)
        )
        return self._parametric_path(fn, steps, cz, coord)

    def create_lemniscate(self,
                          center: Waypoint,
                          r: float,
                          coord: str,
                          altitude: float = None
                          ) -> List[Waypoint]:
        """Create a lemniscate (figure‐of‐eight, infinite-symbol) path with given parameters."""
        cx, cy, cz = self._to_local(center, coord)
        cz = cz if altitude is None else altitude
        steps = max(16, math.ceil(2*math.pi*r/self.intermediate_waypoint_distance))
        fn = lambda theta: (
            cx + r * math.sqrt(abs(math.cos(2*theta))) * math.cos(theta),
            cy + r * math.sqrt(abs(math.cos(2*theta))) * math.sin(theta)
        )
        return self._parametric_path(fn, steps, cz, coord)

    def create_cardioid(self,
                        center: Waypoint,
                        r: float,
                        coord: str,
                        altitude: float = None
                        ) -> List[Waypoint]:
        """Create a cardioid (heart‐shaped) path with given parameters."""
        cx, cy, cz = self._to_local(center, coord)
        cz = cz if altitude is None else altitude
        steps = max(16, math.ceil(2*math.pi*r/self.intermediate_waypoint_distance))
        fn = lambda theta: (
            cx + r*(1 - math.cos(theta))*math.cos(theta),
            cy + r*(1 - math.cos(theta))*math.sin(theta)
        )
        return self._parametric_path(fn, steps, cz, coord)

    def create_lawnmower(self,
                         center: Waypoint,
                         width: float,
                         height: float,
                         rows: int,
                         coord: str,
                         altitude: float = None
                         ) -> List[Waypoint]:
        """Create a lawnmower (zig-zag) path with given parameters."""
        cx, cy, cz = self._to_local(center, coord)
        cz = cz if altitude is None else altitude
        dy = height / (rows - 1) if rows > 1 else 0
        pts: List[Tuple[float,float,float]] = []
        for i in range(rows):
            y = cy + (i * dy) - height/2
            x0, x1 = cx - width/2, cx + width/2
            line = [(x0, y, cz), (x1, y, cz)]
            if i % 2:  # reverse every other row
                line.reverse()
            pts.extend(line)
        return self._build_path(pts, coord)
    
    def create_figure_eight(self,
                            center: Waypoint,
                            radius: float,
                            coord: str) -> List[Waypoint]:
        if radius <= 0:
            raise ValueError("radius must be > 0")
        cx,cy,cz = self._to_local(center, coord)
        circ = 2*math.pi*radius
        n = max(8, math.ceil(circ/self.intermediate_waypoint_distance))
        left = (cx-radius, cy)
        right= (cx+radius, cy)

        pts = []
        pts += [(left[0]+radius*math.cos(2*math.pi*i/n),
                 left[1]+radius*math.sin(2*math.pi*i/n), cz)
                for i in range(n)]
        pts += [(right[0]+radius*math.cos(2*math.pi*(n-i)/n),
                 right[1]+radius*math.sin(2*math.pi*(n-i)/n), cz)
                for i in range(n)]
        return self._build_path(pts+[pts[0]], coord)

    def create_helix(self, center, radius, height, turns, coord):
        """Vary z linearly with theta for a corkscrew climb or descent."""
        cx, cy, cz0 = self._to_local(center, coord)
        total_theta = 2*math.pi*turns
        steps = max(16, math.ceil(total_theta*radius / self.intermediate_waypoint_distance))
        fn = lambda theta: (
            cx + radius*math.cos(theta),
            cy + radius*math.sin(theta)
        )
        # sample θ from 0→total_theta, and lift z
        enu_pts = [
            (*fn(total_theta*i/steps),
            cz0 + height*(i/steps))
            for i in range(steps+1)
        ]
        return self._build_path(enu_pts, coord)

    def create_star(self, center, radius, points, skip, coord):
        """A regular n-pointed star by skipping vertices on a circle."""
        cx, cy, cz = self._to_local(center, coord)
        # generate circle vertices
        verts = [
        (cx + radius*math.cos(2*math.pi*i/points),
        cy + radius*math.sin(2*math.pi*i/points),
        cz)
        for i in range(points)
        ]
        # connect i→i+skip modulo
        idx = 0
        path = []
        for _ in range(points):
            path.append(verts[idx])
            idx = (idx + skip) % points
        path.append(path[0])
        return self._build_path(path, coord)

    def create_rosette(self, center, R, alpha, k, coord):
        """Radius oscillates: r(θ)=R+α·sin(kθ) for a petaled flower."""
        cx, cy, cz = self._to_local(center, coord)
        steps = max(12, math.ceil(2*math.pi*(R+alpha)/self.intermediate_waypoint_distance))
        fn = lambda theta: (
            cx + (R + alpha*math.sin(k*theta))*math.cos(theta),
            cy + (R + alpha*math.sin(k*theta))*math.sin(theta)
        )
        return self._parametric_path(fn, steps, cz, coord)

    def create_expanding_square(self, center, step, loops, coord):
        """A search pattern that spirals out in a square box sequence."""
        cx, cy, cz = self._to_local(center, coord)
        pts = [(cx, cy, cz)]
        for i in range(1, loops+1):
            d = step * i
            # right, up, left, down
            pts += [(cx + d, cy, cz),
                    (cx + d, cy + d, cz),
                    (cx - d, cy + d, cz),
                    (cx - d, cy - d, cz)]
        return self._build_path(pts, coord)

    def create_sector_scan(self, center, radius, start_b, end_b, revolutions, coord):
        """Sweep a fixed-radius arc back-and-forth between two bearings."""
        cx, cy, cz = self._to_local(center, coord)
        theta0, theta1 = math.radians(start_b), math.radians(end_b)
        total_runs = 2*revolutions
        pts = []
        for run in range(total_runs):
            thetas = [theta0 + (theta1-theta0)*i/50 for i in range(51)]
            if run % 2: thetas.reverse()
            for theta in thetas:
                pts.append((cx + radius*math.cos(theta),
                            cy + radius*math.sin(theta), cz))
        return self._build_path(pts, coord)

    def create_diagonal_zigzag(self, center, length, width, spacing, coord):
        """Cover a rectangle in diagonal stripes."""
        cx, cy, cz = self._to_local(center, coord)
        cols = math.ceil(length/spacing)
        rows = math.ceil(width/spacing)
        pts = []
        for r in range(rows+1):
            y = cy - width/2 + r*spacing
            for c in range(cols+1):
                x = cx - length/2 + (c if r%2==0 else cols-c)*spacing
                pts.append((x, y, cz))
        return self._build_path(pts, coord)

    def create_hilbert(self, center, size, order, coord):
        """A simple e-drone Hilbert approximation (useful for dense area coverage)"""
        # generate 2D Hilbert points in [0,1]^2, then scale & translate
        def hilbert(n):
            if n==0: return [(0,0)]
            prev = hilbert(n-1)
            pts = []
            for (x,y) in prev:  pts.append((y, x))
            for (x,y) in prev:  pts.append((x, y+1))
            for (x,y) in prev:  pts.append((x+1, y+1))
            for (x,y) in prev:  pts.append((2-y,1-x))
            return pts
        pts_uv = hilbert(order)
        cx, cy, cz = self._to_local(center, coord)
        pts = [(
            cx + (u-2**order/2)*(size/2**order),
            cy + (v-2**order/2)*(size/2**order),
            cz
        ) for u,v in pts_uv]
        return self._build_path(pts, coord)

    def check_within_bounds(self,
                            wps: List[Waypoint],
                            min_b: Waypoint,
                            max_b: Waypoint,
                            coord: str) -> bool:
        min_e = self._to_local(min_b, coord)
        max_e = self._to_local(max_b, coord)
        for wp in wps:
            x,y,z = self._to_local(wp, coord)
            if not (min_e[0]<=x<=max_e[0]
                    and min_e[1]<=y<=max_e[1]
                    and min_e[2]<=z<=max_e[2]):
                return False
        return True

    # ── File I/O ────────────────────────────────────────────────────────
    def import_waypoints_from_geojson(self,
                                      filepath: str,
                                      coord: str = "gps"
                                      ) -> List[Waypoint]:
        """
        Load a GeoJSON FeatureCollection of Point features.
        Each feature must have geometry.coordinates = [lon, lat, (alt)].
        Optional property "yaw" (in radians) is read if present.
        Converts into internal ENU waypoints via _to_local and replaces self.waypoints.
        Returns the new ENU waypoint list.
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        if data.get("type") != "FeatureCollection":
            raise ValueError("GeoJSON must be a FeatureCollection")
        loaded: List[Waypoint] = []
        for feat in data.get("features", []):
            geom = feat.get("geometry", {})
            if geom.get("type") != "Point":
                continue
            coords = geom.get("coordinates", [])
            if len(coords) < 2:
                continue
            lon, lat = coords[0], coords[1]
            alt = coords[2] if len(coords) >= 3 else 0.0
            yaw = feat.get("properties", {}).get("yaw", None)
            # build a raw waypoint in the specified coord system
            if coord == "gps":
                raw = Waypoint(lat, lon, alt, yaw)
            else:
                # for enu/ned/ecef, coords maps directly to x,y,z
                raw = Waypoint(coords[0], coords[1], alt, yaw)
            ex, ey, ez = self._to_local(raw, coord)
            loaded.append(Waypoint(ex, ey, ez, yaw))
        self.waypoints = loaded
        self._compute_total_distance()
        return self.waypoints

    def export_waypoints_to_geojson(self,
                                    filepath: str
                                    ) -> None:
        """
        Write self.waypoints out as a GeoJSON FeatureCollection.
        Coordinates are in [lon, lat, alt] (absolute alt above ellipsoid),
        and each feature.properties.yaw is the waypoint yaw in radians.
        """
        features = []
        for lat, lon, alt, yaw in self.get_waypoints_gps():
            feat = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [lon, lat, alt],
                },
                "properties": {
                    "yaw": yaw
                }
            }
            features.append(feat)
        geojson = {
            "type": "FeatureCollection",
            "features": features
        }
        with open(filepath, 'w') as f:
            json.dump(geojson, f, indent=2)    