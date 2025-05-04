#!/usr/bin/env python3

# geodetic_converter.py

"""
Usage:
    from geodetic_converter import GeodeticConverter
    gc = GeodeticConverter()
    gc.initialise_reference(49.7939, 9.9512, 120.0)   # Wurzburg city center, 120 m altitude
    lat, lon, alt = gc.ned2geodetic(100.0, 50.0, -10.0)
    print(f"NED→Geodetic: {lat:.6f}°, {lon:.6f}°, {alt:.2f} m")
"""
import math
import numpy as np


class GeodeticConverter:
    # WGS84 ellipsoid constants
    kSemimajorAxis = 6378137.0
    kSemiminorAxis = 6356752.314245
    kFirstEccentricitySquared = 6.69437999014e-3
    kSecondEccentricitySquared = 6.73949674228e-3
    kFlattening = 1.0 / 298.257223563

    def __init__(self):
        self.have_reference = False

    def is_initialised(self) -> bool:
        return self.have_reference

    def get_reference(self):
        """
        Returns (latitude_rad, longitude_rad, altitude_m)
        """
        return (self._lat0, self._lon0, self._alt0)

    def initialise_reference(self, latitude_deg: float, longitude_deg: float, altitude_m: float):
        """
        Set the geodetic reference point (in degrees/meters),
        and precompute ECEF<->NED transforms.
        """
        # Store reference in radians
        self._lat0 = self.deg2rad(latitude_deg)
        self._lon0 = self.deg2rad(longitude_deg)
        self._alt0 = altitude_m

        # Compute ECEF coordinates of reference
        x0, y0, z0 = self.geodetic2ecef(latitude_deg, longitude_deg, altitude_m)
        self._ecef0 = np.array([x0, y0, z0])

        # Compute rotation matrices
        # Geocentric latitude = atan2(z0, √(x0² + y0²))
        # phi_p = math.atan2(z0, math.hypot(x0, y0))
        # Geodetic latitude
        phi_p = self._lat0
        self._ecef_to_ned = self._ecef2neu(phi_p, self._lon0)
        # NED->ECEF is transpose of ECEF->NED but using geodetic latitude
        self._ned_to_ecef = self._ecef2neu(self._lat0, self._lon0).T

        self.have_reference = True

    def geodetic2ecef(self, lat_deg: float, lon_deg: float, alt: float):
        """
        Converts geodetic coordinates (deg, deg, m) to ECEF (m,m,m)
        """
        lat = self.deg2rad(lat_deg)
        lon = self.deg2rad(lon_deg)
        xi = math.sqrt(1 - self.kFirstEccentricitySquared * math.sin(lat)**2)
        x = (self.kSemimajorAxis / xi + alt) * math.cos(lat) * math.cos(lon)
        y = (self.kSemimajorAxis / xi + alt) * math.cos(lat) * math.sin(lon)
        z = (self.kSemimajorAxis / xi * (1 - self.kFirstEccentricitySquared) + alt) * math.sin(lat)
        return x, y, z

    def ecef2geodetic(self, x: float, y: float, z: float):
        """
        Converts ECEF (m,m,m) to geodetic coordinates (deg,deg,m)
        """
        # Zhu (1994) algorithm
        a = self.kSemimajorAxis
        b = self.kSemiminorAxis
        e2 = self.kFirstEccentricitySquared
        r = math.hypot(x, y)
        Esq = a*a - b*b
        F = 54 * b*b * z*z
        G = r*r + (1 - e2) * z*z - e2 * Esq
        C = (e2*e2 * F * r*r) / (G**3)
        S = math.copysign(abs(1 + C + math.sqrt(C*C + 2*C))**(1/3), 1)
        P = F / (3 * (S + 1/S + 1)**2 * G*G)
        Q = math.sqrt(1 + 2*e2*e2 * P)
        r0 = -(P*e2*r)/(1+Q) + math.sqrt(0.5*a*a*(1+1/Q) - P*(1-e2)*z*z/(Q*(1+Q)) - 0.5*P*r*r)
        U = math.sqrt((r - e2*r0)**2 + z*z)
        V = math.sqrt((r - e2*r0)**2 + (1-e2)*z*z)
        Z0 = b*b * z / (a * V)
        h = U * (1 - b*b/(a*V))
        lat = math.atan((z + self.kSecondEccentricitySquared * Z0) / r)
        lon = math.atan2(y, x)
        return self.rad2deg(lat), self.rad2deg(lon), h

    def ecef2ned(self, x: float, y: float, z: float):
        """
        Converts ECEF (m,m,m) to local NED (north, east, down) in meters
        """
        if not self.have_reference:
            raise RuntimeError("Reference point not set. Call initialise_reference() first.")
        v = np.array([x, y, z]) - self._ecef0
        ned = self._ecef_to_ned.dot(v)
        return ned[0], ned[1], -ned[2]

    def ned2ecef(self, north: float, east: float, down: float):
        """
        Converts local NED (m) to ECEF (m,m,m)
        """
        if not self.have_reference:
            raise RuntimeError("Reference point not set. Call initialise_reference() first.")
        ned = np.array([north, east, -down])
        xyz = self._ned_to_ecef.dot(ned) + self._ecef0
        return float(xyz[0]), float(xyz[1]), float(xyz[2])

    def geodetic2ned(self, lat_deg: float, lon_deg: float, alt: float):
        """
        Converts geodetic (deg,deg,m) to NED (m)
        """
        x, y, z = self.geodetic2ecef(lat_deg, lon_deg, alt)
        return self.ecef2ned(x, y, z)

    def ned2geodetic(self, north: float, east: float, down: float):
        """
        Converts NED (m) to geodetic (deg,deg,m)
        """
        x, y, z = self.ned2ecef(north, east, down)
        return self.ecef2geodetic(x, y, z)

    def geodetic2enu(self, lat_deg: float, lon_deg: float, alt: float):
        """
        Converts geodetic (deg,deg,m) to ENU (east, north, up)
        """
        n, e, d = self.geodetic2ned(lat_deg, lon_deg, alt)
        return e, n, -d

    def enu2geodetic(self, east: float, north: float, up: float):
        """
        Converts ENU (m) to geodetic (deg,deg,m)
        """
        return self.ned2geodetic(north, east, -up)

    @staticmethod
    def rad2deg(rad: float) -> float:
        return (rad / math.pi) * 180.0

    @staticmethod
    def deg2rad(deg: float) -> float:
        return (deg / 180.0) * math.pi

    @staticmethod
    def _ecef2neu(lat_rad: float, lon_rad: float) -> np.ndarray:
        """
        Builds the ECEF→NEU rotation matrix for given lat, lon (radians)
        row 0 = North unit‐vector
        R[0] = [ -sinφ⋅cosλ,  -sinφ⋅sinλ,   cosφ ]
        row 1 = East unit‐vector
        R[1] = [   -sinλ,        cosλ,       0  ]
        row 2 = Up unit‐vector
        R[2] = [ cosφ⋅cosλ,   cosφ⋅sinλ,   sinφ ]        
        """
        sLat = math.sin(lat_rad)
        cLat = math.cos(lat_rad)
        sLon = math.sin(lon_rad)
        cLon = math.cos(lon_rad)

        R = np.zeros((3, 3))
        R[0, 0] = -sLat * cLon
        R[0, 1] = -sLat * sLon
        R[0, 2] =  cLat
        R[1, 0] = -sLon
        R[1, 1] =  cLon
        R[1, 2] =  0.0
        R[2, 0] =  cLat * cLon
        R[2, 1] =  cLat * sLon
        R[2, 2] =  sLat
        return R
