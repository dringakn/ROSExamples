#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    BuildingMesh provides utilities to triangulate a simple 2D polygon and
    build a 3D mesh by extruding its walls and capping its top surface.

Features:
  • Polygon triangulation via ear‐clipping:
      – _earclip(): splits a simple, ordered, closed polygon into triangles.
      – Handles numeric stability with an epsilon guard.
  • Wall extrusion:
      – extrude_polygon_sides(height): generates side‐wall triangles by extruding
        the base polygon upward by its height attribute.
  • Top‐face capping:
      – extrude_polygon_cap(): produces triangles covering the top face at full height.
  • Area computation:
      – total_area(triangles): sums triangle areas using a stable Heron’s formula.
  • Orientation‐agnostic:
      – Detects and reverses clockwise input to maintain CCW winding.
  • Pure Python, no external deps beyond the standard library.

Usage Example:
    poly = [
        (0.0, 0.0, 0.0, 3.0),
        (1.0, 0.0, 0.0, 3.0),
        (1.0, 1.0, 0.0, 3.0),
        (0.0, 1.0, 0.0, 3.0),
        (0.0, 0.0, 0.0, 3.0)  # closed loop
    ]
    mesh = BuildingMesh(poly)
    # walls
    verts_sides, tris_sides = mesh.extrude_polygon_sides()
    # top cap
    verts_cap,  tris_cap  = mesh.extrude_polygon_cap()
    # compute area of cap
    area = mesh.total_area([verts_cap[i] for i in tris_cap])

"""

import math
import sys
from collections import namedtuple

# Internal data structure
Point = namedtuple("Point", "x y z h")

class BuildingMesh:

    EPS = math.sqrt(sys.float_info.epsilon)
    
    def __init__(self, polygon):
        """Triangulate the input 2D polygon.
        Assuming the input polygon has an orderd set of vertices.
        Note: The last vertex is same as first to indicate closed polygon.

        Args:
            polygon (list((x,y))): List of polygon vertices, polygon is expected to be an array of 2-tuples of the cartesian points of the polygon.
            output_order_ccw (bool, optional): The output triangle vertice order. Defaults to True.
        """

        # Convert the input list of points as namedtuple
        self.polygon = [Point(*point) for point in polygon]
        # self.polygon = [Point(pt[0], pt[1], pt[2], pt[3]) for pt in polygon]
        
        # Check if it's a closed polygon
        if self.polygon[0] == self.polygon[-1]:
            self.polygon = self.polygon[0:-1]
            # print(f"Closed-loop polygon:\n{self.polygon}")

        # Number of polygon vertices
        self.N = len(self.polygon)

        # Reverse the order of the polygon vertices if required by the desired direction.
        if self._is_clockwise():
            self.polygon.reverse()

    def extrude_polygon_sides(self, height=1):
        # Assuming polygon vertices are ordered
        vtx = [[pt.x, pt.y, pt.z] for pt in self.polygon] # copy original list
        vt0 = self.polygon[0]
        vtx.append([vt0[0], vt0[1], vt0[2]])  # Append first vertex to make close loop polygon
        N = len(vtx)
        
        # Assuming all vertices contains same height
        height = vt0[3]
        
        # Iterate over polygon vertices
        tri_idx = []
        for idx in range(0, N-1): # Skip last vertex (same as first)
            pt = vtx[idx%N]
            vtx.append([pt[0], pt[1], pt[2]+height])  # Append extruded vertex at the end
            # Each rectangular side consist of two triangles ordered in CW direction
            tri_idx.append([idx, idx+1, N+idx])
            tri_idx.append([idx+1, N+idx+1, N+idx])
        
        vtx.append([vt0[0], vt0[1], vt0[2]+height])  # extruded and append closed-loop vertex
        return vtx, tri_idx

    def extrude_polygon_cap(self, height=1):
        vtx = [[pt.x, pt.y, pt.z+pt.h] for pt in self.polygon] # copy original list
        tri_idx = []
        triangles = self._earclip()
        for tri in triangles:
            tri_idx.append([tri[0], tri[1], tri[2]])

        return vtx, tri_idx

    def _earclip(self):
        """Ear clipping algorithm for a given polygon triangulation.
        For a polygon with n vertices, it will return n-2 triangles.
        The triangles are returned as an array of 3-tuples where each item in the tuple is a 2-tuple of the cartesian point.
        Implementation Reference:
            - https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf

        Args:

        Returns:
            list(triangles): List of triangle vertices as coordinates or polygon vertices index
        """
        triangles = []
        N = self.N

        if N < 3:
            print(f"Can't triangulate with less than 3 points.")
            return triangles
        
        # List containing all non-processed vertices
        vtx_idx = list(range(N)) # [0:N)
        vtx_remaining = len(vtx_idx)
        
        # Process remaining vertices
        idx = 0 # start vertex
        while vtx_remaining > 3:
            # Get neighbours of current vertext 
            curr = vtx_idx[idx]
            prev = vtx_idx[idx-1]
            next = vtx_idx[idx+1]
            # Check if it's an ear
            if self._is_ear(self.polygon[prev], self.polygon[curr], self.polygon[next]):
                triangles.append([prev, curr, next])
                # Clip the current ear
                vtx_idx.remove(curr)
                vtx_remaining -= 1
            # Circulator
            idx = (idx+1) % (vtx_remaining-1)
                
        # Process last remaining triangle
        idx = 0
        curr = vtx_idx[idx]
        prev = vtx_idx[idx-1]
        next = vtx_idx[idx+1]
        triangles.append([prev, curr, next])
        
        return triangles

    def _is_clockwise(self):
        s = 0
        for i in range(self.N):
            point = self.polygon[i]
            point2 = self.polygon[(i + 1) % self.N]
            s += (point2.x - point.x) * (point2.y + point.y)
            
        return s > 0

    def _is_convex(self, prev, point, next):
        return self._triangle_sum(prev, point, next) < 0

    def _is_ear(self, p1, p2, p3):
        ear = self._contains_no_points(p1, p2, p3) and \
            self._is_convex(p1, p2, p3) and \
            self._triangle_area(p1, p2, p3) > 0
        return ear

    def _contains_no_points(self, p1, p2, p3):
        for pn in self.polygon:
            if pn in (p1, p2, p3):
                continue
            elif self._is_point_inside(pn, p1, p2, p3):
                return False
        return True

    def _is_point_inside(self, p, a, b, c):
        area = self._triangle_area(a, b, c)
        area1 = self._triangle_area(p, b, c)
        area2 = self._triangle_area(p, a, c)
        area3 = self._triangle_area(p, a, b)
        areadiff = abs(area - sum([area1, area2, area3])) < BuildingMesh.EPS
        return areadiff

    def _triangle_area(self, a, b, c):
        return abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0)

    def _triangle_sum(self, a, b, c):
        return a.x * (c.y - b.y) + b.x * (a.y - c.y) + c.x * (b.y - a.y)

    def total_area(self, triangles):
        """Calculate the sum of areas of the triangles specified by coordinates

        Args:
            triangles (list((pt1, pt2, pt2))): List of triangles vertices

        Returns:
            float: Sum of all triangle areas
        """
        result = []
        for triangle in triangles:
            sides = []

            for i in range(3):
                next_index = (i + 1) % 3
                pt = triangle[i]
                pt2 = triangle[next_index]
                # Distance between two points
                side = math.sqrt(
                    math.pow(pt2[0] - pt[0], 2) + math.pow(pt2[1] - pt[1], 2))
                sides.append(side)

            # Heron's numerically stable forumla for area of a triangle:
            # https://en.wikipedia.org/wiki/Heron%27s_formula
            # However, for line-like triangles of zero area this formula can produce an infinitesimally negative value
            # as an input to sqrt() due to the cumulative arithmetic errors inherent to floating point calculations:
            # https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
            # For this purpose, abs() is used as a reasonable guard against this condition.
            c, b, a = sorted(sides)
            area = .25 * \
                math.sqrt(abs((a + (b + c)) * (c - (a - b))
                          * (c + (a - b)) * (a + (b - c))))
            result.append((area, a, b, c))

        return sum(tri[0] for tri in result)
