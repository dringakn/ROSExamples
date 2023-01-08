import CGAL
from CGAL.CGAL_Kernel import Polygon_2, Line_2, Segment_2, Point_2, Direction_2, Vector_2, Ray_2, Weighted_point_2
from CGAL.CGAL_Kernel import do_intersect, intersection, squared_distance, orientation, determinant, area, bisector, bounded_side_2, cross_product, left_turn
from CGAL.CGAL_Kernel import COLLINEAR, POSITIVE, NEGATIVE, ORIGIN, CLOCKWISE, COUNTERCLOCKWISE, LEFT_TURN, ON_BOUNDARY, ON_BOUNDED_SIDE, ON_POSITIVE_SIDE, ON_UNBOUNDED_SIDE, ON_NEGATIVE_SIDE
import numpy as np # Numpy
import pyvisgraph as vg # Visibility graph

class PolygonSweep:
    
    def __init__(self, vertices: list()):
        self.vertices = vertices
        # Input polygong with ordered (CW or CCW) vertices
        self.polygon = self.create_cgal_polygon()
        self.sweep_orientations = self.find_polygon_unique_edge_orientation()
        self.optimal_orientation, self.optimal_altitude = self.optimal_sweep_edge()
        self.graph = self.create_visbility_graph()

        
    def create_cgal_polygon(self):
        # Assuming vertices contains duplicated starting vertex
        points = [Point_2(pt[0], pt[1]) for pt in self.vertices[:-1]] # 2D points
        pg = Polygon_2(points)
       
        # Input polygon must have ccw orientation
        if pg.is_clockwise_oriented():
            pg.reverse_orientation()
            
        return pg
       
        
    def find_polygon_unique_edge_orientation(self):
        """Find all non-collinear edges orientation. 
        If there are collinear edges, keep the one with largest length.

        Args:

        Returns:
            edges list(Segment_2): List of unique edge directions.
        """

        # Temporary list inorder to avoid re-calculating edge directions.
        dirs = list()
        edges = list()  # Correponging edges

        for e in self.polygon.edges():
            found = False
            dir = e.to_vector()  # Direction vector from target to source
            for idx, d in enumerate(dirs):
                if (orientation(dir, d)) == COLLINEAR:
                    found = True
                    if e.squared_length() > edges[idx].squared_length():
                        # Replace edge and directon
                        edges[idx] = e
                        dirs[idx] = dir

            if not found:
                # Direction not already present, thus add it to the list
                dirs.append(dir)
                edges.append(e)

        return edges  # Edges are more useful than only dirs


    def optimal_sweep_edge(self):
        """Find the edge with smallest polygon altitude

        Args:

        Returns:
            edge (Segment_2): Polygon edge corresponding to the minimum altitude
            alt (float): Smallest altitude corresonding to the edge
        """
        min_dir = None
        min_altitude = float('inf')
        for dir in self.sweep_orientations:
            for e in self.polygon.edges():
                alt = np.sqrt(squared_distance(dir, e))
                if (alt > 0) and (alt < min_altitude):
                    min_altitude = alt
                    min_dir = dir
                
        return min_dir, min_altitude


    def create_visbility_graph(self):
        # Create visibility graph
        polys = [[vg.Point(p.x(), p.y()) for p in self.polygon.vertices()]]
        return self.build_visgraph(polys)


    def build_visgraph(self, polys: list):
        g = vg.VisGraph()
        # g.build(polys, workers=8, status=True) # Automatic build
        # Manual build, assuming each polygon vertices are in-order (ccw or cw).
        g.graph = vg.Graph(polys)
        g.visgraph = vg.Graph([])

        for edge in g.graph.edges:
            g.visgraph.add_edge(edge)

        return g


    def apply_affine_translation_point(self, pt: Point_2, ov: Vector_2):
        # Apply 2D affine translation (given by the vector) to the point
        T = np.array(((1, 0, ov.x()), (0, 1, ov.y()), (0, 0, 1)))
        tpt = T.dot(np.array((pt.x(), pt.y(), 1)))
        return Point_2(tpt[0], tpt[1])


    def apply_affine_translation_segment(self, ls: Segment_2, ov: Vector_2):
        # Apply 2D affine translation (given by the vector) to the line segment
        T = np.array(((1, 0, ov.x()), (0, 1, ov.y()), (0, 0, 1)))
        s = T.dot(np.array((ls.source().x(), ls.source().y(), 1)))
        e = T.dot(np.array((ls.target().x(), ls.target().y(), 1)))
        return Segment_2(Point_2(s[0], s[1]), Point_2(e[0], e[1]))


    def find_segment_intersection(self, sg: Segment_2):
        """Find the intersection of the given inputs segment with the polygon.

        Args:
            sg (Segment_2): Input segment

        Returns:
            Segment_2: Intersection as segment if exists, otherwise, None
        """
        pts = list()
        ln = sg.supporting_line()
        
        for e in self.polygon.edges():
            obj = intersection(ln, e)
            if obj.is_Segment_2():
                ls = obj.get_Segment_2()
                if ls.source() not in pts:
                    pts.append(ls.source())
                if ls.target() not in pts:
                    pts.append(ls.target())

            elif obj.is_Point_2():
                pt = obj.get_Point_2()
                if pt not in pts:
                    pts.append(pt)
                    
        perp_ln = ln.perpendicular(ln.point(0))
        pts = sorted(pts, key=lambda p: np.sqrt(squared_distance(perp_ln, p))*perp_ln.oriented_side(p))

        if len(pts) == 0:
            return False, None
        
        return True, Segment_2(pts[0], pts[-1])  # Front and Back


    def check_same_direction(self, ls1: Segment_2, ls2: Segment_2):
        """Check if the two line segments have same direction.

        Args:
            ls1 (Segment_2): First line segment
            ls2 (Segment_2): Second line segment

        Returns:
            Bool: True if both have same direction, False otherwise
        """
        v1 = ls1.to_vector()
        v2 = ls2.to_vector()
        v1.normalize()
        v2.normalize()
        dv = v1-v2
        
        return True if ((abs(dv.x()) < 0.001) and (abs(dv.y()) < 0.001)) else False


    def vertices_between_segments(self, ls1: Segment_2, ls2: Segment_2):
        """Find the polygon vertices between the ls1 and ls2.
        Calculate the average distances with respect to ls2.

        Args:
            ls1 (Segment_2): First line segment
            ls2 (Segment_2): Second line segment

        Returns:
            vertices list(Point_2): List of vertices between the two line segemnts.
            avgDist (float): Average distance of points wrt ls2.
        """
        l1 = ls1.supporting_line()
        l2 = ls2.supporting_line()
        vertex_dist = list()
        vertices = list()
        avgDist = 0
        for pt in self.polygon.vertices():
            o_wrt_l1 = l1.oriented_side(pt)
            o_wrt_l2 = l2.oriented_side(pt)
            if (o_wrt_l1 == 1) and (o_wrt_l2 == -1) or ((o_wrt_l1 == -1) and (o_wrt_l2 == 1)):
                vertices.append(pt)
                vertex_dist.append(np.sqrt(squared_distance(ls2, pt)))
                avgDist = np.mean(vertex_dist)
                
        return vertices, avgDist


    def project_point_on_polygon_hull(self, pt: Point_2, offset=0.001):

        closest_edge = None
        edge_distance = float('inf')

        for edge in self.polygon.edges():
            dist = squared_distance(edge, pt)
            if dist < edge_distance:
                edge_distance = dist
                closest_edge = edge

        # Project on the line going through the edge
        closest_pt_on_edge = closest_edge.supporting_line().projection(pt)

        # Check the edge limits
        if closest_pt_on_edge > closest_edge.max():
            closest_pt_on_edge = closest_edge.max()
        elif closest_pt_on_edge < closest_edge.min():
            closest_pt_on_edge = closest_edge.min()

        # Project point at an offset(inward) from the polygon hull.
        dir_vector = closest_edge.supporting_line().perpendicular(
            closest_pt_on_edge).to_vector()
        dir_vector.normalize()
        offset_vector = offset * dir_vector
        proj_pt = self.apply_affine_translation_point(closest_pt_on_edge, offset_vector)
        if self.polygon.bounded_side(proj_pt) != ON_BOUNDED_SIDE:
            offset_vector = -offset * dir_vector  # Reverse the direction
            proj_pt = self.apply_affine_translation_point(closest_pt_on_edge, offset_vector)

        # Find distance and direction, negative for outward, positive for inward, zero for on-boundary
        edge_distance = np.sqrt(squared_distance(pt, proj_pt)) * \
            closest_edge.supporting_line().oriented_side(proj_pt)

        return closest_edge, proj_pt, edge_distance, self.polygon.bounded_side(proj_pt)


    def compute_traversal_time(self, start: Point_2, end: Point_2, v_max: float, a_max: float):
        """Compute the time to go from start to end give maximum velocity and acceleration 

        Args:
            start (Point_2): Start point
            end (Point_2): End point
            v_max (float): Maximum velocity
            a_max (float): Maximum acceleration

        Returns:
            time (float): The total time is acceleration time, cruise time, and deacceleration time
        """
        if (v_max < 0) or (a_max < 0):
            print(f"Invalid v_max[{v_max}], a_max[{a_max}]")
            return -1

        distance = np.sqrt(squared_distance(start, end))
        # Time to accelerate or decelerate to or from maximum velocity:
        acc_time = v_max / a_max
        # Distance covered during complete acceleration or decelerate:
        acc_distance = 0.5 * v_max * acc_time
        # Compute total segment time:
        if (distance < 2.0 * acc_distance):
            # Case 1: Distance too small to accelerate to maximum velocity.
            return (2.0 * np.sqrt(distance / a_max))
        else:
            # Case 2: Distance long enough to accelerate to maximum velocity.
            return (2.0 * acc_time) + ((distance - (2.0 * acc_distance)) / v_max)


    def camera_lateral_offset(self, altitude: float, fov: float=60.0, overlap: float=0.5):
        """Given the camera parameter and drone height above ground, find the sweep offset
            Note: Refer the diagram [camera_frustum_model.png] 
        Args:
            altitude (float): Drone flight height in meters
            fov (float): Camera FOV (Horizontal/Vertical) in radians. Defaults to 60.
            overlap (float): Desired percentage overlap along fov, [0, PI]. Defaults to 50%.

        Returns:
            sweep_line_offset_dist (float): Offset distane in meters between conscetive sweep lines
        """
        fov = np.deg2rad(fov)
        if (fov > 0) and (fov < np.pi):
            footprint = 2.0 * altitude * np.tan(fov/2.0)
            sweep_distance = (1.0 - overlap) * footprint
            print(f"lateral_offset: {sweep_distance:0.2f}")
            return sweep_distance
        else:
            print(f"Invalid FOV: {fov}, Altitude: {altitude}")
            return -1


    def find_camera_height(self, hfov: float=60, vfov: float=50, width: int=1100, height: int=900, gsd: float=0.03):
        """Find the maximum flying height given camera parameters and desires GSD.

        Args:
            hfov (float, optional): Horizontal field of view in degrees. Defaults to 60.
            vfov (float, optional): Vertical field of view in degrees. Defaults to 50.
            width (int, optional): Camera image width in pixels. Defaults to 1100.
            height (int, optional): Camera image height in pixels. Defaults to 900.
            gsd (float, optional): Desired ground sampling distance [m/pixel]. Defaults to 0.03.

        Projected Area: Lx, Ly
        Lx = width * gsd # Horizontal projection
        Ly = height * gsd # Vertical projection
        
        Geomatric relationship
        Lx = width * gsd = 2 * h * np.tan(hfov/2.0)
        Ly = height * gsd = 2 * h * np.tan(vfov/2.0)
        
        Returns:
            height (float): Maximum flight hight for desired GSD
        """
        hfov = np.deg2rad(hfov)
        vfov = np.deg2rad(vfov)
        hmax_hor = (width * gsd) / (2.0 * np.tan(hfov/2.0))
        hmax_ver = (height * gsd) / (2.0 * np.tan(vfov/2.0))
        hmax = min(hmax_hor, hmax_ver)

        print(f"hmax: {hmax:0.2f}, hmax_hor: {hmax_hor:0.2f}, hmax_ver: {hmax_ver:0.2f}")
        return hmax


    def calculate_sweep_segments(self, start: Segment_2, offset: float, reverse_start: bool):
    
        # Find start sweep
        sweep = start.supporting_line()
        offset_dir = sweep.perpendicular(start.source()).direction()  # Vector_2
        offset_vector = Vector_2(offset_dir.dx(), offset_dir.dy())
        offset_vector.normalize()
        offset_vector = offset * offset_vector

        sweep_segment = start
        prev_sweep_segment = sweep_segment  
        sweeps = list()  # Output
        while True:
            # Add sweep segment
            sweeps.append(sweep_segment)

            # Previous line segment
            prev_prev_sweep_segment = prev_sweep_segment
            prev_sweep_segment = sweep_segment

            # Translate the sweep segement by applying affine translation
            sweep_segment = self.apply_affine_translation_segment(sweep_segment, offset_vector)

            # Find next sweep segment
            result, sweep_segment = self.find_segment_intersection(sweep_segment)
            if result == True:
                # Exit if sweep_segment is a degenerated one
                if sweep_segment.is_degenerate():
                    sweeps.append(sweep_segment)
                    result = False
                
                # Reverse direction if find_segment_intersection calculated opposite direction
                elif self.check_same_direction(prev_sweep_segment, sweep_segment) == False:
                    sweep_segment = sweep_segment.opposite()
                
                # Exit if the current and previous sweep segment distance is smaller than threshold
                sq_dist = squared_distance(sweep_segment, prev_sweep_segment)
                if sq_dist < 0.01:
                    result = False

            if  result == False:
                # Add a final sweep wrt to last sweep segment, if possible
                vertices, avg_dist = self.vertices_between_segments(prev_sweep_segment, prev_prev_sweep_segment)
        
                # Don't add very close sweep segments
                if (len(vertices) == 0) or (avg_dist < (offset/2.0)):
                    break

                # Recalculate the last offset vector
                offset_vector.normalize()
                offset_vector = avg_dist * offset_vector
                
                # Translate the sweep segement by applying affine translation
                sweep_segment = self.apply_affine_translation_segment(prev_sweep_segment, offset_vector)
                result, sweep_segment = self.find_segment_intersection(sweep_segment)
                if result:
                    if self.check_same_direction(prev_sweep_segment, sweep_segment) == False:
                        sweeps.append(sweep_segment.opposite())
                    else:
                        sweeps.append(sweep_segment)
                
                # Exit the loop
                break

        # If the polygon is ccw, the start segment is also in ccw
        sweep_segments = [seg.opposite() if reverse_start else seg for seg in sweeps]

        return True, sweep_segments
        

    def calculate_shortest_path(self, start: Point_2, goal: Point_2, thresh=0.001):
        # Connect to points in the polygon using the visibility graph.
        path = self.graph.shortest_path(vg.Point(start.x(), start.y()), vg.Point(goal.x(), goal.y()))

        if len(path) < 2:
            print(f"No valid path from {start} to  {goal}.")
            return False, list()

        shortest_path = [Point_2(path[0].x, path[0].y)]
        for pt in path[1:]:
            ppt = shortest_path[-1] # previous point
            npt = Point_2(pt.x, pt.y) # current point
            if squared_distance(ppt, npt) > thresh: # if distance is greather than threshold
                shortest_path.append(npt) # Append to the output list
                
        return True, shortest_path


    def compute_waypoints(self, sweeps: list(), v_max: float=1.0, a_max: float=1.0, offset: float=0.001):
        waypoints = []
        path_length = 0
        path_time = -1
        
        if len(sweeps) > 1:
            waypoints.append(sweeps[0].source())
            waypoints.append(sweeps[0].target())
            reverse_sweep_path = True
            for sweep in sweeps[1:]:
                if reverse_sweep_path:
                    sweep = sweep.opposite()
                reverse_sweep_path = not reverse_sweep_path  # Invert for next sweep-line
                start = sweep.source()
                goal = sweep.target()
                # connect previous segments
                _, origin, _, _ = self.project_point_on_polygon_hull(waypoints[-1], offset)
                _, destination, _, _ = self.project_point_on_polygon_hull(start, offset)

                result, path = self.calculate_shortest_path(origin, destination)
                if result == True:
                    for pt in path[1:-1]:
                        waypoints.append(pt)
                # Add current segment
                waypoints.append(start)
                waypoints.append(goal)

            # Get Point_2 X,Y,Z coordinates and calculate distance and time.
            points = []
            ppt = waypoints[0] # Previous point
            points.append([ppt.x(), ppt.y(), 0])
            for pt in waypoints[1:]:
                points.append([pt.x(), pt.y(), 0])
                path_length += np.sqrt(squared_distance(ppt, pt))
                path_time += self.compute_traversal_time(ppt, pt, v_max, a_max)
                ppt = pt

        return points, path_length, path_time


    def plan_path(self, reverse=False, v_max=1, a_max=1):
        waypoints = []
        path_length = 0
        path_time = 0

        offset = self.camera_lateral_offset(30, 60, 0.7);
        height = self.find_camera_height(60, 50, 1100, 900, 0.03);
        result, sweeps = self.calculate_sweep_segments(self.optimal_orientation, offset, reverse)
        if result:
            waypoints, path_length, path_time = self.compute_waypoints(sweeps, v_max, a_max, 0.001)
        
        return result, waypoints, path_length, path_time