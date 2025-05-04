#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    PolygonOpen3D is a utility class wrapping common Open3D operations to
    create, manipulate, and query 3D geometries—lines, meshes, point clouds,
    and LiDAR-style raycasts.

Features:
  • create_open3d_lineset         Create a LineSet from 3D vertices, optional color & z‑offset
  • create_open3d_lineset_2d      Create a LineSet from 2D vertices at a default Z height
  • create_mesh                   Build a TriangleMesh from vertices & triangles, compute normals, colorize
  • create_poisson_mesh           Perform Poisson surface reconstruction on a point cloud
  • project_points_on_meshes      Compute closest‑point projections onto meshes
  • project_waypoints_on_meshes   Raycast waypoints onto meshes along a given direction
  • create_lidar_model            Generate a unit‑sphere sampling of directions for LiDAR simulation
  • create_lidar_rays             Assemble origin‑direction rays for a LiDAR scan
  • lidar_scan                    Cast LiDAR rays in a RaycastingScene and return hit point cloud
  • create_plane                  Generate a simple quad mesh in the XY plane
  • create_scene                  Build a RaycastingScene from legacy TriangleMesh objects
  • closest_point_on_meshes       Batch closest‑point and distance queries for many points
  • find_intersection_with_meshes Ray‑mesh intersection distance queries
  • show_meshes                   Visualize meshes in the Open3D GUI with normals & wireframe options
"""

import open3d as o3d
import numpy as np

class PolygonOpen3D:
    
    def __init__(self):
        self.lidar_model = None
    
    def create_open3d_lineset(self, vertice: list(), color=[1,0,0], zoffset=0):
        """Create Open3D Lineset from given list of vertice

        Args:
            vertice (list(list([x,y,z]))): List of lists containing points Cartesian coordinates [x,y,z]
            color (list, optional): Color [R, G, B]. Defaults to [1,0,0].

        Returns:
            LineSet: Open3D LineSet object
        """
        points = [[pt[0], pt[1], pt[2]+zoffset] for pt in vertice]
        lines = [[idx-1, idx] for idx in range(1, len(points))] # Create consective point as line
        colors = [color for i in range(len(lines))]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        return line_set


    def create_open3d_lineset_2d(self, vertice: list(), color=[1,0,0], default_z=0):
        """Create Open3D Lineset from given list of vertice

        Args:
            vertice (list(list([x,y]))): List of lists containing points Cartesian coordinates [x,y]
            color (list, optional): Color [R, G, B]. Defaults to [1,0,0].

        Returns:
            LineSet: Open3D LineSet object
        """
        points = [[pt[0], pt[1], default_z] for pt in vertice]
        lines = [[idx-1, idx] for idx in range(1, len(points))] # Create consective point as line
        colors = [color for i in range(len(lines))]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        return line_set
    
    
    def create_mesh(self, vertice: list(), triangle_idx: list(), color=[0, 0, 0]):
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertice), o3d.utility.Vector3iVector(triangle_idx))
        mesh.compute_vertex_normals() # # Important for lighting
        mesh.compute_triangle_normals() # Important for ray-tracing.
        if color == [0, 0, 0]:
            mesh.paint_uniform_color([np.random.random_sample(), np.random.random_sample(), np.random.random_sample()])        
            # mesh.vertex_colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(vtx), 3)))
        else:
            mesh.paint_uniform_color([1, 0, 0])        
        return mesh


    def create_poission_mesh(self, points: list()):
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        pcd.estimate_normals()
        pcd.orient_normals_to_align_with_direction()
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=6)
        mesh.compute_vertex_normals() # # Important for lighting
        mesh.compute_triangle_normals() # Important for ray-tracing.
        mesh.paint_uniform_color([np.random.random_sample(), np.random.random_sample(), np.random.random_sample()])        
        return mesh


    def project_points_on_meshes(self, meshes: list(), points: list(), zoffset=0.0):
        tscene = o3d.t.geometry.RaycastingScene()
        for mesh in meshes:
            tscene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
            
        query_points = o3d.core.Tensor(points, dtype=o3d.core.Dtype.Float32)
        ans = tscene.compute_closest_points(query_points)['points']
        ans = ans + o3d.core.Tensor([0.0, 0.0, zoffset], dtype=o3d.core.Dtype.Float32)
        
        return ans.numpy().tolist()
    
    
    def project_waypoints_on_meshes(self, meshes: list(), points: list(), zoffset=0.0, dir=(0,0,-1)):
        tscene = o3d.t.geometry.RaycastingScene()
        for mesh in meshes:
            tscene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
            
        pts = [[pt[0], pt[1], pt[2]+zoffset, dir[0], dir[1], dir[2]] for pt in points]
        rays = o3d.core.Tensor(pts, dtype=o3d.core.Dtype.Float32)
        result = tscene.cast_rays(rays)['t_hit'].numpy()
        ans = []
        for idx, pt in enumerate(points):
            height = result[idx]
            if np.isinf(height):
                height = 0#pt[2]
            ans.append([pt[0], pt[1], pt[2]+height])
        return ans
    
    
    def create_lidar_model(self, vfov = 30, hres=1, vres=1):
        direction = []
        for theta in np.arange(0, 2*np.pi, hres*np.pi/180.0):
            for phi in np.arange(-(vfov/2.0)*np.pi/180.0 + np.pi/2, (vfov/2.0)*np.pi/180.0 + np.pi/2, vres*np.pi/180.0):
                x = np.sin(phi) * np.cos(theta)
                y = np.sin(phi) * np.sin(theta)
                z = np.cos(phi)
                direction.append([x, y, z])
        self.lidar_model = direction
        # return direction


    def create_lidar_rays(self, scan_pos):
        if self.lidar_model is None:
            self.create_lidar_model(vfov = 30, hres=1, vres=1)
            
        pts = [[scan_pos[0], scan_pos[1], scan_pos[2], dir[0], dir[1], dir[2]] for dir in self.lidar_model]
         
        return o3d.core.Tensor(pts, dtype=o3d.core.Dtype.Float32)


    def lidar_scan(self, scene, scan_pos):
        rays = self.create_lidar_rays(scan_pos)
        ans = scene.cast_rays(rays)
        hit =  (ans['t_hit'] < 10) # ans['t_hit'].isfinite()
        points = rays[hit][:,:3] + rays[hit][:,3:] * ans['t_hit'][hit].reshape((-1,1))
        pcd = o3d.t.geometry.PointCloud(points)
        
        return pcd.to_legacy()
    
    
    def create_plane(self, width=1, height=1, color=(1,0,0)):
        hw = width/2.0
        hh = height/2.0
        vertices = []
        triangle_idx = []
        vertices.append([-hw, -hh, 0]) # 0:Lower left
        vertices.append([hw, -hh, 0]) # 1:Lower right
        vertices.append([hw, hh, 0]) # 2:Upper right
        vertices.append([-hw, hh, 0]) # 3:Upper left
        triangle_idx.append([0, 1, 2]) # First triangle, CCW order
        triangle_idx.append([0, 2, 3]) # First triangle, CCW order
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(triangle_idx))
        mesh.compute_vertex_normals() # For lighting
        mesh.compute_triangle_normals() # Important for ray-tracing.
        mesh.paint_uniform_color([1, 0, 0])        
        return mesh


    def create_scene(self, meshes):
        scene = o3d.t.geometry.RaycastingScene()
        for mesh in meshes:
            mesh_id = scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        return scene    


    def closest_point_on_meshes(self, meshes: list, points: list):
        scene = self.create_scene(meshes)
        query_pts = o3d.core.Tensor(points, dtype=o3d.core.Dtype.Float32)
        ans = scene.compute_closest_points(query_pts)['points']
        return ans.numpy().tolist(), scene.compute_distance(query_pts).numpy().tolist()


    def find_intersection_with_meshes(self, meshes: list, points: list, dir: tuple):
        scene = self.create_scene(meshes)
        pts = [[pt[0], pt[1], pt[2], dir[0], dir[1], dir[2]] for pt in points]
        rays = o3d.core.Tensor(pts, dtype=o3d.core.Dtype.Float32)
        ans = scene.cast_rays(rays)['t_hit']
        return ans.numpy().tolist()
    
    
    def show_meshes(self, meshes):
        o3d.visualization.draw_geometries(meshes, 
                                          mesh_show_back_face=True, 
                                          point_show_normal=True, 
                                          mesh_show_wireframe=False, 
                                          window_name=f"Playground: {len(meshes)}")
        