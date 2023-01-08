import open3d as o3d
import numpy as np

class PolygonOpen3D:
    
    def __init__(self):
        pass
    
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


    def create_mesh(self, vertice: list(), triangle_idx: list(), color=[0, 0, 0]):
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertice), o3d.utility.Vector3iVector(triangle_idx))
        mesh.compute_vertex_normals()
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
        mesh.paint_uniform_color([np.random.random_sample(), np.random.random_sample(), np.random.random_sample()])        
        return mesh