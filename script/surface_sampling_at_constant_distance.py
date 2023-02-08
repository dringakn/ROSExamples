import numpy as np
import plotly.graph_objs as go

class SurfaceMesh:

    def __init__(self, x, y, z):
        self.vertex_coords = np.array([x, y, z]).T
        self.uv_coords = np.array([x, y]).T

    def get_vertex_indices(self, uv):
        """
        Returns the indices of the vertices that correspond to a given UV coordinate
        """
        i, j = np.searchsorted(self.uv_coords[:,0], uv[0]), np.searchsorted(self.uv_coords[:,1], uv[1])
        i, j = i-1, j-1
        return i, j

    def get_barycentric_weights(self, uv):
        """
        Returns the barycentric weights for a given UV coordinate
        """
        i, j = self.get_vertex_indices(uv)
        u, v = uv
        x1, y1 = self.uv_coords[i,0], self.uv_coords[j,1]
        x2, y2 = self.uv_coords[i+1,0], self.uv_coords[j+1,1]
        return (x2 - u) / (x2 - x1), (y2 - v) / (y2 - y1)
    
    def sample_points(self, distance, num_points):
        """
        sample points on the 3D surface at a constant distance
        """
        # Generate a set of random UV coordinates
        uv_samples = np.random.rand(num_points, 2)
        
        # Map the UV coordinates back to the 3D surface
        xyz_samples = np.zeros((num_points, 3))
        for i in range(num_points):
            uv = uv_samples[i]
            vertex_indices = self.get_vertex_indices(uv)
            weights = self.get_barycentric_weights(uv)
            xyz = np.zeros(3)
            for j in range(3):
                xyz += weights[j] * self.vertex_coords[vertex_indices[j]]
            xyz_samples[i] = xyz

        # Filter the points to keep only those that are at the desired distance
        filtered_points = []
        for point in xyz_samples:
            if np.linalg.norm(point) <= distance:
                filtered_points.append(point)
        return filtered_points



# Create a test surface
x, y = np.linspace(-5, 5, 50), np.linspace(-5, 5, 50)
X, Y = np.meshgrid(x, y)
Z = np.sin(np.sqrt(X**2 + Y**2))

# Create a SurfaceMesh object
surface_mesh = SurfaceMesh(X, Y, Z)

# Sample points on the surface at a constant distance
distance = 0.5
num_points = 100
points = surface_mesh.sample_points(distance, num_points)

# Create a 3D surface trace
surface = go.Surface(x=x, y=y, z=Z, colorscale='Viridis')

# Create a scatter trace for the sampled points
scatter = go.Scatter3d(x=[p[0] for p in points], y=[p[1] for p in points], z=[p[2] for p in points], mode='markers', marker=dict(size=3, color='red'))

# Create a layout for the 3D plot
layout = go.Layout(scene=dict(xaxis=dict(title='x'), yaxis=dict(title='y'), zaxis=dict(title='z')))

# Create a Figure object
fig = go.Figure(data=[surface, scatter], layout=layout)

# Show the figure
fig.show()
