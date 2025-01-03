import numpy as np
import open3d as o3d
from scipy.spatial import Delaunay
import pyvista as pv

# Step 1: Read the point cloud from a .ply file
point_cloud = o3d.io.read_point_cloud(r"D:\Code\us_recon\data\output.ply")
points = np.asarray(point_cloud.points)

# Check the number of points
print(f"Number of points in the point cloud: {len(points)}")

# Step 2: Apply Delaunay triangulation (only works for 3D points, this creates tetrahedra)
delaunay = Delaunay(points)

# Step 3: Check Delaunay simplices (tetrahedra) and ensure valid face data
simplices = delaunay.simplices
if len(simplices) == 0:
    print("Delaunay triangulation failed, no simplices found.")
else:
    print(f"Number of simplices: {len(simplices)}")

# Step 4: Ensure the faces array is properly formatted for PyVista (it needs to be flattened)
# The faces array should have a size of (N * 4,) where N is the number of tetrahedra
# Each tetrahedron face has 4 vertices
faces = simplices.flatten()
print(simplices)


# Step 5: Create the 3D mesh using PyVista
mesh = pv.PolyData(points)
mesh.faces = faces

# Step 6: Visualize the 3D mesh using PyVista
plotter = pv.Plotter()
plotter.add_mesh(mesh, color="lightblue", show_edges=True)
plotter.show()
