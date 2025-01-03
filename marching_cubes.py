file_path = r"D:\Code\us_recon\data\output.ply"
output_path = r"D:\Code\us_recon\data\output_mesh.ply"

import numpy as np
import pyvista as pv
from skimage import measure
import open3d as o3d


# 1. 生成模拟数据：球体的点云
def generate_sphere_points(radius=1.0, num_points=5000):
    # 使用PyVista生成一个球体模型
    sphere = pv.Sphere(radius=radius, theta_resolution=30, phi_resolution=30)
    sphere.plot()

    # 随机抽取点云
    points = sphere.points
    # 随机选择部分点生成稀疏点云
    indices = np.random.choice(points.shape[0], size=num_points, replace=True)
    point_cloud = points[indices]

    return point_cloud


# 2. 体素化点云
def voxelize_points(points, voxel_size=0.1):
    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)

    voxel_grid_shape = np.ceil((max_bound - min_bound) / voxel_size).astype(int)
    voxel_grid = np.zeros(voxel_grid_shape, dtype=bool)

    voxel_indices = np.floor((points - min_bound) / voxel_size).astype(int)
    voxel_indices = np.clip(voxel_indices, 0, voxel_grid_shape - 1)

    for idx in voxel_indices:
        voxel_grid[tuple(idx)] = True

    return voxel_grid, min_bound, voxel_size


# 3. 使用Marching Cubes提取网格
def extract_mesh_from_voxel_grid(voxel_grid, min_bound, voxel_size):
    verts, faces, _, _ = measure.marching_cubes(voxel_grid, level=0.05)
    # import mcubes
    # verts, faces = mcubes.marching_cubes(voxel_grid, 0)

    if verts.shape[0] == 0:
        print("Marching Cubes没有生成网格！")
        return None

    verts = verts * voxel_size + min_bound

    mesh = pv.PolyData(verts)
    mesh.faces = np.hstack([3 * np.ones((faces.shape[0], 1), dtype=int), faces])

    return mesh


# 4. 主函数：将点云转换为网格并保存
def point_cloud_to_mesh_and_save(radius=1.0, num_points=5000, voxel_size=0.001, output_file="output_mesh.obj"):
    # 生成模拟数据：球体点云
    points = generate_sphere_points(radius=radius, num_points=num_points)

    # 体素化点云
    voxel_grid, min_bound, voxel_size = voxelize_points(points, voxel_size)

    # 使用Marching Cubes提取网格
    mesh = extract_mesh_from_voxel_grid(voxel_grid, min_bound, voxel_size)

    mesh.plot()
    if mesh:
        # 保存网格为文件
        mesh.save(output_file)
        print(f"网格已保存为 {output_file}")
    else:
        print("没有生成网格，检查点云质量和体素化参数。")


# 运行并生成网格
# point_cloud_to_mesh_and_save(radius=1.0, num_points=2000, voxel_size=0.1, output_file=output_path)

import numpy as np
import mcubes

# Create a data volume (30 x 30 x 30)
X, Y, Z = np.mgrid[:30, :30, :30]
u = (X - 15) ** 2 + (Y - 15) ** 2 + (Z - 15) ** 2 - 8 ** 2
print(u.shape)
point_cloud = o3d.io.read_point_cloud(r"D:\Code\us_recon\data\output.ply")
points = np.asarray(point_cloud.points)
print(points.shape)

# Extract the 0-isosurface
# vertices, triangles = mcubes.marching_cubes(u, 0)
vertices, triangles, _, _ = measure.marching_cubes(u, level=0.05)

# Export the result to sphere.dae
mcubes.export_obj(vertices, triangles, "sphere.obj")

