import numpy as np
import open3d as o3d

# 加载点云
pcd = o3d.io.read_point_cloud(r"D:\Code\us_recon\data\output.ply")

# 计算点云的最近邻间距
distances = pcd.compute_nearest_neighbor_distance()
avg_distance = np.mean(distances)
min_distance = np.min(distances)
max_distance = np.max(distances)

print(f"最近邻最小距离: {min_distance}")
print(f"最近邻平均距离: {avg_distance}")
print(f"最近邻最大距离: {max_distance}")

# 设置探针半径范围（基于平均距离的倍数）
radius = 3 * avg_distance
radii = [radius, radius * 2]
print(f"自动选择的探针半径范围: {radii}")

# 使用 BPA 生成网格
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(k=10)

mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))

mesh.compute_vertex_normals()
mesh.triangle_material_ids = o3d.utility.IntVector([1])  # 假设所有材质共享双面
# mesh.vertex_colors  = o3d.utility.Vector3dVector(np.random.rand(len(mesh.vertices), 3))

# 保存和显示结果
o3d.io.write_triangle_mesh(r"D:\Code\us_recon\data\output_mesh.ply", mesh)
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# 创建线框显示
lines = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
num_lines = len(lines.lines)
lines.colors = o3d.utility.Vector3dVector(np.random.rand(num_lines, 3))  # 随机颜色

# 可视化
o3d.visualization.draw_geometries([lines], window_name="网格线框")
