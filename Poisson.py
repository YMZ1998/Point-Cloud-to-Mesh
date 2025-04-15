import numpy as np
import open3d as o3d

# 加载点云
pcd = o3d.io.read_point_cloud(r"D:\Code\us_recon\data\output_mesh.ply")

# 去除噪声点
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# 法向量估计和方向修正
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(k=10)

# 泊松重建
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=3)

# # 根据密度裁剪
# density_threshold = np.percentile(np.asarray(densities), 2)
# vertices_to_remove = densities < density_threshold
# mesh.remove_vertices_by_mask(vertices_to_remove)
#
# # 清理多余网格组件
# mesh = mesh.remove_unreferenced_vertices()
# mesh = mesh.remove_degenerate_triangles()
# mesh = mesh.remove_duplicated_triangles()
# mesh = mesh.remove_duplicated_vertices()

# 为顶点设置灰色
num_vertices = len(mesh.vertices)
gray_color = [0.5, 0.5, 0.5]  # 灰色 RGB 值
mesh.vertex_colors = o3d.utility.Vector3dVector(np.tile(gray_color, (num_vertices, 1)))
# mesh = mesh.paint_uniform_color(np.array([0,255,0]))

bbox = pcd.get_axis_aligned_bounding_box()
print(bbox)
print(mesh.get_axis_aligned_bounding_box())
mesh = mesh.crop(bbox)
print(mesh.get_axis_aligned_bounding_box())

# 保存和显示结果
o3d.io.write_triangle_mesh(r"D:\Code\us_recon\data\output_mesh_poisson.ply", mesh)
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# 创建线框显示
lines = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
num_lines = len(lines.lines)
lines.colors = o3d.utility.Vector3dVector(np.random.rand(num_lines, 3))  # 随机颜色

# 可视化
o3d.visualization.draw_geometries([lines], window_name="网格线框")
