import numpy as np
import open3d as o3d
import alphashape
import matplotlib.pyplot as plt

# 读取 2D 点云
file_path = r"D:\Code\us_recon\data\output.ply"
pcd = o3d.io.read_point_cloud(file_path)
points = np.asarray(pcd.points)

# 只取 XY 平面点
points_2d = points[:, :2]  # 忽略 Z 轴

# 计算 Alpha Shape
alpha = 0.1  # 控制边界曲率，值越小边界越紧密
alpha_shape = alphashape.alphashape(points_2d, alpha)

# 可视化
fig, ax = plt.subplots()
ax.scatter(points_2d[:, 0], points_2d[:, 1], s=5, c="blue")  # 原始点
ax.add_patch(plt.Polygon(alpha_shape.exterior.coords, fill=None, edgecolor='red'))  # Alpha Shape 边界
plt.show()
