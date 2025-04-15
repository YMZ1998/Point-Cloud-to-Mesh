import numpy as np
import trimesh

mesh = trimesh.load(r"D:\Code\us_recon\data\ring_mesh.ply")

faces = mesh.faces
vertices = mesh.vertices

v0 = vertices[faces[:, 0]]
v1 = vertices[faces[:, 1]]
v2 = vertices[faces[:, 2]]

# 法向判断（可选）
normals = np.cross(v1 - v0, v2 - v0)
normals /= np.linalg.norm(normals, axis=1, keepdims=True) + 1e-8
z_axis = np.array([0, 0, 1])
cos_angles = normals @ z_axis
normal_mask = cos_angles > -0.9  # 过滤掉方向差异大的面片

# 取交集
valid_mask = normal_mask

# 更新面片
mesh.update_faces(valid_mask)
mesh.remove_unreferenced_vertices()

mesh.export(r"D:\Code\us_recon\data\clean_simple_mesh.ply")
