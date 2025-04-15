import os
import numpy as np
from PIL import Image
import open3d as o3d

# 参数配置
IMAGE_DIR = r"D:\Code\us_recon\data\test2"  # 图像目录
NUM_IMAGES = 12
ANGLE_STEP = 15  # 每张图像间隔角度
PIXEL_SPACING = 1.0  # mm per pixel
INTENSITY_THRESHOLD = 128  # 阈值判断白点
IMAGE_WIDTH = 240
IMAGE_HEIGHT = 200

# 输出路径
OUTPUT_PCD_PATH = r"D:\Code\us_recon\data\recon_points.ply"
OUTPUT_MESH_PATH = r"D:\Code\us_recon\data\recon_mesh.ply"

# 将单张图像转换为旋转后的 3D 点
def image_to_3d_points(img: Image.Image, angle_deg: float, pixel_spacing=0.5):
    img = np.array(img)
    points = []
    center_x = img.shape[1] // 2

    for v in range(img.shape[0]):
        for u in range(img.shape[1]):
            if img[v, u] > INTENSITY_THRESHOLD:
                x_offset = (u - center_x) * pixel_spacing
                z = v * pixel_spacing
                angle_rad = np.deg2rad(angle_deg)
                x = x_offset * np.cos(angle_rad)
                y = x_offset * np.sin(angle_rad)
                points.append([x, y, z])
    return points

if __name__ == "__main__":
    all_points = []

    # 遍历所有图像并收集 3D 点
    for i in range(NUM_IMAGES):
        filename = os.path.join(IMAGE_DIR, f"{i + 1:02}.BMP")
        if not os.path.exists(filename):
            print(f"未找到图像: {filename}")
            continue
        img = Image.open(filename).convert("L")
        angle_deg = i * ANGLE_STEP
        pts = image_to_3d_points(img, angle_deg, PIXEL_SPACING)
        print(f"✅ 3D 点已收集完成: {len(pts)} 个点")
        all_points.extend(pts)

    # 构建点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(np.array(all_points))

    # 保存点云
    o3d.io.write_point_cloud(OUTPUT_PCD_PATH, point_cloud)
    print(f"✅ 点云已保存至: {OUTPUT_PCD_PATH}")

    # 可视化点云
    o3d.visualization.draw_geometries([point_cloud], window_name="3D Point Cloud")

    # 表面重建（Poisson 或 Alpha shape）
    print("⏳ 正在进行Poisson重建...")
    # 去除噪声点

    # 法向量估计和方向修正
    # 去除噪声点
    pcd, _ = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=5.0)

    # 法向量估计和方向修正
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(k=10)

    # 泊松重建
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=3)

    # 保存 mesh
    o3d.io.write_triangle_mesh(OUTPUT_MESH_PATH, mesh)
    print(f"✅ Mesh 已保存至: {OUTPUT_MESH_PATH}")

    # 可视化 mesh
    o3d.visualization.draw_geometries([mesh], window_name="Reconstructed Mesh")
